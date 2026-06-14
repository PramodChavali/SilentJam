/**
 * silent_jam.cpp — YIN pitch-to-headphones, smoothed & low-latency
 *
 * This is the lean version (no recording yet — that gets merged back later).
 * Focus: minimal input delay, smooth/clear tone, no volume throb.
 *
 * What changed vs the previous version of this file, and why:
 *
 *   1. SMALLER OUTPUT BLOCK  (OUT_BLOCK 512 -> 128)
 *      The output block was ~23 ms at 22050 Hz and was the single biggest
 *      source of delay. Matching it to the input hop (128) cuts output latency
 *      to ~6 ms and lets pitch/amplitude update 4x more often.
 *
 *   2. PER-SAMPLE SMOOTHING  (was per-block)
 *      Frequency, amplitude envelope, and dynamics are now interpolated every
 *      sample. Previously they stepped once per 512-sample block, so the phase
 *      increment jumped at block boundaries — that periodic discontinuity is
 *      what made the volume rise and fall like a wave, and what made notes
 *      sound choppy. Per-sample updates remove both.
 *
 *   3. CONTINUOUS PHASE  (no per-note reset)
 *      Phase is never snapped back to 0 on note-off; it keeps running and the
 *      amplitude envelope fades the sound out instead. Resetting phase mid-tone
 *      caused clicks.
 *
 *   4. GENTLE DYNAMICS
 *      Output loudness follows a *slowly smoothed* input level, mapped to a
 *      narrow range with a floor — so playing harder/softer comes through as a
 *      clean swell, not the jittery, noisy amplitude tracking from before.
 *
 *   5. SLIGHTLY RELAXED GATE
 *      CONF_PLAY 0.90 -> 0.85 and a short hold so notes stop flickering in and
 *      out when confidence wobbles on the in-bell mic.
 *
 * Build (Linux / Raspberry Pi):
 *   g++ -O2 -std=c++17 silent_jam.cpp -lportaudio -laubio -lm -o silent_jam
 *   (libsndfile not needed in this lean version)
 */

#include <aubio/aubio.h>
#include <portaudio.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

// ─────────────────────────── SETTINGS ─────────────────────────────────────

static constexpr int    SR          = 22050;
static constexpr int    HOP_SIZE    = 128;   // aubio hop / input block
static constexpr int    BUFFER_SIZE = 1024;  // aubio analysis window (was 512;
                                             // larger = steadier low notes)
static constexpr int    OUT_BLOCK   = 128;   // output block — matched to hop to
                                             // minimise latency (was 512)

static constexpr int    INPUT_DEVICE  = 1;   // adjust to your USB mic index
static constexpr int    OUTPUT_DEVICE = 0;   // adjust to your headphone index

// ─────────────────────── TUNABLE THRESHOLDS ───────────────────────────────

static constexpr double CONF_DETECT = 0.60;
static constexpr double CONF_PLAY   = 0.85;  // was 0.90 — fewer dropouts
static constexpr double LEVEL_PLAY  = -20.0; // dBFS
static constexpr double HOLD_TIME   = 0.08;  // s — hold note briefly through
                                             // confidence dips (was 0.30, which
                                             // made note-offs feel laggy)

// ─────────────────────── SYNTHESIS / SMOOTHING ────────────────────────────
// One-pole time constants (seconds). Converted to per-sample coefficients.
static constexpr double GLIDE_TIME   = 0.010; // pitch glide between notes
static constexpr double ENV_ATTACK   = 0.006; // amplitude attack
static constexpr double ENV_RELEASE  = 0.040; // amplitude release

// Dynamics: output loudness follows input level, slowly, within a floor..1 band
static constexpr double DYN_SMOOTH   = 0.150; // 150 ms loudness tracking
static constexpr double DYN_DB_SOFT  = -30.0; // input dB mapped to floor gain
static constexpr double DYN_DB_LOUD  = -10.0; // input dB mapped to full gain
static constexpr double DYN_GAIN_MIN = 0.45;  // softest playing = 45% volume

static constexpr double OUTPUT_LEVEL = 0.25;  // overall output headroom

// ─────────────────────── PER-SAMPLE COEFFS ────────────────────────────────
static inline double one_pole(double tau_s) {
    return 1.0 - std::exp(-1.0 / (tau_s * SR));
}

// ─────────────────────── THREAD-SAFE STATE ────────────────────────────────
static std::atomic<double> g_target_freq{0.0};
static std::atomic<double> g_dyn_gain   {1.0};   // 0..1 smoothed loudness

static inline double get_target()        { return g_target_freq.load(std::memory_order_relaxed); }
static inline void   set_target(double f){ g_target_freq.store(f, std::memory_order_relaxed); }

// ─────────────────────── OUTPUT / INPUT STATE ─────────────────────────────
struct OutputState {
    double phase        = 0.0;
    double current_freq = 0.0;
    double amplitude    = 0.0;   // 0..1 on/off envelope
    double dyn          = 1.0;   // 0..1 smoothed dynamics gain
};

struct InputState {
    aubio_pitch_t* pitch_obj = nullptr;
    fvec_t*        in_buf    = nullptr;
    fvec_t*        out_buf   = nullptr;
    double last_pitch     = 0.0;
    double last_good_time = 0.0;
};

// ─────────────────────── WAVEFORM ─────────────────────────────────────────
// Additive trumpet-ish timbre. Partial amplitudes sum to ~2.35 at peak, so we
// divide by that sum to keep the waveform within roughly [-1, 1]. Without this
// normalisation the signal slams into the tanh below and hard-clips into a
// near-square wave — that was the source of the static/crackle, and the
// phase-beating of the partials against the clipper caused the volume to waver.
static constexpr double HARM_SUM = 1.00 + 0.60 + 0.35 + 0.20 + 0.12 + 0.08;
static inline double trumpet_wave(double phase) {
    double v = 1.00 * std::sin(phase)
             + 0.60 * std::sin(2.0 * phase)
             + 0.35 * std::sin(3.0 * phase)
             + 0.20 * std::sin(4.0 * phase)
             + 0.12 * std::sin(5.0 * phase)
             + 0.08 * std::sin(6.0 * phase);
    return v / HARM_SUM;   // now in ~[-1, 1]
}

static double perf_counter() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ─────────────────────── OUTPUT CALLBACK ──────────────────────────────────
static int output_callback(
    const void*, void* output, unsigned long frames,
    const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags statusFlags, void* userData)
{
    auto* st  = static_cast<OutputState*>(userData);
    auto* out = static_cast<float*>(output);

    // Diagnostic: if this prints "X" while you hear crackle, the crackle is
    // buffer underruns (CPU can't keep up at this block size), NOT synthesis.
    // In that case raise OUT_BLOCK (e.g. 256) to give more headroom. Remove
    // this line once diagnosed.
    if (statusFlags & paOutputUnderflow) { std::fputc('X', stderr); std::fflush(stderr); }

    const double target   = get_target();
    const double dyn_targ = g_dyn_gain.load(std::memory_order_relaxed);
    const bool   want     = (target > 0.0);

    // Per-sample smoothing coefficients (computed once; cheap).
    static const double glide_a = one_pole(GLIDE_TIME);
    static const double atk_a   = one_pole(ENV_ATTACK);
    static const double rel_a   = one_pole(ENV_RELEASE);
    static const double dyn_a   = one_pole(DYN_SMOOTH);
    static const double two_pi  = 2.0 * M_PI;

    for (unsigned long i = 0; i < frames; ++i) {
        // 1. Glide frequency toward target every sample (no block-edge jumps).
        if (want) {
            if (st->current_freq < 1.0) st->current_freq = target; // snap 1st note
            else st->current_freq += glide_a * (target - st->current_freq);
        }

        // 2. On/off amplitude envelope, per sample.
        double amp_target = want ? 1.0 : 0.0;
        double amp_a = (amp_target > st->amplitude) ? atk_a : rel_a;
        st->amplitude += amp_a * (amp_target - st->amplitude);

        // 3. Dynamics gain glides toward the (already slow-smoothed) input level.
        st->dyn += dyn_a * (dyn_targ - st->dyn);

        // 4. Synthesise this sample.
        double sample = 0.0;
        if (st->current_freq > 1.0 && st->amplitude > 1e-4) {
            sample = trumpet_wave(st->phase);                  // ~[-1, 1]
            // Gentle saturation for warmth + headroom. Drive kept low (1.0) so
            // tanh only softens peaks instead of clipping the tone to a square.
            sample = std::tanh(sample * 1.0) * OUTPUT_LEVEL;
            sample *= st->amplitude * st->dyn;                 // envelope + dynamics

            st->phase += two_pi * st->current_freq / SR;
            if (st->phase >= two_pi) st->phase -= two_pi;      // wrap at 2*pi
        }

        float s = static_cast<float>(sample);
        out[2 * i]     = s;   // L
        out[2 * i + 1] = s;   // R
    }

    return paContinue;
}

// ─────────────────────── INPUT CALLBACK ───────────────────────────────────
// Rolling window into aubio so low notes detect reliably, plus a slow dynamics
// level for the output to follow.
static float g_ring[BUFFER_SIZE] = {};
static int   g_ring_pos = 0;

static int input_callback(
    const void* input, void*, unsigned long frames,
    const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void* userData)
{
    auto*        st = static_cast<InputState*>(userData);
    const float* in = static_cast<const float*>(input);

    unsigned long n = std::min(static_cast<unsigned long>(HOP_SIZE), frames);

    // Push hop into the rolling history, then hand aubio the whole window.
    for (unsigned long i = 0; i < n; ++i) {
        g_ring[g_ring_pos] = in[i];
        g_ring_pos = (g_ring_pos + 1) % BUFFER_SIZE;
    }
    for (int i = 0; i < BUFFER_SIZE; ++i)
        st->in_buf->data[i] = g_ring[(g_ring_pos + i) % BUFFER_SIZE];

    double now = perf_counter();

    aubio_pitch_do(st->pitch_obj, st->in_buf, st->out_buf);
    double pitch = static_cast<double>(st->out_buf->data[0]);
    double conf  = static_cast<double>(aubio_pitch_get_confidence(st->pitch_obj));

    // ── Pitch stabilisation ───────────────────────────────────────────────
    // On a distorted in-bell signal, aubio's estimate can jump to an octave
    // (or fifth) of the true note for a hop or two, even on a held note. Those
    // brief jumps make the synth lurch in frequency — heard as a gritty crackle
    // while the average pitch still sounds steady. We reject a new estimate that
    // is suspiciously close to 2x/0.5x (octave) or wildly far from the last
    // stable pitch, unless it persists. This keeps the target rock-steady.
    static double stable_pitch = 0.0;
    static int    jump_count   = 0;
    if (pitch > 0.0 && conf > CONF_DETECT) {
        bool accept = true;
        if (stable_pitch > 0.0) {
            double ratio = pitch / stable_pitch;
            // Treat near-octave and near-half jumps, or >7% sudden deviations,
            // as suspect. Require 3 consecutive hops agreeing before accepting,
            // so a real new note still comes through quickly (~17 ms) but a
            // one-hop glitch is ignored.
            bool octave_jump = (ratio > 1.8 && ratio < 2.2) ||
                               (ratio > 0.45 && ratio < 0.55);
            bool big_jump    = (ratio > 1.07 || ratio < 0.93);
            if (octave_jump || big_jump) {
                if (++jump_count < 3) accept = false;  // hold off, likely glitch
            } else {
                jump_count = 0;
            }
        }
        if (accept) { stable_pitch = pitch; jump_count = 0; }
    }
    double use_pitch = stable_pitch;

    // Level in dBFS (this hop).
    double rms = 0.0;
    for (unsigned long i = 0; i < n; ++i) rms += (double)in[i] * in[i];
    rms /= (double)n;
    double db = 10.0 * std::log10(rms + 1e-12);

    // Dynamics: slowly smooth dB, map to [DYN_GAIN_MIN, 1]. Slow smoothing is
    // what keeps the resulting volume steady instead of chasing mic jitter.
    static double dyn_db = DYN_DB_SOFT;
    static const double dyn_a = 1.0 - std::exp(-1.0 / (DYN_SMOOTH * SR / HOP_SIZE));
    dyn_db += dyn_a * (db - dyn_db);
    double t = (dyn_db - DYN_DB_SOFT) / (DYN_DB_LOUD - DYN_DB_SOFT);
    t = std::max(0.0, std::min(1.0, t));
    g_dyn_gain.store(DYN_GAIN_MIN + (1.0 - DYN_GAIN_MIN) * t,
                     std::memory_order_relaxed);

    // Gate.
    if (use_pitch > 0.0) {
        st->last_pitch     = use_pitch;
        st->last_good_time = now;
    }
    bool held = (now - st->last_good_time) < HOLD_TIME;
    if (conf > CONF_PLAY && db > LEVEL_PLAY && held && st->last_pitch > 0.0) {
        set_target(st->last_pitch);
    } else {
        set_target(0.0);
    }

    return paContinue;
}

// ─────────────────────── MAIN ─────────────────────────────────────────────
int main(int argc, char* argv[]) {
    const char* method = (argc > 1) ? argv[1] : "yinfast";

    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::fprintf(stderr, "PortAudio init error: %s\n", Pa_GetErrorText(err));
        return 1;
    }

    InputState in_state;
    in_state.in_buf   = new_fvec(BUFFER_SIZE);
    in_state.out_buf  = new_fvec(1);
    in_state.pitch_obj = new_aubio_pitch(method, BUFFER_SIZE, HOP_SIZE, SR);
    aubio_pitch_set_unit(in_state.pitch_obj, "Hz");
    aubio_pitch_set_silence(in_state.pitch_obj, -40.0f);

    OutputState out_state;

    std::printf("Silent Jam (lean)\n");
    std::printf("  Sample rate   : %d Hz\n", SR);
    std::printf("  Analysis win  : %d samples\n", BUFFER_SIZE);
    std::printf("  Hop / out blk : %d / %d samples\n", HOP_SIZE, OUT_BLOCK);
    std::printf("  Pitch method  : %s\n\n", method);

    int nd = Pa_GetDeviceCount();
    std::printf("── Audio devices ──\n");
    for (int i = 0; i < nd; ++i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        std::printf("  [%d] %s  in=%d out=%d\n", i, d->name,
                    d->maxInputChannels, d->maxOutputChannels);
    }
    std::printf("\nUsing input=%d output=%d\n\n", INPUT_DEVICE, OUTPUT_DEVICE);

    PaStreamParameters in_params{};
    in_params.device           = INPUT_DEVICE;
    in_params.channelCount     = 1;
    in_params.sampleFormat     = paFloat32;
    in_params.suggestedLatency = Pa_GetDeviceInfo(INPUT_DEVICE)->defaultLowInputLatency;

    PaStreamParameters out_params{};
    out_params.device           = OUTPUT_DEVICE;
    out_params.channelCount     = 2;
    out_params.sampleFormat     = paFloat32;
    out_params.suggestedLatency = Pa_GetDeviceInfo(OUTPUT_DEVICE)->defaultLowOutputLatency;

    PaStream* in_stream = nullptr;
    err = Pa_OpenStream(&in_stream, &in_params, nullptr, SR, HOP_SIZE,
                        paClipOff, input_callback, &in_state);
    if (err != paNoError) {
        std::fprintf(stderr, "Input stream: %s\n", Pa_GetErrorText(err));
        Pa_Terminate(); return 1;
    }

    PaStream* out_stream = nullptr;
    err = Pa_OpenStream(&out_stream, nullptr, &out_params, SR, OUT_BLOCK,
                        paClipOff, output_callback, &out_state);
    if (err != paNoError) {
        std::fprintf(stderr, "Output stream: %s\n", Pa_GetErrorText(err));
        Pa_CloseStream(in_stream); Pa_Terminate(); return 1;
    }

    const PaStreamInfo* ii = Pa_GetStreamInfo(in_stream);
    const PaStreamInfo* oi = Pa_GetStreamInfo(out_stream);
    std::printf("── Latency ──\n  In : %.2f ms\n  Out: %.2f ms\n  Tot: %.2f ms\n\n",
                ii->inputLatency * 1000.0, oi->outputLatency * 1000.0,
                (ii->inputLatency + oi->outputLatency) * 1000.0);

    Pa_StartStream(in_stream);
    Pa_StartStream(out_stream);

    std::puts("Playing. Press Enter to quit.");
    std::getchar();

    Pa_StopStream(in_stream);  Pa_StopStream(out_stream);
    Pa_CloseStream(in_stream); Pa_CloseStream(out_stream);
    Pa_Terminate();
    del_aubio_pitch(in_state.pitch_obj);
    del_fvec(in_state.in_buf);
    del_fvec(in_state.out_buf);
    aubio_cleanup();
    return 0;
}