/*
 * trumpet_synth.cpp  –  v3: improved in-process synthesis
 *
 * Key improvements over v1:
 *   1. Pitch-adaptive harmonic amplitudes  — harmonic ratios shift with frequency
 *      to match a real trumpet's acoustic behaviour (brighter high notes,
 *      rounder low notes)
 *   2. Smooth frequency glide              — phase-continuous pitch interpolation
 *      between detected notes; no clicks on note changes
 *   3. RMS-driven amplitude               — output volume tracks how hard you play
 *   4. Breath noise layer                 — bandpass-filtered noise mixed under
 *      the tone, scaled with RMS, gives attack transient texture
 *   5. Soft note on/off envelope          — smooth ramp on gate open/close
 *   6. Same gate logic as your v1         — PLAY_CONF_THRESHOLD + PLAY_LEVEL_DB
 *      with HOLD_TIME to prevent mid-note dropouts
 *
 * Latency: same as v1 — pitch detect and synthesis both happen per-callback,
 * no inter-thread audio buffering.
 *
 * Build:
 *   g++ -O2 -o trumpet_synth trumpet_synth.cpp -lportaudio -laubio -lm
 *
 * Run:
 *   ./trumpet_synth [pitch_method]   (default: yinfast)
 */

#include <aubio/aubio.h>
#include <portaudio.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <algorithm>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

// ──────────────────────────────────────────────
//  SETTINGS — tune these on the Pi
// ──────────────────────────────────────────────
static constexpr int    SR            = 44100;
static constexpr int    BUFFER_SIZE   = 256;
static constexpr int    HOP_SIZE      = 128;
static constexpr int    INPUT_DEVICE  = 1;
static constexpr int    OUTPUT_DEVICE = 0;

// Gate (same as your v1)
static constexpr float  CONF_THRESHOLD      = 0.5f;
static constexpr float  PLAY_CONF_THRESHOLD = 0.8f;
static constexpr float  PLAY_LEVEL_DB       = -10.0f;
static constexpr float  HOLD_TIME_S         = 0.05f;

// Synthesis
static constexpr float  OUTPUT_GAIN         = 0.30f;   // raise if too quiet
static constexpr float  BREATH_MIX          = 0.06f;   // 0 = no breath noise, 0.15 = quite breathy
static constexpr float  GLIDE_TIME_S        = 0.012f;  // pitch glide between notes (seconds)
static constexpr float  ENV_ATTACK_S        = 0.004f;  // amplitude attack
static constexpr float  ENV_RELEASE_S       = 0.060f;  // amplitude release

// Telemetry
static constexpr int    TELEM_INTERVAL      = 100;

// ──────────────────────────────────────────────
//  SHARED STATE  (input callback → output callback)
// ──────────────────────────────────────────────
static std::atomic<float>  g_freq         {0.0f};
static std::atomic<float>  g_rms_lin      {0.0f};
static std::atomic<double> g_last_good_ts {0.0};
static std::atomic<double> g_last_pitch   {0.0};

// ──────────────────────────────────────────────
//  TELEMETRY
// ──────────────────────────────────────────────
struct TelemSnapshot { float avg_db, avg_conf, avg_hz; int gate_pct; };
static std::atomic<bool> g_telem_ready {false};
static TelemSnapshot     g_telem_snap  {};
static int   telem_count    = 0;
static float telem_db_sum   = 0.0f;
static float telem_conf_sum = 0.0f;
static float telem_hz_sum   = 0.0f;
static int   telem_gate_sum = 0;

// ──────────────────────────────────────────────
//  AUBIO
// ──────────────────────────────────────────────
static aubio_pitch_t* pitch_obj = nullptr;
static fvec_t*        in_buf    = nullptr;
static fvec_t*        pitch_out = nullptr;

// ──────────────────────────────────────────────
//  DSP HELPERS
// ──────────────────────────────────────────────

// One-pole IIR smoothing coefficient from time constant (seconds)
static inline float smooth_coeff(float tau_s) {
    return 1.0f - expf(-1.0f / (tau_s * (float)SR));
}

// Pitch-adaptive harmonic amplitudes.
//
// A real trumpet's spectrum changes with pitch:
//   - Low register (around Bb3, ~233 Hz): fundamental dominates, warm/round
//   - High register (around Bb5, ~932 Hz): upper harmonics are much stronger, bright/brassy
//
// We interpolate between two measured-ish amplitude tables.
// harmonics[k] = amplitude of (k+1)th partial (k=0 is fundamental)
static constexpr int N_HARM = 8;

//                                    f1     f2     f3     f4     f5     f6     f7     f8
static constexpr float HARM_LOW[N_HARM]  = { 1.00f, 0.50f, 0.30f, 0.15f, 0.08f, 0.04f, 0.02f, 0.01f };
static constexpr float HARM_HIGH[N_HARM] = { 0.70f, 0.80f, 0.60f, 0.45f, 0.30f, 0.18f, 0.10f, 0.06f };

static void harm_amps(float freq, float out[N_HARM]) {
    float t = (freq - 200.0f) / (1000.0f - 200.0f);
    t = std::max(0.0f, std::min(1.0f, t));
    float sum = 0.0f;
    for (int k = 0; k < N_HARM; ++k) {
        out[k] = HARM_LOW[k] * (1.0f - t) + HARM_HIGH[k] * t;
        sum += out[k];
    }
    if (sum > 0.0f) for (int k = 0; k < N_HARM; ++k) out[k] /= sum;
}

// Fast LCG white noise (no stdlib calls in audio thread)
static uint32_t lcg_state = 0x12345678u;
static inline float white_noise() {
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return ((int32_t)lcg_state) * (1.0f / 2147483648.0f);
}

// Simple bandpass via two one-pole low-passes subtracted
// Centre ~800 Hz, bandwidth ~500 Hz — sits in the trumpet breath range
struct BandpassState {
    float lp1 = 0.0f, lp2 = 0.0f;
    static constexpr float A1 = 0.11f;
    static constexpr float A2 = 0.047f;
    inline float process(float x) {
        lp1 += A1 * (x - lp1);
        lp2 += A2 * (x - lp2);
        return lp1 - lp2;
    }
};

// ──────────────────────────────────────────────
//  INPUT CALLBACK
// ──────────────────────────────────────────────
static int input_callback(
        const void* inputBuffer, void*,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*)
{
    const float* in = static_cast<const float*>(inputBuffer);

    // 1. Feed aubio
    unsigned long n = std::min(framesPerBuffer, (unsigned long)HOP_SIZE);
    for (unsigned long i = 0; i < n; ++i) in_buf->data[i] = in[i];
    for (unsigned long i = n; i < (unsigned long)HOP_SIZE; ++i) in_buf->data[i] = 0.0f;

    // 2. Pitch detect
    aubio_pitch_do(pitch_obj, in_buf, pitch_out);
    float pitch = fvec_get_sample(pitch_out, 0);
    float conf  = aubio_pitch_get_confidence(pitch_obj);

    // 3. RMS + smoothed envelope
    float rms_sq = 0.0f;
    for (unsigned long i = 0; i < n; ++i) rms_sq += in[i] * in[i];
    float rms_lin = sqrtf(rms_sq / (float)n);
    float db      = 20.0f * log10f(rms_lin + 1e-12f);

    static float env_rms = 0.0f;
    static const float alpha_a = smooth_coeff(ENV_ATTACK_S);
    static const float alpha_r = smooth_coeff(ENV_RELEASE_S);
    float alpha = (rms_lin > env_rms) ? alpha_a : alpha_r;
    env_rms = alpha * rms_lin + (1.0f - alpha) * env_rms;
    g_rms_lin.store(env_rms, std::memory_order_relaxed);

    // 4. Gate (identical logic to v1, plus hold time)
    static double cb_time = 0.0;
    cb_time += (double)HOP_SIZE / SR;

    if (conf > CONF_THRESHOLD && pitch > 50.0f && pitch < 2000.0f) {
        g_last_pitch.store((double)pitch, std::memory_order_relaxed);
        g_last_good_ts.store(cb_time,     std::memory_order_relaxed);
    }
    bool pitch_valid = (cb_time - g_last_good_ts.load(std::memory_order_relaxed)) < HOLD_TIME_S;
    bool gate_open   = (conf > PLAY_CONF_THRESHOLD) && (db > PLAY_LEVEL_DB) && pitch_valid;

    g_freq.store(gate_open
        ? (float)g_last_pitch.load(std::memory_order_relaxed)
        : 0.0f,
        std::memory_order_relaxed);

    // 5. Telemetry
    telem_db_sum   += db;
    telem_conf_sum += conf;
    if (gate_open) { telem_hz_sum += pitch; telem_gate_sum++; }
    if (++telem_count >= TELEM_INTERVAL && !g_telem_ready.load()) {
        float fn = (float)TELEM_INTERVAL;
        g_telem_snap = {
            telem_db_sum / fn,
            telem_conf_sum / fn,
            telem_gate_sum > 0 ? telem_hz_sum / telem_gate_sum : 0.0f,
            (int)(100.0f * telem_gate_sum / fn)
        };
        g_telem_ready.store(true, std::memory_order_release);
        telem_count = 0; telem_db_sum = telem_conf_sum = telem_hz_sum = 0.0f; telem_gate_sum = 0;
    }

    return paContinue;
}

// ──────────────────────────────────────────────
//  OUTPUT CALLBACK  — synthesis
// ──────────────────────────────────────────────
static int output_callback(
        const void*, void* outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*)
{
    float* out = static_cast<float*>(outputBuffer);

    float target_freq = g_freq.load(std::memory_order_relaxed);
    float rms         = g_rms_lin.load(std::memory_order_relaxed);

    // ── Persistent synthesis state ──
    static float phases[N_HARM]   = {};
    static float cur_freq         = 0.0f;
    static float env_amp          = 0.0f;
    static float cached_freq      = 0.0f;
    static float harm[N_HARM]     = {};
    static BandpassState bp;

    static const float glide_alpha = smooth_coeff(GLIDE_TIME_S);
    static const float amp_alpha_a = smooth_coeff(ENV_ATTACK_S);
    static const float amp_alpha_r = smooth_coeff(ENV_RELEASE_S);

    bool playing = (target_freq > 0.0f);

    for (unsigned long i = 0; i < framesPerBuffer; ++i) {

        // 1. Glide frequency toward target (phase-continuous — no clicks)
        if (playing) {
            if (cur_freq < 10.0f) cur_freq = target_freq;  // snap on first note
            cur_freq += glide_alpha * (target_freq - cur_freq);
        }

        // 2. Amplitude envelope follows RMS of the player's input
        //    Scale rms to a useful range: rms ~0.01 (soft) to ~0.1 (loud)
        float amp_target = playing ? std::max(0.01f, rms * 8.0f) : 0.0f;
        float amp_alpha  = (amp_target > env_amp) ? amp_alpha_a : amp_alpha_r;
        env_amp += amp_alpha * (amp_target - env_amp);

        // 3. Update harmonic table when pitch changes by more than ~3%
        if (fabsf(cur_freq - cached_freq) > cached_freq * 0.03f + 1.0f) {
            harm_amps(cur_freq, harm);
            cached_freq = cur_freq;
        }

        // 4. Additive synthesis
        float sample = 0.0f;
        if (cur_freq > 10.0f) {
            float phase_inc_fund = 2.0f * (float)M_PI * cur_freq / (float)SR;
            for (int k = 0; k < N_HARM; ++k) {
                if ((k + 1) * cur_freq >= (float)SR * 0.45f) break;  // skip above Nyquist
                phases[k] += (float)(k + 1) * phase_inc_fund;
                if (phases[k] > (float)M_PI) phases[k] -= 2.0f * (float)M_PI;
                sample += harm[k] * sinf(phases[k]);
            }
        }

        // 5. Breath noise (bandpass filtered, mixed under tone)
        sample += bp.process(white_noise()) * BREATH_MIX * env_amp;

        // 6. Envelope + gain + soft clip (tanh used lightly here, not for distortion)
        sample *= env_amp * OUTPUT_GAIN;
        sample  = tanhf(sample * 0.8f) / 0.8f;  // headroom protection only

        out[2 * i]     = sample;
        out[2 * i + 1] = sample;
    }

    return paContinue;
}

// ──────────────────────────────────────────────
//  MAIN
// ──────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const char* method = (argc > 1) ? argv[1] : "yinfast";

    std::cout << "trumpet_synth v3\n"
              << "Pitch method   : " << method << "\n"
              << "Buffer/hop     : " << BUFFER_SIZE << "/" << HOP_SIZE << "\n"
              << "Sample rate    : " << SR << " Hz\n"
              << "Glide time     : " << GLIDE_TIME_S * 1000.0f << " ms\n"
              << "Breath mix     : " << BREATH_MIX << "\n\n";

    pitch_obj = new_aubio_pitch(method, BUFFER_SIZE, HOP_SIZE, SR);
    if (!pitch_obj) { std::cerr << "Failed to create aubio pitch object\n"; return 1; }
    aubio_pitch_set_unit(pitch_obj, "Hz");
    aubio_pitch_set_silence(pitch_obj, -40.0f);
    in_buf    = new_fvec(HOP_SIZE);
    pitch_out = new_fvec(1);

    PaError err = Pa_Initialize();
    if (err != paNoError) { std::cerr << Pa_GetErrorText(err) << "\n"; return 1; }

    int nd = Pa_GetDeviceCount();
    std::cout << "── Audio devices ──\n";
    for (int i = 0; i < nd; ++i) {
        const PaDeviceInfo* d = Pa_GetDeviceInfo(i);
        std::cout << "  [" << i << "] " << d->name
                  << "  in=" << d->maxInputChannels
                  << "  out=" << d->maxOutputChannels << "\n";
    }
    std::cout << "\nUsing input=" << INPUT_DEVICE << "  output=" << OUTPUT_DEVICE << "\n\n";

    PaStreamParameters inParams{};
    inParams.device           = INPUT_DEVICE;
    inParams.channelCount     = 1;
    inParams.sampleFormat     = paFloat32;
    inParams.suggestedLatency = Pa_GetDeviceInfo(INPUT_DEVICE)->defaultLowInputLatency;

    PaStreamParameters outParams{};
    outParams.device          = OUTPUT_DEVICE;
    outParams.channelCount    = 2;
    outParams.sampleFormat    = paFloat32;
    outParams.suggestedLatency = Pa_GetDeviceInfo(OUTPUT_DEVICE)->defaultLowOutputLatency;

    PaStream* inStream = nullptr, *outStream = nullptr;

    err = Pa_OpenStream(&inStream,  &inParams,  nullptr,    SR, HOP_SIZE, paClipOff, input_callback,  nullptr);
    if (err != paNoError) { std::cerr << "Input:  " << Pa_GetErrorText(err) << "\n"; Pa_Terminate(); return 1; }
    err = Pa_OpenStream(&outStream, nullptr,    &outParams, SR, HOP_SIZE, paClipOff, output_callback, nullptr);
    if (err != paNoError) { std::cerr << "Output: " << Pa_GetErrorText(err) << "\n"; Pa_CloseStream(inStream); Pa_Terminate(); return 1; }

    const PaStreamInfo* inInfo  = Pa_GetStreamInfo(inStream);
    const PaStreamInfo* outInfo = Pa_GetStreamInfo(outStream);
    std::cout << "── Latency ──\n"
              << "  Input  : " << inInfo->inputLatency   * 1000.0 << " ms\n"
              << "  Output : " << outInfo->outputLatency * 1000.0 << " ms\n"
              << "  Total  : " << (inInfo->inputLatency + outInfo->outputLatency) * 1000.0 << " ms\n\n"
              << "Play into the mic. Telemetry every " << TELEM_INTERVAL << " callbacks.\n"
              << "Press Enter to stop.\n\n";
    std::cout.flush();

    Pa_StartStream(inStream);
    Pa_StartStream(outStream);

    while (true) {
        if (g_telem_ready.load(std::memory_order_acquire)) {
            TelemSnapshot s = g_telem_snap;
            g_telem_ready.store(false, std::memory_order_release);
            std::cout << "\n─────────────────────────────\n"
                      << "  Avg level  : " << s.avg_db   << " dBFS\n"
                      << "  Avg conf   : " << s.avg_conf << "\n"
                      << "  Avg pitch  : " << s.avg_hz   << " Hz\n"
                      << "  Gate open  : " << s.gate_pct << "%\n"
                      << "─────────────────────────────\n";
            std::cout.flush();
        }
#ifndef _WIN32
        fd_set fds; FD_ZERO(&fds); FD_SET(0, &fds);
        struct timeval tv = {0, 10000};
        if (select(1, &fds, nullptr, nullptr, &tv) > 0) break;
#else
        if (_kbhit()) { int c = _getch(); if (c == '\r' || c == '\n') break; }
        Sleep(10);
#endif
    }

    Pa_StopStream(inStream);  Pa_StopStream(outStream);
    Pa_CloseStream(inStream); Pa_CloseStream(outStream);
    Pa_Terminate();
    del_aubio_pitch(pitch_obj); del_fvec(in_buf); del_fvec(pitch_out);
    return 0;
}