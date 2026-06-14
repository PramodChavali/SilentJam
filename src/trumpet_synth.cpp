/*
 * silent_jam.cpp  –  v5: trumpet_synth v4 + recording
 *
 * Combines:
 *   - The v4 low-note detection + synthesis engine (rolling-window YIN,
 *     pitch-adaptive harmonics, breath noise, glide, RMS-driven amplitude).
 *     All thresholds and tuning values are kept from trumpet_synth v4.
 *   - The recording feature ported from silent_jam.py / the recording build:
 *     captures the SYNTHESIZED output (what you hear), normalises it, and
 *     writes a timestamped mono WAV into ./recordings/ via libsndfile.
 *
 * Monitoring is always live: you hear yourself play whether or not you are
 * recording. Recording is a toggle — start, then stop to save.
 *
 * Triggering recording:
 *   - Console fallback (kept for testing): '1' toggles record, '0' quits.
 *   - Physical button on the Pi: call toggle_recording() from your GPIO
 *     interrupt handler. See the BUTTON HOOK comments in main() and below.
 *
 * Build (Linux / Raspberry Pi):
 *   g++ -O2 -std=c++17 silent_jam.cpp \
 *       -lportaudio -laubio -lsndfile -lm -o silent_jam
 *
 *   With CMake/pkg-config:
 *     pkg_check_modules(... aubio portaudio-2.0 sndfile)
 *
 * Run:
 *   ./silent_jam [pitch_method]   (default: yinfast)
 */

#include <aubio/aubio.h>
#include <portaudio.h>
#include <sndfile.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// GPIO button support (Raspberry Pi). Compile with -DUSE_GPIO_BUTTON and link
// against libgpiod (-lgpiod). On non-Pi builds, leave the flag off and the
// button code is excluded so the file still compiles anywhere.
#ifdef USE_GPIO_BUTTON
#  include <gpiod.h>
#endif

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

// ──────────────────────────────────────────────
//  SETTINGS — tune these on the Pi  (kept from v4)
// ──────────────────────────────────────────────
static constexpr int    SR            = 44100;
static constexpr int    BUFFER_SIZE   = 1024;   // analysis window — 1024 keeps the
                                                // low-note fix (musical floor ~85 Hz,
                                                // an octave below any trumpet note)
                                                // while halving onset lag vs 2048
static constexpr int    HOP_SIZE      = 128;    // keep small for latency
static constexpr int    INPUT_DEVICE  = 1;
static constexpr int    OUTPUT_DEVICE = 0;

// GPIO button (Raspberry Pi). Button wired across 3.3V and GPIO17; internal
// pull-down enabled in software, so press = HIGH, rising edge = toggle record.
static constexpr int    BUTTON_GPIO_CHIP_LINE = 17;     // BCM GPIO17 (phys pin 11)
static constexpr int    BUTTON_DEBOUNCE_MS    = 40;     // ignore bounces within this
static constexpr const char* BUTTON_GPIO_CHIP = "gpiochip0";  // Pi 4B main bank

// Gate (kept from v4)
static constexpr float  CONF_THRESHOLD      = 0.5f;
static constexpr float  PLAY_CONF_THRESHOLD = 0.8f;
static constexpr float  PLAY_LEVEL_DB       = -10.0f;
static constexpr float  HOLD_TIME_S         = 0.05f;

// Synthesis (kept from v4)
static constexpr float  OUTPUT_GAIN         = 0.30f;
static constexpr float  BREATH_MIX          = 0.06f;
static constexpr float  GLIDE_TIME_S        = 0.012f;
static constexpr float  ENV_ATTACK_S        = 0.004f;
static constexpr float  ENV_RELEASE_S       = 0.060f;

// Dynamics — output loudness follows how hard you play, based on input dB.
// The input level is smoothed slowly (DYN_SMOOTH_S) so the volume tracks your
// breath, not the mic's per-sample jitter, keeping the tone consistent.
//   DYN_DB_SOFT  : input dB mapped to the quietest output (DYN_GAIN_MIN)
//   DYN_DB_LOUD  : input dB mapped to full output (1.0)
//   DYN_GAIN_MIN : floor gain so soft notes stay clean & present, never vanish
// Set DYN_GAIN_MIN = 1.0f to disable dynamics (constant volume).
static constexpr float  DYN_SMOOTH_S        = 0.150f;  // 150 ms loudness tracking
static constexpr float  DYN_DB_SOFT         = -30.0f;  // soft playing level (dBFS)
static constexpr float  DYN_DB_LOUD         = -8.0f;   // loud playing level (dBFS)
static constexpr float  DYN_GAIN_MIN        = 0.40f;   // floor: softest = 40% vol

// Telemetry
static constexpr int    TELEM_INTERVAL      = 100;

// ──────────────────────────────────────────────
//  SHARED STATE  (input callback → output callback)
// ──────────────────────────────────────────────
static std::atomic<float>  g_freq         {0.0f};
static std::atomic<float>  g_rms_lin      {0.0f};
static std::atomic<float>  g_dyn_gain     {1.0f};   // smoothed loudness 0..1 (dynamics)
static std::atomic<double> g_last_good_ts {0.0};
static std::atomic<double> g_last_pitch   {0.0};

// ──────────────────────────────────────────────
//  RECORDING STATE
// ──────────────────────────────────────────────
// g_is_recording is read in the realtime output callback (lock-free atomic).
// g_recorded_chunks is appended under a mutex; the callback only locks briefly
// to push the block it just synthesised. Saving happens off the audio thread.
static std::atomic<bool>   g_is_recording{false};
static std::mutex          g_chunks_mutex;
static std::vector<float>  g_recorded_chunks;   // mono, one sample per frame

// Set by the GPIO button thread on a clean press; the main loop polls it and
// calls toggle_recording() on the normal thread (so file I/O never runs inside
// the button thread's event context).
static std::atomic<bool>   g_button_toggle_requested{false};
// Tells the button thread to exit cleanly on shutdown.
static std::atomic<bool>   g_running{true};

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

// Rolling input history fed to aubio each hop.
static float g_ring[BUFFER_SIZE] = {};
static int   g_ring_pos = 0;

// ──────────────────────────────────────────────
//  DSP HELPERS  (kept from v4)
// ──────────────────────────────────────────────
static inline float smooth_coeff(float tau_s) {
    return 1.0f - expf(-1.0f / (tau_s * (float)SR));
}

static constexpr int N_HARM = 8;
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

static uint32_t lcg_state = 0x12345678u;
static inline float white_noise() {
    lcg_state = lcg_state * 1664525u + 1013904223u;
    return ((int32_t)lcg_state) * (1.0f / 2147483648.0f);
}

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
//  RECORDING CONTROL
// ──────────────────────────────────────────────
// Forward declaration — defined below save_recording().
static void save_recording();

// toggle_recording():  single entry point for BOTH the console command and
// the physical button. Call this from your GPIO interrupt handler.
//
// NOTE ON CALLING FROM A GPIO ISR:
//   Saving a WAV (file I/O + normalisation) must NOT run inside an interrupt
//   context. This function only flips an atomic and, on stop, calls
//   save_recording() directly — which is fine when called from the console
//   thread. If you wire it to a hardware interrupt, do NOT call save_recording()
//   inside the ISR. Instead, set a flag (e.g. g_save_requested) in the ISR and
//   let the main loop notice it and call save_recording(). See the BUTTON HOOK
//   block in main() for the recommended pattern.
static void toggle_recording() {
    if (!g_is_recording.load()) {
        {
            std::lock_guard<std::mutex> lk(g_chunks_mutex);
            g_recorded_chunks.clear();
        }
        g_is_recording.store(true);
        std::puts("\n[REC] Recording started...");
    } else {
        g_is_recording.store(false);
        std::puts("\n[REC] Recording stopped. Saving...");
        save_recording();
    }
}

static void save_recording() {
    std::vector<float> audio;
    {
        std::lock_guard<std::mutex> lk(g_chunks_mutex);
        audio = g_recorded_chunks;  // copy out, release lock
    }

    if (audio.empty()) {
        std::puts("[REC] Nothing to save.");
        return;
    }

    // Normalise to 95% of full scale
    float peak = 0.0f;
    for (float s : audio) peak = std::max(peak, std::fabs(s));
    if (peak > 0.0f) {
        float scale = 0.95f / peak;
        for (float& s : audio) s *= scale;
    }

    std::filesystem::create_directories("recordings");
    std::time_t now_t = std::time(nullptr);
    char ts[20];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now_t));
    std::string filename = std::string("recordings/silentjam_") + ts + ".wav";

    SF_INFO sfinfo{};
    sfinfo.samplerate = SR;
    sfinfo.channels   = 1;                                  // mono
    sfinfo.format     = SF_FORMAT_WAV | SF_FORMAT_FLOAT;

    SNDFILE* sf = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
    if (!sf) {
        std::fprintf(stderr, "[REC] Could not open %s: %s\n",
                     filename.c_str(), sf_strerror(nullptr));
        return;
    }
    sf_writef_float(sf, audio.data(), static_cast<sf_count_t>(audio.size()));
    sf_close(sf);

    std::printf("[REC] Saved: %s  (%.1f s)\n",
                filename.c_str(), (double)audio.size() / SR);
}

// ──────────────────────────────────────────────
//  INPUT CALLBACK  (kept from v4)
// ──────────────────────────────────────────────
static int input_callback(
        const void* inputBuffer, void*,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*)
{
    const float* in = static_cast<const float*>(inputBuffer);

    unsigned long n = std::min(framesPerBuffer, (unsigned long)HOP_SIZE);
    for (unsigned long i = 0; i < n; ++i) {
        g_ring[g_ring_pos] = in[i];
        g_ring_pos = (g_ring_pos + 1) % BUFFER_SIZE;
    }
    for (int i = 0; i < BUFFER_SIZE; ++i) {
        in_buf->data[i] = g_ring[(g_ring_pos + i) % BUFFER_SIZE];
    }

    aubio_pitch_do(pitch_obj, in_buf, pitch_out);
    float pitch = fvec_get_sample(pitch_out, 0);
    float conf  = aubio_pitch_get_confidence(pitch_obj);

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

    // Dynamics: slowly smooth the input dB, map it to an output gain in
    // [DYN_GAIN_MIN, 1.0]. Slow smoothing is what keeps the resulting volume
    // steady instead of chasing the mic's jitter. Hop-rate one-pole, so the
    // coefficient is per-hop (HOP_SIZE samples), not per-sample.
    static float dyn_db = DYN_DB_SOFT;
    static const float dyn_alpha =
        1.0f - expf(-1.0f / (DYN_SMOOTH_S * (float)SR / (float)HOP_SIZE));
    dyn_db += dyn_alpha * (db - dyn_db);
    float dyn_t    = (dyn_db - DYN_DB_SOFT) / (DYN_DB_LOUD - DYN_DB_SOFT);
    dyn_t          = std::max(0.0f, std::min(1.0f, dyn_t));
    float dyn_gain = DYN_GAIN_MIN + (1.0f - DYN_GAIN_MIN) * dyn_t;
    g_dyn_gain.store(dyn_gain, std::memory_order_relaxed);

    static double cb_time = 0.0;
    cb_time += (double)HOP_SIZE / SR;

    if (conf > CONF_THRESHOLD && pitch > 0 && pitch < 2000.0f) {
        g_last_pitch.store((double)pitch, std::memory_order_relaxed);
        g_last_good_ts.store(cb_time,     std::memory_order_relaxed);
    }
    bool pitch_valid = (cb_time - g_last_good_ts.load(std::memory_order_relaxed)) < HOLD_TIME_S;
    bool gate_open   = (conf > PLAY_CONF_THRESHOLD) && (db > PLAY_LEVEL_DB) && pitch_valid;

    g_freq.store(gate_open
        ? (float)g_last_pitch.load(std::memory_order_relaxed)
        : 0.0f,
        std::memory_order_relaxed);

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
//  OUTPUT CALLBACK  — synthesis (v4) + recording tap
// ──────────────────────────────────────────────
static int output_callback(
        const void*, void* outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*)
{
    float* out = static_cast<float*>(outputBuffer);

    float target_freq = g_freq.load(std::memory_order_relaxed);
    float dyn_gain    = g_dyn_gain.load(std::memory_order_relaxed);  // 0..1 loudness
    // (input RMS is still tracked on the input side for telemetry; output level
    //  is the fixed envelope below, scaled by the smoothed dynamics gain.)

    static float phases[N_HARM]   = {};
    static float cur_freq         = 0.0f;
    static float env_dyn          = 1.0f;   // per-sample-smoothed dynamics gain
    static float env_amp          = 0.0f;
    static float cached_freq      = 0.0f;
    static float harm[N_HARM]     = {};
    static BandpassState bp;

    static const float glide_alpha = smooth_coeff(GLIDE_TIME_S);
    static const float amp_alpha_a = smooth_coeff(ENV_ATTACK_S);
    static const float amp_alpha_r = smooth_coeff(ENV_RELEASE_S);

    // Capture the mono signal for recording as we generate it. Both output
    // channels are identical, so we store one sample per frame.
    const bool recording = g_is_recording.load(std::memory_order_relaxed);
    static std::vector<float> rec_scratch;   // reused; avoids per-callback alloc
    if (recording) {
        rec_scratch.clear();
        rec_scratch.reserve(framesPerBuffer);
    }

    bool playing = (target_freq > 0.0f);

    for (unsigned long i = 0; i < framesPerBuffer; ++i) {

        if (playing) {
            if (cur_freq < 10.0f) cur_freq = target_freq;
            cur_freq += glide_alpha * (target_freq - cur_freq);
        }

        // On/off envelope: a smooth attack/release gate so notes start and stop
        // click-free. The sustained LEVEL is constant (1.0) — loudness comes from
        // the dynamics gain below, not from chasing the noisy input amplitude.
        float amp_target = playing ? 1.0f : 0.0f;
        float amp_alpha  = (amp_target > env_amp) ? amp_alpha_a : amp_alpha_r;
        env_amp += amp_alpha * (amp_target - env_amp);

        // Dynamics: glide the loudness gain toward the value the input side set.
        // Already slow-smoothed there; this extra per-sample glide just removes
        // any block-rate stepping so swells are perfectly smooth.
        env_dyn += amp_alpha_r * (dyn_gain - env_dyn);

        if (fabsf(cur_freq - cached_freq) > cached_freq * 0.03f + 1.0f) {
            harm_amps(cur_freq, harm);
            cached_freq = cur_freq;
        }

        float sample = 0.0f;
        if (cur_freq > 10.0f) {
            float phase_inc_fund = 2.0f * (float)M_PI * cur_freq / (float)SR;
            for (int k = 0; k < N_HARM; ++k) {
                if ((k + 1) * cur_freq >= (float)SR * 0.45f) break;
                phases[k] += (float)(k + 1) * phase_inc_fund;
                if (phases[k] > 2.0f * (float)M_PI) phases[k] -= 2.0f * (float)M_PI;
                sample += harm[k] * sinf(phases[k]);
            }
        }

        sample += bp.process(white_noise()) * BREATH_MIX * env_amp;
        sample *= env_amp * env_dyn * OUTPUT_GAIN;
        sample  = tanhf(sample * 0.8f) / 0.8f;

        out[2 * i]     = sample;
        out[2 * i + 1] = sample;

        if (recording) rec_scratch.push_back(sample);
    }

    // Push this block into the recording buffer under the mutex. Kept outside
    // the per-sample loop so we lock once per callback, not once per sample.
    if (recording && !rec_scratch.empty()) {
        std::lock_guard<std::mutex> lk(g_chunks_mutex);
        g_recorded_chunks.insert(g_recorded_chunks.end(),
                                 rec_scratch.begin(), rec_scratch.end());
    }

    return paContinue;
}

// ──────────────────────────────────────────────
//  GPIO BUTTON THREAD  (Raspberry Pi, libgpiod v1.x)
// ──────────────────────────────────────────────
// Blocks waiting for rising edges on GPIO17 (press = HIGH thanks to the
// internal pull-down). Debounces in software, then sets a flag the main loop
// acts on. Uses zero CPU while waiting. Excluded unless -DUSE_GPIO_BUTTON.
#ifdef USE_GPIO_BUTTON
static void button_thread_fn() {
    gpiod_chip* chip = gpiod_chip_open_by_name(BUTTON_GPIO_CHIP);
    if (!chip) {
        std::fprintf(stderr, "[BTN] Could not open %s — button disabled. "
                             "Console controls still work.\n", BUTTON_GPIO_CHIP);
        return;
    }
    gpiod_line* line = gpiod_chip_get_line(chip, BUTTON_GPIO_CHIP_LINE);
    if (!line) {
        std::fprintf(stderr, "[BTN] Could not get GPIO%d — button disabled.\n",
                     BUTTON_GPIO_CHIP_LINE);
        gpiod_chip_close(chip);
        return;
    }

    // Request rising-edge events with the internal pull-down enabled.
    gpiod_line_request_config cfg{};
    cfg.consumer    = "silent_jam";
    cfg.request_type = GPIOD_LINE_REQUEST_EVENT_RISING_EDGE;
    cfg.flags       = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    if (gpiod_line_request(line, &cfg, 0) < 0) {
        std::fprintf(stderr, "[BTN] Failed to request GPIO%d events "
                             "(need permission? try running with sudo, or add "
                             "your user to the 'gpio' group). Button disabled.\n",
                     BUTTON_GPIO_CHIP_LINE);
        gpiod_chip_close(chip);
        return;
    }

    std::printf("[BTN] Button ready on GPIO%d — press to start/stop recording.\n",
                BUTTON_GPIO_CHIP_LINE);

    double last_press_ts = 0.0;
    while (g_running.load()) {
        // Wait up to 200 ms for an edge, then loop so we can check g_running.
        timespec timeout{0, 200 * 1000 * 1000};
        int rv = gpiod_line_event_wait(line, &timeout);
        if (rv < 0) break;        // error
        if (rv == 0) continue;    // timeout, just re-check g_running

        gpiod_line_event ev;
        if (gpiod_line_event_read(line, &ev) < 0) continue;

        // Software debounce: ignore edges within BUTTON_DEBOUNCE_MS of the last.
        double now = (double)ev.ts.tv_sec + (double)ev.ts.tv_nsec * 1e-9;
        if ((now - last_press_ts) * 1000.0 < BUTTON_DEBOUNCE_MS) continue;
        last_press_ts = now;

        // Hand off to the main thread — do NOT save a file here.
        g_button_toggle_requested.store(true);
    }

    gpiod_line_release(line);
    gpiod_chip_close(chip);
}
#endif // USE_GPIO_BUTTON

// ──────────────────────────────────────────────
//  MAIN
// ──────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const char* method = (argc > 1) ? argv[1] : "yinfast";

    std::cout << "silent_jam v5  (v4 synth + recording)\n"
              << "Pitch method   : " << method << "\n"
              << "Analysis window: " << BUFFER_SIZE << " samples\n"
              << "Hop size       : " << HOP_SIZE << " samples\n"
              << "Sample rate    : " << SR << " Hz\n"
              << "Glide time     : " << GLIDE_TIME_S * 1000.0f << " ms\n"
              << "Breath mix     : " << BREATH_MIX << "\n\n";

    pitch_obj = new_aubio_pitch(method, BUFFER_SIZE, HOP_SIZE, SR);
    if (!pitch_obj) { std::cerr << "Failed to create aubio pitch object\n"; return 1; }
    aubio_pitch_set_unit(pitch_obj, "Hz");
    aubio_pitch_set_silence(pitch_obj, -40.0f);
    in_buf    = new_fvec(BUFFER_SIZE);
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

    // ──────────────────────────────────────────
    //  GPIO BUTTON — launch watcher thread (Pi)
    // ──────────────────────────────────────────
    // The thread blocks on GPIO17 edge events and sets
    // g_button_toggle_requested on each clean press. The command loop polls
    // that flag and calls toggle_recording() on this thread, so the WAV write
    // never happens in the button thread's context.
    // Built only with -DUSE_GPIO_BUTTON; otherwise console controls are used.
#ifdef USE_GPIO_BUTTON
    std::thread button_thread(button_thread_fn);
#endif

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
              << "  Total  : " << (inInfo->inputLatency + outInfo->outputLatency) * 1000.0 << " ms\n\n";

    Pa_StartStream(inStream);
    Pa_StartStream(outStream);

    std::puts("Live monitoring is ON — you can hear yourself whether or not you're recording.");
    std::puts("Controls (console fallback): 1 = start/stop recording | 0 = quit");
    std::puts("(Physical button calls the same toggle_recording().)\n");

    // ── Command loop ──
    // Uses select() so we can poll both stdin AND the button flag without
    // blocking. Telemetry prints here too.
    std::string line_buf;
    while (true) {
        if (g_telem_ready.load(std::memory_order_acquire)) {
            TelemSnapshot s = g_telem_snap;
            g_telem_ready.store(false, std::memory_order_release);
            std::cout << "\n─────────────────────────────\n"
                      << "  Avg level  : " << s.avg_db   << " dBFS\n"
                      << "  Avg conf   : " << s.avg_conf << "\n"
                      << "  Avg pitch  : " << s.avg_hz   << " Hz\n"
                      << "  Gate open  : " << s.gate_pct << "%"
                      << (g_is_recording.load() ? "   [REC ●]" : "")
                      << "\n─────────────────────────────\n";
            std::cout.flush();
        }

        // Physical button: the GPIO thread sets this flag on a clean press.
        // We act on it here, on the normal thread, so the WAV save is safe.
        if (g_button_toggle_requested.exchange(false)) {
            toggle_recording();
        }

#ifndef _WIN32
        fd_set fds; FD_ZERO(&fds); FD_SET(0, &fds);
        struct timeval tv = {0, 10000};   // 10 ms poll
        if (select(1, &fds, nullptr, nullptr, &tv) > 0) {
            if (!std::getline(std::cin, line_buf)) break;  // EOF
            if (line_buf == "1") {
                toggle_recording();
            } else if (line_buf == "0") {
                if (g_is_recording.load()) toggle_recording();  // stop+save first
                std::puts("Goodbye!");
                break;
            } else if (!line_buf.empty()) {
                std::puts("Unknown command. 1 = record | 0 = quit");
            }
        }
#else
        if (_kbhit()) {
            int c = _getch();
            if (c == '1') toggle_recording();
            else if (c == '0') {
                if (g_is_recording.load()) toggle_recording();
                break;
            }
        }
        Sleep(10);
#endif
    }

    // Tell the button thread to exit, then join it before tearing down audio.
    g_running.store(false);
#ifdef USE_GPIO_BUTTON
    if (button_thread.joinable()) button_thread.join();
#endif

    Pa_StopStream(inStream);  Pa_StopStream(outStream);
    Pa_CloseStream(inStream); Pa_CloseStream(outStream);
    Pa_Terminate();
    del_aubio_pitch(pitch_obj); del_fvec(in_buf); del_fvec(pitch_out);
    aubio_cleanup();
    return 0;
}