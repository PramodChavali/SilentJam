/*
 * silent_jam.cpp  –  v5 + pigpio button
 *
 * Recording branch (v5) merged with pigpio button from trumpet_synth.
 * Button on GPIO16: press = start recording, press again = stop + save WAV.
 *
 * Build:
 *   g++ -O2 -std=c++17 silent_jam.cpp \
 *       -lportaudio -laubio -lsndfile -lm -lpigpio -lrt -lpthread \
 *       -DUSE_GPIO_BUTTON -o silent_jam
 *
 *   Without button:
 *   g++ -O2 -std=c++17 silent_jam.cpp \
 *       -lportaudio -laubio -lsndfile -lm -o silent_jam
 *
 * Run:  sudo ./silent_jam [pitch_method]   (default: yinfast)
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
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#ifdef USE_GPIO_BUTTON
#  include <pigpio.h>
#endif

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

// ──────────────────────────────────────────────
//  SETTINGS
// ──────────────────────────────────────────────
static constexpr int    SR            = 44100;
static constexpr int    BUFFER_SIZE   = 512;   // 512@44100=11.6ms window, enough for trumpet lowest note ~120Hz
static constexpr int    HOP_SIZE      = 128;
static constexpr int    INPUT_DEVICE  = 1;   // USB PnP mic
static constexpr int    OUTPUT_DEVICE = 0;   // bcm2835 headphones

static constexpr int    BUTTON_PIN         = 16;   // BCM GPIO16 (physical pin 36)
static constexpr int    BUTTON_DEBOUNCE_MS = 200;

// Gate
static constexpr float  CONF_THRESHOLD      = 0.5f;
static constexpr float  PLAY_CONF_THRESHOLD = 0.8f;
static constexpr float  PLAY_LEVEL_DB       = -10.0f;
static constexpr float  HOLD_TIME_S         = 0.05f;

// Synthesis
static constexpr float  OUTPUT_GAIN         = 0.30f;
static constexpr float  BREATH_MIX          = 0.06f;
static constexpr float  GLIDE_TIME_S        = 0.012f;
static constexpr float  ENV_ATTACK_S        = 0.004f;
static constexpr float  ENV_RELEASE_S       = 0.120f;

// Dynamics
static constexpr float  DYN_SMOOTH_S        = 0.150f;
static constexpr float  DYN_DB_SOFT         = -30.0f;
static constexpr float  DYN_DB_LOUD         = -8.0f;
static constexpr float  DYN_GAIN_MIN        = 0.40f;

// Telemetry
static constexpr int    TELEM_INTERVAL      = 100;

// ──────────────────────────────────────────────
//  SHARED STATE
// ──────────────────────────────────────────────
static std::atomic<float>  g_freq         {0.0f};
static std::atomic<float>  g_rms_lin      {0.0f};
static std::atomic<float>  g_dyn_gain     {1.0f};
static std::atomic<double> g_last_good_ts {0.0};
static std::atomic<double> g_last_pitch   {0.0};

// ──────────────────────────────────────────────
//  RECORDING STATE
// ──────────────────────────────────────────────
static std::atomic<bool>   g_is_recording{false};
static std::vector<float>  g_recorded_chunks;
static std::atomic<size_t> g_rec_write_pos{0};

// Button sets this flag; main loop acts on it (so file I/O never runs in button thread)
static std::atomic<bool>   g_button_toggle_requested{false};
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

static float g_ring[BUFFER_SIZE] = {};
static int   g_ring_pos = 0;

// ──────────────────────────────────────────────
//  DSP HELPERS
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
static void save_recording() {
    size_t len = g_rec_write_pos.load(std::memory_order_relaxed);
    if (len == 0) { std::puts("[REC] Nothing to save."); return; }

    std::vector<float> audio(g_recorded_chunks.begin(),
                             g_recorded_chunks.begin() + len);

    std::filesystem::create_directories("recordings");
    std::time_t now_t = std::time(nullptr);
    char ts[20];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now_t));
    std::string filename = std::string("recordings/silentjam_") + ts + ".wav";

    SF_INFO sfinfo{};
    sfinfo.samplerate = SR;
    sfinfo.channels   = 2;
    sfinfo.format     = SF_FORMAT_WAV | SF_FORMAT_FLOAT;

    SNDFILE* sf = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
    if (!sf) {
        std::fprintf(stderr, "[REC] Could not open %s: %s\n",
                     filename.c_str(), sf_strerror(nullptr));
        return;
    }
    sf_writef_float(sf, audio.data(), (sf_count_t)(len / 2));
    sf_close(sf);

    std::printf("[REC] Saved: %s  (%.1f s)\n",
                filename.c_str(), (double)(len / 2) / SR);
    g_rec_write_pos.store(0, std::memory_order_relaxed);
}

static void toggle_recording() {
    if (!g_is_recording.load()) {
        g_rec_write_pos.store(0, std::memory_order_relaxed);
        g_is_recording.store(true);
        std::puts("\n[REC] Recording started...");
    } else {
        g_is_recording.store(false);
        std::puts("\n[REC] Recording stopped. Saving...");
        save_recording();
    }
}

// ──────────────────────────────────────────────
//  INPUT CALLBACK
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
    for (int i = 0; i < BUFFER_SIZE; ++i)
        in_buf->data[i] = g_ring[(g_ring_pos + i) % BUFFER_SIZE];

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

    static float dyn_db = DYN_DB_SOFT;
    static const float dyn_alpha =
        1.0f - expf(-1.0f / (DYN_SMOOTH_S * (float)SR / (float)HOP_SIZE));
    dyn_db += dyn_alpha * (db - dyn_db);
    float dyn_t    = (dyn_db - DYN_DB_SOFT) / (DYN_DB_LOUD - DYN_DB_SOFT);
    dyn_t          = std::max(0.0f, std::min(1.0f, dyn_t));
    g_dyn_gain.store(DYN_GAIN_MIN + (1.0f - DYN_GAIN_MIN) * dyn_t,
                     std::memory_order_relaxed);

    static double cb_time = 0.0;
    cb_time += (double)HOP_SIZE / SR;

    if (conf > CONF_THRESHOLD && pitch > 0 && pitch < 2000.0f) {
        // Stabiliser: reject one-hop glitches and octave jumps.
        static float stable    = 0.0f;
        static float candidate = 0.0f;
        static int   agree     = 0;
        float ratio = (stable > 0.0f) ? pitch / stable : 1.0f;
        bool is_octave = (ratio > 1.8f && ratio < 2.2f) || (ratio > 0.45f && ratio < 0.55f);
        bool is_jump   = (ratio > 1.07f || ratio < 0.93f);
        int  needed    = is_octave ? 3 : (is_jump ? 2 : 1);
        if (fabsf(pitch - candidate) / (candidate + 1.0f) < 0.07f)
            agree++;
        else { candidate = pitch; agree = 1; }
        if (agree >= needed) {
            stable = pitch;
            g_last_pitch.store((double)stable, std::memory_order_relaxed);
            g_last_good_ts.store(cb_time, std::memory_order_relaxed);
        }
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
//  OUTPUT CALLBACK
// ──────────────────────────────────────────────
static int output_callback(
        const void*, void* outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*)
{
    float* out = static_cast<float*>(outputBuffer);

    float target_freq = g_freq.load(std::memory_order_relaxed);
    float dyn_gain    = g_dyn_gain.load(std::memory_order_relaxed);

    static float phases[N_HARM]   = {};
    static float cur_freq         = 0.0f;
    static float env_dyn          = 1.0f;
    static float env_amp          = 0.0f;
    static float cached_freq      = 0.0f;
    static float harm[N_HARM]     = {};
    static BandpassState bp;

    static const float glide_alpha = smooth_coeff(GLIDE_TIME_S);
    static const float amp_alpha_a = smooth_coeff(ENV_ATTACK_S);
    static const float amp_alpha_r = smooth_coeff(ENV_RELEASE_S);

    const bool recording = g_is_recording.load(std::memory_order_relaxed);
    bool playing = (target_freq > 0.0f);

    for (unsigned long i = 0; i < framesPerBuffer; ++i) {
        if (playing) {
            if (cur_freq < 10.0f) cur_freq = target_freq;
            cur_freq += glide_alpha * (target_freq - cur_freq);
        }

        float amp_target = playing ? 1.0f : 0.0f;
        float amp_alpha  = (amp_target > env_amp) ? amp_alpha_a : amp_alpha_r;
        env_amp += amp_alpha * (amp_target - env_amp);
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

        if (recording) {
            size_t pos = g_rec_write_pos.load(std::memory_order_relaxed);
            if (pos + 2 <= g_recorded_chunks.size()) {
                g_recorded_chunks[pos]     = sample;
                g_recorded_chunks[pos + 1] = sample;
                g_rec_write_pos.store(pos + 2, std::memory_order_relaxed);
            }
        }
    }

    return paContinue;
}

// ──────────────────────────────────────────────
//  GPIO BUTTON THREAD  (pigpio)
// ──────────────────────────────────────────────
// Polls GPIO16 with internal pull-up (idle HIGH, press LOW = falling edge).
// On each clean press, sets g_button_toggle_requested. The main loop acts on
// it so WAV saves never happen inside this thread.
#ifdef USE_GPIO_BUTTON
static void button_thread_fn() {
    gpioSetMode(BUTTON_PIN, PI_INPUT);
    gpioSetPullUpDown(BUTTON_PIN, PI_PUD_UP);  // idle HIGH, press LOW

    bool previous = gpioRead(BUTTON_PIN);
    double last_press = 0.0;

    std::printf("[BTN] Button ready on GPIO%d\n", BUTTON_PIN);
    std::fflush(stdout);

    while (g_running.load()) {
        bool current = gpioRead(BUTTON_PIN);

        if (previous && !current) {   // falling edge = press
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            double now = ts.tv_sec + ts.tv_nsec * 1e-9;

            if ((now - last_press) * 1000.0 >= BUTTON_DEBOUNCE_MS) {
                last_press = now;
                g_button_toggle_requested.store(true);
                std::printf("[BTN] Press detected\n");
                std::fflush(stdout);
            }
        }

        previous = current;
        usleep(5000);  // 5 ms poll
    }

    std::printf("[BTN] GPIO thread exiting\n");
}
#endif

// ──────────────────────────────────────────────
//  MAIN
// ──────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const char* method = (argc > 1) ? argv[1] : "schmitt";

    std::cout << "silent_jam  (v5 synth + recording + pigpio button)\n"
              << "Pitch method: " << method << "  |  SR: " << SR << " Hz\n\n";

    // ── pigpio init ──
#ifdef USE_GPIO_BUTTON
    if (gpioInitialise() < 0) {
        std::fprintf(stderr, "[BTN] pigpio init failed — run with sudo. Button disabled.\n");
    } else {
        std::printf("[BTN] pigpio OK\n");
        std::thread(button_thread_fn).detach();
    }
#endif

    // ── aubio ──
    pitch_obj = new_aubio_pitch(method, BUFFER_SIZE, HOP_SIZE, SR);
    if (!pitch_obj) { std::cerr << "Failed to create aubio pitch object\n"; return 1; }
    aubio_pitch_set_unit(pitch_obj, "Hz");
    aubio_pitch_set_silence(pitch_obj, -40.0f);
    in_buf    = new_fvec(BUFFER_SIZE);
    pitch_out = new_fvec(1);

    // ── PortAudio ──
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
    inParams.suggestedLatency = 0;  // request absolute minimum

    PaStreamParameters outParams{};
    outParams.device           = OUTPUT_DEVICE;
    outParams.channelCount     = 2;
    outParams.sampleFormat     = paFloat32;
    outParams.suggestedLatency = 0;  // request absolute minimum

    PaStream* inStream = nullptr, *outStream = nullptr;

    err = Pa_OpenStream(&inStream,  &inParams,  nullptr,    SR, HOP_SIZE, paClipOff, input_callback,  nullptr);
    if (err != paNoError) { std::cerr << "Input:  " << Pa_GetErrorText(err) << "\n"; Pa_Terminate(); return 1; }
    err = Pa_OpenStream(&outStream, nullptr,    &outParams, SR, HOP_SIZE, paClipOff, output_callback, nullptr);
    if (err != paNoError) { std::cerr << "Output: " << Pa_GetErrorText(err) << "\n"; Pa_CloseStream(inStream); Pa_Terminate(); return 1; }

    g_recorded_chunks.resize(SR * 2 * 300);  // 5 min stereo pre-alloc
    Pa_StartStream(inStream);
    Pa_StartStream(outStream);

    std::puts("Live monitoring ON. Press button or type '1' to start/stop recording. '0' to quit.\n");

    // ── Command loop ──
    std::string line_buf;
    while (true) {
        if (g_telem_ready.load(std::memory_order_acquire)) {
            TelemSnapshot s = g_telem_snap;
            g_telem_ready.store(false, std::memory_order_release);
            std::printf("\n  Level: %.1f dBFS  Conf: %.2f  Pitch: %.1f Hz  Gate: %d%%%s\n",
                        s.avg_db, s.avg_conf, s.avg_hz, s.gate_pct,
                        g_is_recording.load() ? "  [REC ●]" : "");
            std::fflush(stdout);
        }

        // Button press → toggle recording (safe: file I/O on main thread)
        if (g_button_toggle_requested.exchange(false)) {
            toggle_recording();
        }

        fd_set fds; FD_ZERO(&fds); FD_SET(0, &fds);
        struct timeval tv = {0, 10000};
        if (select(1, &fds, nullptr, nullptr, &tv) > 0) {
            if (!std::getline(std::cin, line_buf)) break;
            if (line_buf == "1") {
                toggle_recording();
            } else if (line_buf == "0") {
                if (g_is_recording.load()) toggle_recording();
                std::puts("Goodbye!");
                break;
            }
        }
    }

    g_running.store(false);

    Pa_StopStream(inStream);  Pa_StopStream(outStream);
    Pa_CloseStream(inStream); Pa_CloseStream(outStream);
    Pa_Terminate();
    del_aubio_pitch(pitch_obj); del_fvec(in_buf); del_fvec(pitch_out);
    aubio_cleanup();

#ifdef USE_GPIO_BUTTON
    gpioTerminate();
#endif

    return 0;
}