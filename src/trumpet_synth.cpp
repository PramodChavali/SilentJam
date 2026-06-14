/**
 * silent_jam.cpp — YIN pitch-to-headphones with smooth, glitch-free audio
 *
 * C++ port of silent_jam.py
 *
 * Dependencies:
 *   - PortAudio  (pa)
 *   - aubio      (aubio)
 *   - libsndfile (sndfile)
 *
 * Build (Linux / Raspberry Pi):
 *   g++ -O2 -std=c++17 silent_jam.cpp \
 *       -lportaudio -laubio -lsndfile \
 *       -o silent_jam
 *
 * Build (macOS with Homebrew):
 *   g++ -O2 -std=c++17 silent_jam.cpp \
 *       $(pkg-config --cflags --libs portaudio-2.0 aubio sndfile) \
 *       -o silent_jam
 */

#include <aubio/aubio.h>
#include <portaudio.h>
#include <sndfile.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

// ─────────────────────────── SETTINGS ─────────────────────────────────────

static constexpr int    SR          = 22050;
static constexpr int    HOP_SIZE    = 128;   // aubio hop size
static constexpr int    BUFFER_SIZE = 512;   // aubio window
static constexpr int    OUT_BLOCK   = 512;   // output blocksize

static constexpr int    INPUT_DEVICE  = 0;
static constexpr int    OUTPUT_DEVICE = 2;   // 4=default,5=Beats,20=AirPods,7=wired

// ─────────────────────── TUNABLE THRESHOLDS ───────────────────────────────

static constexpr double CONF_DETECT = 0.60;
static constexpr double CONF_PLAY   = 0.90;
static constexpr double LEVEL_PLAY  = -20.0; // dBFS
static constexpr double HOLD_TIME   = 0.30;  // seconds

// ─────────────────────── SYNTHESIS PARAMETERS ─────────────────────────────

static constexpr double FREQ_SMOOTH_A = 0.05;  // per-frame EMA alpha
static constexpr double ENV_ATTACK    = 0.008; // 8 ms
static constexpr double ENV_RELEASE   = 0.025; // 25 ms

// ─────────────────────── THREAD-SAFE STATE ────────────────────────────────

static std::atomic<double> g_target_freq{0.0};

static inline double get_target() { return g_target_freq.load(std::memory_order_relaxed); }
static inline void   set_target(double hz) { g_target_freq.store(hz, std::memory_order_relaxed); }

// ─────────────────────── RECORDING STATE ──────────────────────────────────

static std::atomic<bool>   g_is_recording{false};
static std::mutex          g_chunks_mutex;
static std::vector<float>  g_recorded_chunks;

// ─────────────────────── OUTPUT STATE ─────────────────────────────────────

struct OutputState {
    double phase        = 0.0;
    double current_freq = 0.0;
    double amplitude    = 0.0;
    double attack_step;
    double release_step;

    OutputState() {
        attack_step  = static_cast<double>(OUT_BLOCK) / (ENV_ATTACK  * SR);
        release_step = static_cast<double>(OUT_BLOCK) / (ENV_RELEASE * SR);
    }
};

// ─────────────────────── INPUT STATE ──────────────────────────────────────

struct InputState {
    aubio_pitch_t* pitch_obj = nullptr;
    fvec_t*        in_buf    = nullptr;
    fvec_t*        out_buf   = nullptr;

    double last_pitch     = 0.0;
    double last_good_time = 0.0; // seconds since epoch (perf counter)
};

// ─────────────────────── WAVEFORM ─────────────────────────────────────────

/**
 * Additive synthesis approximating a trumpet timbre.
 * Mirrors the Python trumpet_wave() function sample-by-sample.
 */
static inline double trumpet_wave(double phase) {
    return 1.00 * std::sin(phase)
         + 0.60 * std::sin(2.0 * phase)
         + 0.35 * std::sin(3.0 * phase)
         + 0.20 * std::sin(4.0 * phase)
         + 0.12 * std::sin(5.0 * phase)
         + 0.08 * std::sin(6.0 * phase);
}

// ─────────────────────── TIME HELPER ──────────────────────────────────────

static double perf_counter() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ─────────────────────── RECORDING SAVE ───────────────────────────────────

static void save_recording() {
    std::vector<float> audio;
    {
        std::lock_guard<std::mutex> lk(g_chunks_mutex);
        audio = g_recorded_chunks;  // copy
    }

    if (audio.empty()) {
        std::puts("Nothing to save.");
        return;
    }

    // Normalise to 95 % of full scale
    float peak = 0.0f;
    for (float s : audio) peak = std::max(peak, std::abs(s));
    if (peak > 0.0f) {
        float scale = 0.95f / peak;
        for (float& s : audio) s *= scale;
    }

    // Build filename
    std::filesystem::create_directories("recordings");
    std::time_t now_t = std::time(nullptr);
    char ts[20];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now_t));
    std::string filename = std::string("recordings/silentjam_") + ts + ".wav";

    // Write WAV via libsndfile (float32 — avoids manual int16 conversion)
    SF_INFO sfinfo{};
    sfinfo.samplerate = SR;
    sfinfo.channels   = 1;
    sfinfo.format     = SF_FORMAT_WAV | SF_FORMAT_FLOAT;

    SNDFILE* sf = sf_open(filename.c_str(), SFM_WRITE, &sfinfo);
    if (!sf) {
        std::fprintf(stderr, "Could not open %s for writing: %s\n",
                     filename.c_str(), sf_strerror(nullptr));
        return;
    }
    sf_writef_float(sf, audio.data(), static_cast<sf_count_t>(audio.size()));
    sf_close(sf);

    std::printf("Saved: %s\n", filename.c_str());
}

// ─────────────────────── OUTPUT CALLBACK ──────────────────────────────────

static int output_callback(
    const void* /*input*/,
    void*       output,
    unsigned long frames,
    const PaStreamCallbackTimeInfo* /*timeInfo*/,
    PaStreamCallbackFlags /*statusFlags*/,
    void* userData)
{
    auto*  state = static_cast<OutputState*>(userData);
    auto*  out   = static_cast<float*>(output);

    double target     = get_target();
    bool   want_sound = (target > 0.0);

    // 1. Smooth frequency toward target
    if (want_sound) {
        if (state->current_freq == 0.0)
            state->current_freq = target;  // snap on first note
        else
            state->current_freq += FREQ_SMOOTH_A * (target - state->current_freq);
    }

    // 2. Amplitude envelope
    if (want_sound)
        state->amplitude = std::min(1.0, state->amplitude + state->attack_step);
    else
        state->amplitude = std::max(0.0, state->amplitude - state->release_step);

    // 3. Synthesise
    if (state->amplitude > 0.0 && state->current_freq > 0.0) {
        const double phase_inc    = 2.0 * M_PI * state->current_freq / SR;
        const double two_pi       = 2.0 * M_PI;
        const double amp          = state->amplitude;
        float        record_buf[OUT_BLOCK];

        for (unsigned long i = 0; i < frames; ++i) {
            double ph   = state->phase + phase_inc * static_cast<double>(i);
            double wave = trumpet_wave(ph);
            // Soft clip + level
            wave = std::tanh(wave * 2.2) * 0.25 * amp;
            float s = static_cast<float>(wave);
            out[i * 2 + 0] = s;  // left
            out[i * 2 + 1] = s;  // right
            record_buf[i]   = s;
        }

        // Advance and wrap phase
        state->phase = std::fmod(
            state->phase + phase_inc * static_cast<double>(frames),
            two_pi);

        if (g_is_recording.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> lk(g_chunks_mutex);
            g_recorded_chunks.insert(
                g_recorded_chunks.end(), record_buf, record_buf + frames);
        }

    } else {
        if (!want_sound)
            state->current_freq = 0.0;
        state->phase = 0.0;
        std::memset(out, 0, frames * 2 * sizeof(float));

        if (g_is_recording.load(std::memory_order_relaxed)) {
            std::lock_guard<std::mutex> lk(g_chunks_mutex);
            g_recorded_chunks.insert(
                g_recorded_chunks.end(), frames, 0.0f);
        }
    }

    return paContinue;
}

// ─────────────────────── INPUT CALLBACK ───────────────────────────────────

static int input_callback(
    const void* input,
    void*       /*output*/,
    unsigned long frames,
    const PaStreamCallbackTimeInfo* /*timeInfo*/,
    PaStreamCallbackFlags /*statusFlags*/,
    void* userData)
{
    auto*        state    = static_cast<InputState*>(userData);
    const float* in       = static_cast<const float*>(input);

    // Fill aubio input buffer
    unsigned long copy_n = std::min(static_cast<unsigned long>(HOP_SIZE), frames);
    for (unsigned long i = 0; i < copy_n; ++i)
        state->in_buf->data[i] = in[i];

    double now = perf_counter();

    // YIN detection
    aubio_pitch_do(state->pitch_obj, state->in_buf, state->out_buf);
    double pitch      = static_cast<double>(state->out_buf->data[0]);
    double confidence = static_cast<double>(aubio_pitch_get_confidence(state->pitch_obj));

    // Level in dBFS
    double rms_val = 0.0;
    for (unsigned long i = 0; i < copy_n; ++i)
        rms_val += static_cast<double>(in[i]) * static_cast<double>(in[i]);
    rms_val /= static_cast<double>(copy_n);
    double db = 10.0 * std::log10(rms_val + 1e-12);

    // Update last known good pitch
    if (confidence > CONF_DETECT && pitch > 0.0) {
        state->last_pitch     = pitch;
        state->last_good_time = now;
    }

    bool held_recently = (now - state->last_good_time) < HOLD_TIME;

    if (confidence > CONF_PLAY
            && db > LEVEL_PLAY
            && held_recently
            && state->last_pitch > 0.0) {
        set_target(state->last_pitch);
        std::printf("\rPitch: %7.2f Hz | Conf: %.2f | %5.1f dBFS  ",
                    state->last_pitch, confidence, db);
        std::fflush(stdout);
    } else {
        set_target(0.0);
    }

    return paContinue;
}

// ─────────────────────── MAIN ─────────────────────────────────────────────

int main() {
    // ── Initialise PortAudio ──────────────────────────────────────────────
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::fprintf(stderr, "PortAudio init error: %s\n", Pa_GetErrorText(err));
        return 1;
    }

    // ── Set up aubio pitch detector ───────────────────────────────────────
    InputState  in_state;
    in_state.in_buf   = new_fvec(HOP_SIZE);
    in_state.out_buf  = new_fvec(1);
    in_state.pitch_obj = new_aubio_pitch(
        "yinfast",
        static_cast<uint_t>(BUFFER_SIZE),
        static_cast<uint_t>(HOP_SIZE),
        static_cast<uint_t>(SR));
    aubio_pitch_set_unit(in_state.pitch_obj, "Hz");
    aubio_pitch_set_silence(in_state.pitch_obj, -40.0f);

    // ── Output state ──────────────────────────────────────────────────────
    OutputState out_state;

    // ── Print info ────────────────────────────────────────────────────────
    std::printf("Starting Silent Jam...\n");
    std::printf("  Input device  : %d\n", INPUT_DEVICE);
    std::printf("  Output device : %d\n", OUTPUT_DEVICE);
    std::printf("  Sample rate   : %d Hz\n", SR);
    std::printf("  YIN hop size  : %d samples (%.1f ms)\n",
                HOP_SIZE, 1000.0 * HOP_SIZE / SR);
    std::printf("  Output block  : %d samples (%.1f ms)\n\n",
                OUT_BLOCK, 1000.0 * OUT_BLOCK / SR);

    // ── Open input stream ─────────────────────────────────────────────────
    PaStreamParameters in_params{};
    in_params.device                    = INPUT_DEVICE;
    in_params.channelCount              = 1;
    in_params.sampleFormat              = paFloat32;
    in_params.suggestedLatency          =
        Pa_GetDeviceInfo(INPUT_DEVICE)->defaultLowInputLatency;

    PaStream* in_stream = nullptr;
    err = Pa_OpenStream(
        &in_stream,
        &in_params, nullptr,
        SR, HOP_SIZE,
        paClipOff,
        input_callback, &in_state);
    if (err != paNoError) {
        std::fprintf(stderr, "Failed to open input stream: %s\n", Pa_GetErrorText(err));
        Pa_Terminate();
        return 1;
    }

    // ── Open output stream ────────────────────────────────────────────────
    PaStreamParameters out_params{};
    out_params.device                    = OUTPUT_DEVICE;
    out_params.channelCount              = 2;
    out_params.sampleFormat              = paFloat32;
    out_params.suggestedLatency          =
        Pa_GetDeviceInfo(OUTPUT_DEVICE)->defaultLowOutputLatency;

    PaStream* out_stream = nullptr;
    err = Pa_OpenStream(
        &out_stream,
        nullptr, &out_params,
        SR, OUT_BLOCK,
        paClipOff,
        output_callback, &out_state);
    if (err != paNoError) {
        std::fprintf(stderr, "Failed to open output stream: %s\n", Pa_GetErrorText(err));
        Pa_CloseStream(in_stream);
        Pa_Terminate();
        return 1;
    }

    Pa_StartStream(in_stream);
    Pa_StartStream(out_stream);

    // ── Command loop ──────────────────────────────────────────────────────
    std::puts("Controls: 1 = start/stop recording | 0 = quit");
    std::string cmd;
    while (true) {
        std::printf("> ");
        std::fflush(stdout);
        if (!std::getline(std::cin, cmd)) break;  // EOF

        if (cmd == "1") {
            if (!g_is_recording.load()) {
                {
                    std::lock_guard<std::mutex> lk(g_chunks_mutex);
                    g_recorded_chunks.clear();
                }
                g_is_recording.store(true);
                std::puts("Recording started...");
            } else {
                g_is_recording.store(false);
                std::puts("\nRecording stopped. Saving...");
                save_recording();
            }
        } else if (cmd == "0") {
            if (g_is_recording.load()) {
                g_is_recording.store(false);
                save_recording();
            }
            std::puts("Goodbye!");
            break;
        } else {
            std::puts("Unknown command.  1 = record | 0 = quit");
        }
    }

    // ── Cleanup ───────────────────────────────────────────────────────────
    Pa_StopStream(in_stream);
    Pa_StopStream(out_stream);
    Pa_CloseStream(in_stream);
    Pa_CloseStream(out_stream);
    Pa_Terminate();

    del_aubio_pitch(in_state.pitch_obj);
    del_fvec(in_state.in_buf);
    del_fvec(in_state.out_buf);
    aubio_cleanup();

    return 0;
}