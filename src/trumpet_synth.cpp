/*
 * trumpet_synth.cpp  –  with latency telemetry
 * Telemetry is passed from the audio thread to main via an atomic flag,
 * then printed safely from the main thread.
 */

#include <aubio/aubio.h>
#include <portaudio.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <algorithm>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#  include <conio.h>
#  ifndef M_PI
#    define M_PI 3.14159265358979323846
#  endif
#endif

// ──────────────────────────────────────────────
//  SETTINGS
// ──────────────────────────────────────────────
static constexpr int    SR            = 44100;
static constexpr int    BUFFER_SIZE   = 256;
static constexpr int    HOP_SIZE      = 128;
static constexpr int    INPUT_DEVICE  = 1;
static constexpr int    OUTPUT_DEVICE = 0;

static constexpr double CONF_THRESHOLD      = 0.5;
static constexpr double PLAY_CONF_THRESHOLD = 0.8;
static constexpr double PLAY_LEVEL_DB       = -10.0;
static constexpr double HOLD_TIME           = 0.05;
static constexpr int    TELEM_INTERVAL      = 100;

// ──────────────────────────────────────────────
//  SHARED AUDIO STATE
// ──────────────────────────────────────────────
static std::atomic<float>  g_freq         {0.0f};
static std::atomic<double> g_last_pitch   {0.0};
static std::atomic<double> g_last_good_ts {0.0};
static float g_phase = 0.0f;

// ──────────────────────────────────────────────
//  TELEMETRY  (written by audio thread, read by main thread)
// ──────────────────────────────────────────────
struct TelemSnapshot {
    double avg_copy_us;
    double avg_pitch_ms;
    double avg_rms_us;
    double avg_gate_us;
    double avg_total_ms;
    double avg_jitter_ms;
    double pitch_min_ms;
    double pitch_max_ms;
    double jitter_max_ms;
    double avg_conf;
    double avg_db;
    int    gate_pct;
};



static std::atomic<bool>  g_telem_ready {false};
static TelemSnapshot      g_telem_snap  {};   // written by audio thread, read by main

// Audio-thread-only accumulators (no atomics needed)
static int    telem_count         = 0;
static double telem_copy_total    = 0.0;
static double telem_pitch_total   = 0.0;
static double telem_rms_total     = 0.0;
static double telem_gate_total    = 0.0;
static double telem_cb_total      = 0.0;
static double telem_pitch_min     = 1e9;
static double telem_pitch_max     = 0.0;
static double telem_last_cb_time  = 0.0;
static double telem_jitter_total  = 0.0;
static double telem_jitter_max    = 0.0;
static int    telem_gate_open     = 0;
static float  telem_conf_sum      = 0.0f;
static float  telem_db_sum        = 0.0f;

// ──────────────────────────────────────────────
//  AUBIO
// ──────────────────────────────────────────────
static aubio_pitch_t* pitch_obj = nullptr;
static fvec_t*        in_buf    = nullptr;
static fvec_t*        pitch_out = nullptr;

// ──────────────────────────────────────────────
//  HELPERS
// ──────────────────────────────────────────────
static inline double now_sec()
{
#ifdef _WIN32
    LARGE_INTEGER freq, counter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
#endif
}

static inline float trumpet_sample(float phase)
{
    return 1.00f * sinf(phase)
         + 0.60f * sinf(2.0f * phase)
         + 0.35f * sinf(3.0f * phase)
         + 0.20f * sinf(4.0f * phase)
         + 0.12f * sinf(5.0f * phase)
         + 0.08f * sinf(6.0f * phase);
}

// ──────────────────────────────────────────────
//  OUTPUT CALLBACK
// ──────────────────────────────────────────────
static int output_callback(
        const void*, void* outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*)
{
    float* out  = static_cast<float*>(outputBuffer);
    float  freq = g_freq.load(std::memory_order_relaxed);

    if (freq > 0.0f) {
        const float phase_inc = 2.0f * (float)M_PI * freq / (float)SR;
        for (unsigned long i = 0; i < framesPerBuffer; ++i) {
            float sample = tanhf(trumpet_sample(g_phase) * 2.2f) * 0.25f;
            out[2 * i]     = sample;
            out[2 * i + 1] = sample;
            g_phase += phase_inc;
        }
        g_phase = fmodf(g_phase, 2.0f * (float)M_PI);
    } else {
        g_phase = 0.0f;
        memset(out, 0, framesPerBuffer * 2 * sizeof(float));
    }
    return paContinue;
}

// ──────────────────────────────────────────────
//  INPUT CALLBACK
// ──────────────────────────────────────────────
static int input_callback(
        const void* inputBuffer, void*,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo*, PaStreamCallbackFlags, void*)
{
    double cb_enter = now_sec();

    // Jitter
    if (telem_last_cb_time > 0.0) {
        double j = cb_enter - telem_last_cb_time;
        telem_jitter_total += j;
        if (j > telem_jitter_max) telem_jitter_max = j;
    }
    telem_last_cb_time = cb_enter;

    const float* in = static_cast<const float*>(inputBuffer);

    // 1. Buffer copy
    double t0 = now_sec();
    unsigned long copy_frames = framesPerBuffer < (unsigned long)HOP_SIZE
                                    ? framesPerBuffer : (unsigned long)HOP_SIZE;
    for (unsigned long i = 0; i < copy_frames; ++i)
        in_buf->data[i] = in[i];
    for (unsigned long i = copy_frames; i < (unsigned long)HOP_SIZE; ++i)
        in_buf->data[i] = 0.0f;
    telem_copy_total += now_sec() - t0;

    // 2. Pitch detection
    double t2 = now_sec();
    aubio_pitch_do(pitch_obj, in_buf, pitch_out);
    double pitch_time = now_sec() - t2;
    telem_pitch_total += pitch_time;
    if (pitch_time < telem_pitch_min) telem_pitch_min = pitch_time;
    if (pitch_time > telem_pitch_max) telem_pitch_max = pitch_time;

    float pitch      = fvec_get_sample(pitch_out, 0);
    float confidence = aubio_pitch_get_confidence(pitch_obj);
    telem_conf_sum  += confidence;

    // 3. RMS
    double t4 = now_sec();
    float rms_sq = 0.0f;
    for (unsigned long i = 0; i < copy_frames; ++i)
        rms_sq += in[i] * in[i];
    rms_sq /= (float)copy_frames;
    float db = 10.0f * log10f(rms_sq + 1e-12f);
    telem_rms_total += now_sec() - t4;
    telem_db_sum    += db;

    // 4. Gate + store
    double t6 = now_sec();
    double now = cb_enter;
    if (confidence > (float)CONF_THRESHOLD && pitch > 0.0f) {
        g_last_pitch.store((double)pitch, std::memory_order_relaxed);
        g_last_good_ts.store(now,         std::memory_order_relaxed);
    }
    bool valid = (now - g_last_good_ts.load(std::memory_order_relaxed)) < HOLD_TIME;
    if (confidence > (float)PLAY_CONF_THRESHOLD && db > (float)PLAY_LEVEL_DB && valid) {
        g_freq.store((float)g_last_pitch.load(std::memory_order_relaxed), std::memory_order_relaxed);
        telem_gate_open++;
    } else {
        g_freq.store(0.0f, std::memory_order_relaxed);
    }
    telem_gate_total += now_sec() - t6;
    telem_cb_total   += now_sec() - cb_enter;

    // Every N callbacks: package telemetry and signal main thread
    if (++telem_count >= TELEM_INTERVAL && !g_telem_ready.load(std::memory_order_relaxed)) {
        double n = (double)TELEM_INTERVAL;
        g_telem_snap.avg_copy_us  = telem_copy_total  / n * 1e6;
        g_telem_snap.avg_pitch_ms = telem_pitch_total / n * 1e3;
        g_telem_snap.avg_rms_us   = telem_rms_total   / n * 1e6;
        g_telem_snap.avg_gate_us  = telem_gate_total  / n * 1e6;
        g_telem_snap.avg_total_ms = telem_cb_total    / n * 1e3;
        g_telem_snap.avg_jitter_ms= telem_jitter_total/ n * 1e3;
        g_telem_snap.pitch_min_ms = telem_pitch_min   * 1e3;
        g_telem_snap.pitch_max_ms = telem_pitch_max   * 1e3;
        g_telem_snap.jitter_max_ms= telem_jitter_max  * 1e3;
        g_telem_snap.avg_conf     = telem_conf_sum    / (float)TELEM_INTERVAL;
        g_telem_snap.avg_db       = telem_db_sum      / (float)TELEM_INTERVAL;
        g_telem_snap.gate_pct     = (int)(100.0 * telem_gate_open / TELEM_INTERVAL);
        g_telem_ready.store(true, std::memory_order_release);

        // Reset
        telem_count = 0;
        telem_copy_total = telem_pitch_total = telem_rms_total = 0.0;
        telem_gate_total = telem_cb_total = telem_jitter_total = 0.0;
        telem_pitch_min  = 1e9; telem_pitch_max = telem_jitter_max = 0.0;
        telem_gate_open  = 0;
        telem_conf_sum   = 0.0f; telem_db_sum = 0.0f;
    }

    return paContinue;
}

// ──────────────────────────────────────────────
//  MAIN
// ──────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const char* method = (argc > 1) ? argv[1] : "yinfast";
    double ideal_ms = 1000.0 * HOP_SIZE / SR;

    std::cout << "Pitch method   : " << method << "\n"
              << "Buffer/hop     : " << BUFFER_SIZE << "/" << HOP_SIZE << " samples\n"
              << "Sample rate    : " << SR << " Hz\n"
              << "Ideal cb period: " << ideal_ms << " ms\n\n";

    pitch_obj = new_aubio_pitch(method, BUFFER_SIZE, HOP_SIZE, SR);
    if (!pitch_obj) { std::cerr << "Failed to create aubio pitch object\n"; return 1; }
    aubio_pitch_set_unit(pitch_obj, "Hz");
    aubio_pitch_set_silence(pitch_obj, -40.0f);
    in_buf    = new_fvec(HOP_SIZE);
    pitch_out = new_fvec(1);

    PaError err = Pa_Initialize();
    if (err != paNoError) { std::cerr << Pa_GetErrorText(err) << "\n"; return 1; }

    int numDevices = Pa_GetDeviceCount();
    std::cout << "── Audio devices ──\n";
    for (int i = 0; i < numDevices; ++i) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        std::cout << "  [" << i << "] " << info->name
                  << "  in=" << info->maxInputChannels
                  << " out=" << info->maxOutputChannels << "\n";
    }
    std::cout << "\nUsing input=" << INPUT_DEVICE << "  output=" << OUTPUT_DEVICE << "\n\n";

    PaStreamParameters inParams{};
    inParams.device            = INPUT_DEVICE;
    inParams.channelCount      = 1;
    inParams.sampleFormat      = paFloat32;
    inParams.suggestedLatency  = Pa_GetDeviceInfo(INPUT_DEVICE)->defaultLowInputLatency;

    PaStreamParameters outParams{};
    outParams.device           = OUTPUT_DEVICE;
    outParams.channelCount     = 2;
    outParams.sampleFormat     = paFloat32;
    outParams.suggestedLatency = Pa_GetDeviceInfo(OUTPUT_DEVICE)->defaultLowOutputLatency;

    PaStream* inStream  = nullptr;
    PaStream* outStream = nullptr;

    err = Pa_OpenStream(&inStream,  &inParams,  nullptr,    SR, HOP_SIZE, paClipOff, input_callback,  nullptr);
    if (err != paNoError) { std::cerr << "Input stream: "  << Pa_GetErrorText(err) << "\n"; Pa_Terminate(); return 1; }

    err = Pa_OpenStream(&outStream, nullptr,    &outParams, SR, HOP_SIZE, paClipOff, output_callback, nullptr);
    if (err != paNoError) { std::cerr << "Output stream: " << Pa_GetErrorText(err) << "\n"; Pa_CloseStream(inStream); Pa_Terminate(); return 1; }

    const PaStreamInfo* inInfo  = Pa_GetStreamInfo(inStream);
    const PaStreamInfo* outInfo = Pa_GetStreamInfo(outStream);
    std::cout << "── PortAudio negotiated latency ──\n"
              << "  Input  : " << inInfo->inputLatency   * 1000.0 << " ms\n"
              << "  Output : " << outInfo->outputLatency * 1000.0 << " ms\n"
              << "  Total  : " << (inInfo->inputLatency + outInfo->outputLatency) * 1000.0 << " ms\n\n"
              << "Play into the mic. Telemetry prints every " << TELEM_INTERVAL << " callbacks.\n"
              << "Press Enter to stop.\n\n";
    std::cout.flush();

    Pa_StartStream(inStream);
    Pa_StartStream(outStream);

    // ── Main loop: poll for telemetry and print it safely ──
    while (true) {
        // Check for Enter key (non-blocking on Windows)
#ifdef _WIN32
        if (_kbhit()) { 
            int c = _getch();
            if (c == '\r' || c == '\n') break;
        }
#else
        // On non-Windows just block on cin after checking telem
#endif
        if (g_telem_ready.load(std::memory_order_acquire)) {
            TelemSnapshot s = g_telem_snap;
            g_telem_ready.store(false, std::memory_order_release);

            std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                      << "  LATENCY TELEMETRY  (last " << TELEM_INTERVAL << " callbacks)\n"
                      << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
                      << "\n  [INPUT CALLBACK BREAKDOWN]\n"
                      << "  1. Buffer copy     : " << s.avg_copy_us  << " us\n"
                      << "  2. Pitch detection : " << s.avg_pitch_ms << " ms"
                      << "   min=" << s.pitch_min_ms << " ms"
                      << "   max=" << s.pitch_max_ms << " ms\n"
                      << "  3. RMS calc        : " << s.avg_rms_us   << " us\n"
                      << "  4. Gate + store    : " << s.avg_gate_us  << " us\n"
                      << "  ── TOTAL callback  : " << s.avg_total_ms << " ms"
                      << "   (ideal=" << ideal_ms << " ms)\n"
                      << "\n  [SCHEDULING JITTER]\n"
                      << "  Avg jitter         : " << s.avg_jitter_ms << " ms\n"
                      << "  Max jitter         : " << s.jitter_max_ms << " ms\n"
                      << "\n  [SIGNAL]\n"
                      << "  Avg confidence     : " << s.avg_conf  << "\n"
                      << "  Avg level          : " << s.avg_db    << " dBFS\n"
                      << "  Gate open          : " << s.gate_pct  << "%\n"
                      << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
            std::cout.flush();
        }

#ifdef _WIN32
        Sleep(10);  // poll every 10ms
#endif
    }

    Pa_StopStream(inStream);
    Pa_StopStream(outStream);
    Pa_CloseStream(inStream);
    Pa_CloseStream(outStream);
    Pa_Terminate();

    del_aubio_pitch(pitch_obj);
    del_fvec(in_buf);
    del_fvec(pitch_out);

    return 0;
}