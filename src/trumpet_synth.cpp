import aubio
import numpy as np
import sounddevice as sd
import time
import threading

# ---------------- SETTINGS ----------------
SR = 22050
BUFFER_SIZE = 256      # LOWER = less latency (was 512)
HOP_SIZE = 64          # LOWER = less latency (was 128)

INPUT_DEVICE  = 1  # USB PnP mic
OUTPUT_DEVICE = 4   # your working headphone device

sd.default.latency = 'low'
sd.default.extra_settings = None

# ---------------- AUBIO PITCH DETECTOR ----------------
pitch_o = aubio.pitch("yinfast", BUFFER_SIZE, HOP_SIZE, SR)
pitch_o.set_unit("Hz")
pitch_o.set_silence(-40)

# ---------------- SHARED STATE ----------------
last_pitch = 0.0
conf_threshold = 0.5
play_conf_threshold = 0.8
play_level_threshold = -20.0

# HOLD SYSTEM (replaces smoothing)
last_good_time = 0.0
HOLD_TIME = 0.05  # 50 ms memory window

_freq  = 0.0
_phase = 0.0

# ---------------- SET PITCH ----------------
def set_pitch(hz):
    global _freq
    _freq = hz   # removed lock (lower latency, safe for this use)

def trumpet_wave(phases):
    return (
        1.0 * np.sin(phases) +
        0.6 * np.sin(2 * phases) +
        0.35 * np.sin(3 * phases) +
        0.2 * np.sin(4 * phases) +
        0.12 * np.sin(5 * phases) +
        0.08 * np.sin(6 * phases)
    )

# ---------------- OUTPUT CALLBACK ----------------
def output_callback(outdata, frames, time_info, status):
    global _phase

    freq = _freq

    if freq > 0:
        phase_inc = 2.0 * np.pi * freq / SR

        phases = _phase + phase_inc * np.arange(frames, dtype=np.float32)

        # trumpet harmonic synthesis
        wave = trumpet_wave(phases)

        # brass bite
        wave = np.tanh(wave * 2.2)

        # gain
        wave *= 0.25

        _phase = float((phases[-1] + phase_inc) % (2.0 * np.pi))

        outdata[:, 0] = wave.astype(np.float32)
        outdata[:, 1] = outdata[:, 0]

    else:
        _phase = 0.0
        outdata.fill(0)

# ---------------- INPUT CALLBACK ----------------
def input_callback(indata, frames, time_info, status):
    global last_pitch, last_good_time

    audio = indata[:, 0].astype(np.float32)

    t0 = time.perf_counter()

    pitch = pitch_o(audio)[0]
    confidence = pitch_o.get_confidence()

    t1 = time.perf_counter()

    # faster / lighter RMS calc (avoids sqrt)
    rms = np.mean(audio * audio)
    db  = 10.0 * np.log10(rms + 1e-12)

    latency_ms = (t1 - t0) * 1000
    now = t0

    # ---------------- PITCH UPDATE ----------------
    if confidence > conf_threshold and pitch > 0:
        last_pitch = pitch
        last_good_time = now

    valid_recently = (now - last_good_time) < HOLD_TIME

    # ---------------- GATING ----------------
    if confidence > play_conf_threshold and db > play_level_threshold and valid_recently:
        set_pitch(last_pitch)
        #print(f"Pitch: {last_pitch:7.2f} Hz | Conf: {confidence:.2f} | "
              #f"Level: {db:5.1f} dBFS | Algo: {latency_ms:.2f} ms")
    else:
        set_pitch(0)
        #print(f"not playing anything | Level: {db:5.1f} dBFS")

# ---------------- LAUNCH ----------------
print("Starting low-latency trumpet system...")

with sd.InputStream(
        device=INPUT_DEVICE,
        samplerate=SR,
        blocksize=HOP_SIZE,
        channels=1,
        callback=input_callback,
        latency='low'
), sd.OutputStream(
        device=OUTPUT_DEVICE,
        samplerate=SR,
        blocksize=HOP_SIZE,
        channels=2,
        callback=output_callback,
        latency='low'
):
    input("Running... press Enter to stop\n")