import json
import math
import time
from collections import deque

import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


PORT = "COM7"                  # Pico COM port (used only when TEST_DATA = False)
TEST_DATA = False             # True = test data without a Pico, False = real Pico
TEST_ERROR = False             # True = test data sends an error message after 5 s
SENSORS = ["adc", "ads1115"]   # One column per sensor, left to right; must match the "sensor" field
HISTORY_SECONDS = 2            # Waveform length shown (s)
SPECTRUM_MAX_HZ = 1000         # Right edge of the spectrum; capped at half the sample rate
SPECTROGRAM_SECONDS = 10       # Spectrogram length shown (s)
SPECTROGRAM_ROWS = 100         # Frequency rows in the spectrogram; peaks are kept when rows are merged


# Test signal for each sensor: sample rate, block size, FFT length, DC bias,
# amplitude and base tone (Hz). The tone doubles every other 3 s period.
TEST_SENSORS = {
    "adc": {"rate": 8000.0, "block": 256, "fft": 2048, "bias": 2048, "amp": 300,
            "tone": 125, "units": "ADC counts peak"},
    "ads1115": {"rate": 867.0, "block": 128, "fft": 1024, "bias": 0, "amp": 2000,
                "tone": 40, "units": "ADS1115 counts peak"},
}
test_blocks = {name: 0 for name in TEST_SENSORS}
test_start = time.monotonic()



# Returns the bytes both sensors would have sent since the last call,
# in the same JSON format as the Pico.
def read_test_data():
    out = b""
    elapsed = time.monotonic() - test_start
    if TEST_ERROR and elapsed > 5:
        return b'{"type":"error","code":"acquisition_overflow","message":"Measurement stopped. Reset the Pico to restart."}\n'
    for name, t in TEST_SENSORS.items():
        blocks_per_second = t["rate"] / t["block"]
        while test_blocks[name] < int(elapsed * blocks_per_second):
            block = test_blocks[name]
            if block % round(blocks_per_second) == 0:
                out += (json.dumps({"type": "config", "sensor": name, "sample_rate_hz": t["rate"],
                                    "fft_length": t["fft"], "spectrum_units": t["units"]}) + "\n").encode()
            seconds = block / blocks_per_second
            tone = t["tone"] if int(seconds // 3) % 2 == 0 else 2 * t["tone"]
            first = block * t["block"]
            values = [round(t["bias"] + t["amp"] * math.sin(2 * math.pi * tone * n / t["rate"]))
                      for n in range(first, first + t["block"])]
            out += (json.dumps({"type": "samples", "sensor": name, "first_sample_index": first,
                                "values": values}) + "\n").encode()
            blocks_per_fft = t["fft"] // t["block"]
            if block % blocks_per_fft == blocks_per_fft - 1:
                bins = np.random.uniform(0.2, 2.0, t["fft"] // 2 + 1).round(4)
                bins[round(tone * t["fft"] / t["rate"])] = float(t["amp"])
                out += (json.dumps({"type": "fft", "sensor": name, "values": bins.tolist()}) + "\n").encode()
            test_blocks[name] += 1
    return out



port = None if TEST_DATA else serial.Serial(PORT, timeout=0)   # timeout=0: reading never waits
buffer = b""                   # Bytes of an unfinished line wait here


# Window: 3 rows (waveform, spectrum, spectrogram) x one column per sensor.
fig, axes = plt.subplots(3, len(SENSORS), figsize=(14, 9), squeeze=False)
sensors = {}
for col, name in enumerate(SENSORS):
    ax_wave, ax_spec, ax_sgram = axes[:, col]
    wave_line, = ax_wave.plot([], [], lw=0.8)
    spec_line, = ax_spec.plot([], [], lw=0.8)
    sgram_image = ax_sgram.imshow(np.zeros((2, 2)), origin="lower", aspect="auto",
                                  cmap="viridis", interpolation="nearest",
                                  extent=(-SPECTROGRAM_SECONDS, 0, 0, SPECTRUM_MAX_HZ))
    ax_wave.set(title=name, xlim=(-HISTORY_SECONDS, 0), xlabel="Time (s)", ylabel="Counts")
    ax_spec.set(xlim=(0, SPECTRUM_MAX_HZ), xlabel="Frequency (Hz)", ylabel="Counts peak")
    ax_sgram.set(xlim=(-SPECTROGRAM_SECONDS, 0), xlabel="Time (s)", ylabel="Frequency (Hz)")
    fig.colorbar(sgram_image, ax=ax_sgram, label="dB")
    # Everything about one sensor: its settings, data and plot objects.
    sensors[name] = {"rate": None, "fft_len": None, "idx": None, "val": None,
                     "spectrum": None, "history": None,
                     "ax_wave": ax_wave, "ax_spec": ax_spec, "ax_sgram": ax_sgram,
                     "wave_line": wave_line, "spec_line": spec_line, "sgram_image": sgram_image}
fig.tight_layout()



# First config of a sensor: store its settings and size its data and axes.
def handle_config(s, msg):
    s["rate"], s["fft_len"] = msg["sample_rate_hz"], msg["fft_length"]
    s["idx"] = deque(maxlen=int(HISTORY_SECONDS * s["rate"]))
    s["val"] = deque(maxlen=int(HISTORY_SECONDS * s["rate"]))
    s["history"] = deque(maxlen=max(1, round(SPECTROGRAM_SECONDS * s["rate"] / s["fft_len"])))
    max_hz = min(SPECTRUM_MAX_HZ, s["rate"] / 2)
    s["ax_spec"].set(xlim=(0, max_hz), ylabel=msg.get("spectrum_units", "Counts peak"))
    s["ax_sgram"].set(ylim=(0, max_hz))


# Redraw one sensor's three plots from its stored data.
def draw(s):
    rate, fft_len = s["rate"], s["fft_len"]
    if s["idx"]:
        i = np.array(s["idx"])
        s["wave_line"].set_data((i - i[-1]) / rate, s["val"])   # Newest sample at 0 s
        s["ax_wave"].relim()
        s["ax_wave"].autoscale_view(scalex=False)
    if s["spectrum"] is not None:
        s["spec_line"].set_data(np.arange(len(s["spectrum"])) * rate / fft_len, s["spectrum"])
        s["ax_spec"].relim()
        s["ax_spec"].autoscale_view(scalex=False)
    if s["history"]:
        top_bin = int(min(SPECTRUM_MAX_HZ, rate / 2) * fft_len / rate) + 1
        group = -(-top_bin // SPECTROGRAM_ROWS)   # Merge this many bins into one row, keeping the max
        spectra = np.array(s["history"])[:, :top_bin]
        spectra = np.pad(spectra, ((0, 0), (0, -top_bin % group)))
        spectra = spectra.reshape(len(spectra), -1, group).max(axis=2)
        image = 20 * np.log10(spectra.T + 1e-3)
        s["sgram_image"].set_data(image)
        s["sgram_image"].set_extent((-len(s["history"]) * fft_len / rate, 0,
                                     0, image.shape[0] * group * rate / fft_len))
        s["sgram_image"].set_clim(image.max() - 40, image.max())   # Show 40 dB below the strongest value


# Runs every 100 ms: read bytes, split into lines, store each message, redraw.
def update(_frame):
    global buffer

    if TEST_DATA:
        buffer += read_test_data()
    else:
        buffer += port.read(port.in_waiting)
    *lines, buffer = buffer.split(b"\n")      # Complete lines; unfinished end stays in buffer

    for line in lines:
        try:
            msg = json.loads(line)
        except ValueError:
            continue                           # Skip a cut-off first line or an empty line
        if msg["type"] == "error":
            fig.suptitle(f'{msg.get("sensor", "")} {msg["message"]}'.strip(), color="red")
            anim.event_source.stop()
            if port:
                port.close()
            return
        s = sensors.get(msg.get("sensor", "adc"))   # No "sensor" field = "adc" (current firmware)
        if s is None:
            continue                           # Sensor not in SENSORS: ignore
        if msg["type"] == "config" and s["rate"] is None:
            handle_config(s, msg)
        elif s["rate"] is None:
            continue                           # Ignore data until the sensor's config arrives
        elif msg["type"] == "samples":
            first = msg["first_sample_index"]
            s["idx"].extend(range(first, first + len(msg["values"])))
            s["val"].extend(msg["values"])
        elif msg["type"] == "fft":
            s["spectrum"] = np.array(msg["values"])
            s["history"].append(s["spectrum"])

    for s in sensors.values():
        if s["rate"] is not None:
            draw(s)



anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)   # Call update() every 100 ms
plt.show()                     # Opens the window; returns when it is closed
if port:
    port.close()