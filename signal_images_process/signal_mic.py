import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =====================
# Paramètres audio
# =====================
fs = 44100
duration = 0.1
blocksize = int(fs * duration)

# Buffer long pour le spectrogramme
spec_duration = 2.0  # secondes affichées
spec_size = int(fs * spec_duration)
audio_history = np.zeros(spec_size)

# =====================
# Figure
# =====================
fig, (ax_fft, ax_spec) = plt.subplots(2, 1, figsize=(10, 6))

# ----- FFT -----
freqs = np.fft.rfftfreq(blocksize, 1 / fs)
fft_line, = ax_fft.plot(freqs, np.zeros_like(freqs))
ax_fft.set_xlim(0, 5000)
ax_fft.set_ylim(0, 0.05)
ax_fft.set_title("FFT temps réel")
ax_fft.set_xlabel("Fréquence (Hz)")
ax_fft.set_ylabel("Amplitude")

# ----- Spectrogramme -----
ax_spec.set_title("Spectrogramme (specgram)")
ax_spec.set_xlabel("Temps (s)")
ax_spec.set_ylabel("Fréquence (Hz)")
ax_spec.set_ylim(0, 5000)

plt.tight_layout()

# =====================
# Buffer audio courant
# =====================
audio_buffer = np.zeros(blocksize)

def audio_callback(indata, frames, time, status):
    global audio_buffer, audio_history

    audio_buffer = indata[:, 0].copy()

    # Décalage du buffer long
    audio_history = np.roll(audio_history, -blocksize)
    audio_history[-blocksize:] = audio_buffer

# =====================
# Update graphique
# =====================
def update(frame):
    # FFT
    fft = np.abs(np.fft.rfft(audio_buffer)) / blocksize
    fft_line.set_ydata(fft)

    # Spectrogramme
    ax_spec.cla()  # important avec specgram
    ax_spec.specgram(
        audio_history,
        NFFT=2048,
        Fs=fs,
        noverlap=512,
        cmap="magma",
        scale="dB"
    )

    ax_spec.set_ylim(0, 1000)
    ax_spec.set_ylabel("Fréquence (Hz)")
    ax_spec.set_xlabel("Temps (s)")
    ax_spec.set_title("Spectrogramme (specgram)")

    return fft_line,

# =====================
# Stream audio
# =====================
stream = sd.InputStream(
    channels=1,
    samplerate=fs,
    blocksize=blocksize,
    callback=audio_callback
)

with stream:
    ani = FuncAnimation(fig, update, interval=50, blit=False)
    plt.show()
#Signed by Z.ABA