import csv
import glob
import os
import numpy as np
import matplotlib.pyplot as plt

# Find the latest CSV file
csv_files = glob.glob(os.path.join(os.path.dirname(__file__), "Data_records", "*.csv"))
latest_csv = max(csv_files, key=os.path.getmtime)
print(f"Analyzing: {os.path.basename(latest_csv)}")

# Load data
with open(latest_csv) as f:
    reader = csv.DictReader(f)
    rows = list(reader)

TS = 100e-6  # 100 µs sampling period
FS = 1.0 / TS  # 10 kHz sample rate
F0 = 50.0

vgrid = np.array([float(r["Vgrid"]) for r in rows])
igrid = np.array([float(r["Igrid"]) for r in rows])
N = len(vgrid)
t = np.arange(N) * TS * 1e3  # ms

# FFT
freqs = np.fft.rfftfreq(N, d=TS)
Vgrid_fft = np.abs(np.fft.rfft(vgrid)) * 2 / N
Igrid_fft = np.abs(np.fft.rfft(igrid)) * 2 / N

# Print harmonic content
print(f"\n{'Harmonic':<12} {'Frequency (Hz)':<18} {'Vgrid (V)':<14} {'Igrid (A)':<12} {'THD contrib'}")
print("-" * 72)
fundamental_v = None
for h in range(1, 12):
    freq = h * F0
    idx = int(round(freq / (FS / N)))
    if idx >= len(Vgrid_fft):
        break
    v_amp = Vgrid_fft[idx]
    i_amp = Igrid_fft[idx]
    if h == 1:
        fundamental_v = v_amp
    thd_pct = (v_amp / fundamental_v * 100) if fundamental_v else 0
    print(f"  h={h:<8} {freq:<18.1f} {v_amp:<14.4f} {i_amp:<12.4f} {thd_pct:.2f}%")

harmonics_sq = sum(
    Vgrid_fft[int(round(h * F0 / (FS / N)))]**2
    for h in range(2, 12)
    if int(round(h * F0 / (FS / N))) < len(Vgrid_fft)
)
thd_total = np.sqrt(harmonics_sq) / fundamental_v * 100 if fundamental_v else 0
print(f"\nTotal THD (h2-h11): {thd_total:.2f}%")

# Plot
fig, axes = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle(f"FFT Analysis — {os.path.basename(latest_csv)}", fontsize=12)

# Time domain
ax = axes[0]
ax.plot(t, vgrid, label="Vgrid (V)", color="steelblue")
ax.plot(t, igrid * 5, label="Igrid × 5 (A)", color="orange", alpha=0.8)
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Amplitude")
ax.set_title("Time domain")
ax.legend()
ax.grid(True)

# FFT spectrum (log scale, 0–1000 Hz)
ax = axes[1]
mask = freqs <= 1000
ax.semilogy(freqs[mask], Vgrid_fft[mask], color="steelblue", label="Vgrid")
ax.semilogy(freqs[mask], Igrid_fft[mask], color="orange", label="Igrid", alpha=0.8)
for h in range(1, 12):
    freq = h * F0
    if freq <= 1000:
        ax.axvline(freq, color="red", linestyle="--", alpha=0.3, linewidth=0.8)
        ax.text(freq + 5, ax.get_ylim()[0] * 10, f"h{h}", fontsize=7, color="red")
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Amplitude (V or A)")
ax.set_title("FFT spectrum (log scale, 0–1000 Hz)")
ax.legend()
ax.grid(True, which="both")

# Bar chart of harmonic magnitudes
ax = axes[2]
harmonic_nums = list(range(1, 12))
v_amps = []
for h in harmonic_nums:
    idx = int(round(h * F0 / (FS / N)))
    v_amps.append(Vgrid_fft[idx] if idx < len(Vgrid_fft) else 0)
bars = ax.bar(harmonic_nums, v_amps, color="steelblue", edgecolor="black", linewidth=0.5)
ax.set_xlabel("Harmonic number")
ax.set_ylabel("Vgrid amplitude (V)")
ax.set_title("Harmonic content of Vgrid")
ax.set_xticks(harmonic_nums)
ax.set_xticklabels([f"h{h}\n({h*F0:.0f} Hz)" for h in harmonic_nums], fontsize=8)
ax.grid(True, axis="y")
for bar, amp in zip(bars, v_amps):
    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.05,
            f"{amp:.2f}", ha="center", va="bottom", fontsize=8)

plt.tight_layout()
plt.savefig(os.path.join(os.path.dirname(__file__), "fft_analysis.png"), dpi=150)
print("\nPlot saved to src/fft_analysis.png")
plt.show()
