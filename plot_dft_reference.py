from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    samples = np.array([4, 3, 0, -3, -4, -3, 0, 3], dtype=float)
    spectrum = np.fft.rfft(samples)

    out_dir = Path(__file__).resolve().parents[1] / "docs"
    out_dir.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(7, 3))
    ax.stem(np.arange(len(spectrum)), np.abs(spectrum), basefmt=" ")
    ax.set_title("Reference 8-Point DFT Spectrum")
    ax.set_xlabel("Bin")
    ax.set_ylabel("Magnitude")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "dft_reference.png", dpi=150)


if __name__ == "__main__":
    main()
