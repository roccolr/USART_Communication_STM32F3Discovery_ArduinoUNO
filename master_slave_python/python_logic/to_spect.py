import numpy as np
import matplotlib.pyplot as plt

def compute_spectrogram(samples, fft_size=512, hop_size=128, spec_height=256, spec_width=1024):
    """
    Calcola uno spettrogramma 256x1024 da una sequenza di campioni audio.
    
    Args:
        samples (np.ndarray): array mono dei campioni audio (float32 o int16).
        fft_size (int): dimensione della FFT (es. 512, minimo 256).
        hop_size (int): step tra una finestra e l'altra (es. 128).
        spec_height (int): numero di righe (frequenze).
        spec_width (int): numero di colonne (finestre).
        
    Returns:
        np.ndarray: spettrogramma 2D di forma (256, 1024), normalizzato tra 0 e 1.
    """
    spectrogram = np.zeros((spec_height, spec_width), dtype=np.float32)

    window = np.hanning(fft_size)

    for i in range(spec_width):
        start = i * hop_size
        end = start + fft_size
        if end > len(samples):
            break
        segment = samples[start:end] * window
        spectrum = np.fft.rfft(segment)  # output: fft_size//2+1
        magnitude = np.abs(spectrum)[:spec_height]  # taglia a 256
        spectrogram[:, i] = magnitude

    # Normalizza logaritmicamente (tipo dB scale)
    spectrogram = 20 * np.log10(spectrogram + 1e-6)
    spectrogram -= spectrogram.min()
    spectrogram /= spectrogram.max()

    return spectrogram
