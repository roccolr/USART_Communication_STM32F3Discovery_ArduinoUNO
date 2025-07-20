import serial
import time
import numpy as np
from pydub import AudioSegment
import to_spect as sp
import os

PORT = 'COM6'
BAUDRATE = 9600
MP3_PATH = "C:\\Users\\rocco\\Desktop\\USART_Communication_STM32F3Discovery_ArduinoUNO\\master_slave_python\\nature_sounds.mp3"
NUM_SAMPLES = 256

AudioSegment.converter = "C:\\Users\\rocco\\Documents\\ffmpeg\\ffmpeg-7.1.1-full_build\\bin\\ffmpeg.exe"
audio = AudioSegment.from_mp3(MP3_PATH)
audio = audio.set_channels(1).set_frame_rate(16000)  # mono, 16kHz
samples = np.array(audio.get_array_of_samples())
print("[DEBUG] Valori raw 16-bit:", samples[:NUM_SAMPLES])

if len(samples) < NUM_SAMPLES:
    raise ValueError("L'MP3 è troppo corto!")

segment = samples[:NUM_SAMPLES].astype(np.float32)
print(np.max(np.abs(segment)))
segment = segment / np.max(np.abs(segment))   # normalizza tra [-1, 1]
segment = ((segment + 1.0) * 127.5).astype(np.uint8)  # mappa [-1,1] → [0,255]

print("[INFO] Campioni da inviare:", segment)

print(f"[INFO] Connessione alla porta seriale {PORT}...")
ser = serial.Serial(PORT, BAUDRATE, timeout=10)
time.sleep(3)

print("[INFO] Invio dei 256 campioni...")
ser.write(segment.tobytes())

print("[INFO] In attesa di 256 byte in risposta...")
response = ser.read(NUM_SAMPLES)

if len(response) != NUM_SAMPLES:
    print(f"[WARN] Ricevuti solo {len(response)} byte.")
else:
    print("[INFO] Vettore ricevuto correttamente.")

received_vector = np.frombuffer(response, dtype=np.uint8)
print("Vettore ricevuto:", received_vector)

ser.close()


print("[INFO] Comunicazione seriale terminata.")

spec = sp.compute_spectrogram()