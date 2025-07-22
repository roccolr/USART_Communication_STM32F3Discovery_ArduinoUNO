import serial
import time
import numpy as np
from pydub import AudioSegment
from pydub.utils import mediainfo
import to_spect as sp
import matplotlib.pyplot as plt

PORT = 'COM6'
BAUDRATE = 9600
MP3_PATH = "mp3 path here"
NUM_SAMPLES = 128
AudioSegment.converter = "yout path to \\ffmpeg.exe"


if(__name__ == '__main__'):
    # info traccia
    print('[MAIN]\tottenendo informazioni dalla traccia...')
    info = mediainfo(MP3_PATH)
    duration_sec = float(info['duration'])
    sample_rate = int(info['sample_rate'])
    total_samples = int(duration_sec*sample_rate)
    print(f'[MAIN]\tDurata: {duration_sec}\tsample_rate: {sample_rate}\tnumero_campioni: {total_samples}')

    # calcolo campioni da ottenere
    # n_finestre_temporali = 105, n_elementi_finestra = 128
    # totale_campioni = 105*128 = 13440 codificati come uint8_t
    new_frame_rate = int(13440/duration_sec)
    print(f'[MAIN]\tnuova frequenza di campionamento: {new_frame_rate}')
    audio = AudioSegment.from_mp3(MP3_PATH)
    audio = audio.set_channels(1).set_frame_rate(new_frame_rate)
    samples = np.array(audio.get_array_of_samples())
    print(f'[MAIN]\tcampioni ottenuti: {len(samples)}, tipo: {samples.dtype}')

    # creazione struttura dati 
    n_windows = 105
    data_structure = np.zeros((105,128), np.int16)
    fourier = np.zeros((105,128), np.uint8)

    # fill struttura dati 
    k = 0
    for i in range(0,105):
        for j in range(0,128):
            try:
                data_structure[i,j] = samples[k]
                k+=1
            except IndexError as e:
                data_structure[i,j] = 0
    # print(f'[DEBUG]\tMatrice di dimensioni:{data_structure.shape}\n{data_structure}')
    print(f'[MAIN]\tstruttura dati popolata')
    max = np.int32(data_structure.max())
    min = np.int32(data_structure.min())
    info = np.int64(max or min<<32)
    print(f'[MAIN]\tmax={max}, min={min}, codificati su 32 bit con segno')

    # normalizzazione
    data_structure = (data_structure-min)/(max-min)     # [min,max] -> [0,1]
    data_structure = np.clip(np.round(data_structure*255), 0, 255).astype(np.uint8)     # [0,1] -> [0,255]
    # print(f'[DEBUG]\tMatrice di dimensioni:{data_structure.shape}\n{data_structure}')

    # inizio business logic 
    ser = serial.Serial(PORT, BAUDRATE, timeout=10)
    # time.sleep(3)

    #invio massimo e minimo, in totale 8 byte 
    print('[MAIN]\tinvio metadati...')
    ser.write(info.tobytes())    # invia 4 byte
    print('[MAIN]\tinviati metadati...')

    # time.sleep(2)
    # invio dati vero e proprio 
    for i in range(0,105):
        print(f'[MAIN]\tinvio finestra {i}...')
        for j in range(0,128):
            ser.write(data_structure[i,j].tobytes())
        # time.sleep(0.1)
    print('[MAIN]\tdati inviati')

    # ricezione stft
    for i in range(0,105):
        print(f'[MAIN]\tricezione finestra {i}...')
        raw = ser.read(128)
        fourier[i] = np.frombuffer(raw, np.uint8)
    print('[MAIN]\tdati ricevuti')

    ser.close()

    spect = sp.enhance_spectrogram(fourier)
    print(fourier)

    plt.figure()
    plt.imshow(spect, cmap='gray', clim=[0,255])
    plt.show()
    
