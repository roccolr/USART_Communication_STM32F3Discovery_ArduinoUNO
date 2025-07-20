import serial
import time
import numpy as np
from pydub import AudioSegment
from pydub.utils import mediainfo
import to_spect as sp
import matplotlib.pyplot as plt

PORT = 'COM6'
BAUDRATE = 9600
MP3_PATH = "C:\\Users\\rocco\\Desktop\\USART_Communication_STM32F3Discovery_ArduinoUNO\\master_slave_python\\nature_sounds.mp3"
NUM_SAMPLES = 256
AudioSegment.converter = "C:\\Users\\rocco\\Documents\\ffmpeg\\ffmpeg-7.1.1-full_build\\bin\\ffmpeg.exe"


if(__name__ == '__main__'):
    # info traccia
    print('[MAIN]\tottenendo informazioni dalla traccia...')
    info = mediainfo(MP3_PATH)
    duration_sec = float(info['duration'])
    sample_rate = int(info['sample_rate'])
    total_samples = int(duration_sec*sample_rate)
    print(f'[MAIN]\tDurata: {duration_sec}\tsample_rate: {sample_rate}\tnumero_campioni: {total_samples}')

    # calcolo campioni da ottenere
    # n_finestre_temporali = 210, n_elementi_finestra = 256
    # totale_campioni = 210*256 = 53760 codificati come uint8_t
    new_frame_rate = int(53760/duration_sec)
    print(f'[MAIN]\tnuova frequenza di campionamento: {new_frame_rate}')
    audio = AudioSegment.from_mp3(MP3_PATH)
    audio = audio.set_channels(1).set_frame_rate(new_frame_rate)
    samples = np.array(audio.get_array_of_samples())
    print(f'[MAIN]\tcampioni ottenuti: {len(samples)}, tipo: {samples.dtype}')

    # creazione struttura dati 
    n_windows = 210
    data_structure = np.zeros((210,256), np.int16)
    fourier = np.zeros((210,256), np.uint8)

    # fill struttura dati 
    k = 0
    for i in range(0,210):
        for j in range(0,256):
            try:
                data_structure[i,j] = samples[k]
                k+=1
            except IndexError as e:
                data_structure[i,j] = 0
    # print(f'[DEBUG]\tMatrice di dimensioni:{data_structure.shape}\n{data_structure}')
    print(f'[MAIN]\tstruttura dati popolata')
    max = np.int32(data_structure.max())
    min = np.int32(data_structure.min())
    print(f'[MAIN]\tmax={max}, min={min}, codificati su 32 bit con segno')

    # normalizzazione
    data_structure = (data_structure-min)/(max-min)     # [min,max] -> [0,1]
    data_structure = np.clip(np.round(data_structure*255), 0, 255).astype(np.uint8)     # [0,1] -> [0,255]
    # print(f'[DEBUG]\tMatrice di dimensioni:{data_structure.shape}\n{data_structure}')

    # inizio business logic 
    ser = serial.Serial(PORT, BAUDRATE, timeout=10)
    time.sleep(3)

    #invio massimo e minimo, in totale 8 byte 
    print('[MAIN]\tinvio metadati')
    ser.write(max.tobytes())    # invia 4 byte
    time.sleep(0.5)             
    ser.write(min.tobytes())    # invia 4 byte
    time.sleep(0.5)             


    # invio dati vero e proprio 
    for i in range(0,210):
        print(f'[MAIN]\tinvio finestra {i}')
        ser.write(data_structure[i].tobytes())
    
    print('[MAIN]\tdati inviati')

    # ricezione stft
    for i in range(0,210):
        print(f'[MAIN]\tricezione finestra {i}')
        fourier[i] = ser.read(256)
    print('[MAIN]\tdati ricevuti')

    ser.close()

    spect = sp.enhance_spectrogram(fourier)

    plt.figure()
    plt.imshow(spect, cmap='jet', clim=[0,255])
    plt.show()
    