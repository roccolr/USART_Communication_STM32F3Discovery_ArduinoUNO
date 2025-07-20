import numpy as np 

def enhance_spectrogram(x:np.ndarray) -> np.ndarray:
    # reverse di ogni riga 
    for i in range(0,105):
        x[i] = x[i][::-1]
    
    x = x.T     # [105,128] -> [128, 105]

    y = np.zeros((128,210))

    for i in range(0,105):
        y[:,2*i] = x[:,i]
        y[:,2*i+1] = y[:,2*i]
    return y 

