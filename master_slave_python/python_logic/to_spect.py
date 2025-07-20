import numpy as np 

def enhance_spectrogram(x:np.ndarray) -> np.ndarray:
    # reverse di ogni riga 
    for i in range(0,210):
        x[i] = x[i][::-1]
    
    x = x.T

    y = np.zeros((256,420))
    for i in range(0,420):
        if(i%2 == 0):
            y[:,i] = x[:,i]
        else:
            y[:,i] = x[:, i-1]

    return y 

