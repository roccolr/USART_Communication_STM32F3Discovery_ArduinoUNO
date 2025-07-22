#ifndef ARM_FFT
#define ARM_FFT

#include "fft_math.h"

void arm_fft(Complesso*, Complesso* ,int);
int pad_to_pow2(const Complesso*, int, Complesso*);
void fft_to_spectrogram(const Complesso*, float*, int, float);

#endif 