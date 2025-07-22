#ifndef COMPLESSI_H
#define COMPLESSI_H

#include <math.h>
#define PI 3.14159265358979323846f

typedef struct {
    float re;
    float im;
} Complesso;


// Operazioni sui numeri complessi
Complesso esponenziale_negj(int, int);
Complesso somma_c(Complesso, Complesso);
Complesso sottrazione_c(Complesso, Complesso);
Complesso prodotto_c(Complesso, Complesso);
Complesso coniugato_c(Complesso);
float modulo_c(Complesso);
float fase_c(Complesso);  // restituisce la fase in radianti

#endif
