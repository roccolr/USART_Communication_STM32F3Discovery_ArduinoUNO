/*
This code generates a sinusoidal function at a given frequency, and then sends it via USART to another microcontroller. 
Then it waits for the Transformed signal. 
*/

#define   DURATA        1
#define   FREQ          1
#define   NUM_SAMPLES   128
#define   FREQ_CAMP     NUM_SAMPLES/DURATA
#define   TWO_PI        6.28318530718
#define   NUM_FIN       105

void genera_sinusoide_quantizzata(float, float, int, uint8_t *);

void setup() {
    uint8_t   indici[NUM_SAMPLES];
    genera_sinusoide_quantizzata(FREQ, FREQ_CAMP, NUM_SAMPLES, indici);

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    Serial.begin(9600);
    Serial.setTimeout(100000);

    //inviamo 105 finestre contenenti la stessa sinusoide
    for(int i=0; i<NUM_FIN; i++){
        for(int j=0; j<NUM_SAMPLES; j++){
            Serial.write(indici[j]);
        }
        delay(10);
    }

    digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);
    delay(100);

    // aspetto i risultati dello spettrogramma 
    // lampeggia ogni volta che ricevo i risultati di una finestra
    for(int i=0; i<NUM_FIN; i++){
        Serial.readBytes(indici, 128);
        digitalWrite(13,HIGH);
        delay(100);
        digitalWrite(13,LOW);
        delay(100);
    }


}

void loop() {
  // put your main code here, to run repeatedly:
}

void genera_sinusoide_quantizzata(float freq, float freq_camp, int num_samples, uint8_t * indici){
    for(int i=0; i<num_samples; i++){
        indici[i] = (sin(TWO_PI*freq*i/freq_camp)+1)/2*255;  //  [-1,1] -> [0,255]
    }
}