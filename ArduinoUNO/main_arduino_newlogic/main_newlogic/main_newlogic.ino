/*
This code generates a sinusoidal function at a given frequency, and then sends it via USART to another microcontroller. 
Then it waits for the Transformed signal. 
*/

#define     DURATA          1
#define     FREQ            1
#define     NUM_SAMPLES     128
#define     FREQ_CAMP       NUM_SAMPLES/DURATA
#define     TWO_PI          6.28318530718
#define     NUM_FIN         105
#define     TRSM            13                      // lamp -> trasmissione in corso, high -> trasmissione avvenuta con successo
#define     REQ             8
#define     ACK             7

void genera_sinusoide_quantizzata(float, float, int, uint8_t *);
void toggle(int, float);

int data_rec;
int count = 0;
uint8_t   indici[NUM_SAMPLES];

void setup() {
    genera_sinusoide_quantizzata(FREQ, FREQ_CAMP, NUM_SAMPLES, indici);


    //inizializzazione PIN
    pinMode(TRSM, OUTPUT);
    pinMode(REQ,OUTPUT);
    pinMode(ACK, INPUT);
    digitalWrite(REQ,LOW);
    digitalWrite(TRSM, LOW);

    Serial.begin(115200);
    Serial.setTimeout(1000);

    //inviamo massimo e minimo 
    int32_t massimo = 1;
    int32_t minimo = -1;
    for(int i=0; i<4; i++){
        Serial.write((uint8_t)((massimo >> (8 * i)) & 0xFF));
    }
    // toggle(TRSM,500);
    for(int i=0; i<4; i++){
        Serial.write((uint8_t)((minimo >> (8 * i)) & 0xFF));
    }
    // toggle(TRSM,1000);
    
    delay(100);

    //inviamo 105 finestre contenenti la stessa sinusoide
    digitalWrite(TRSM, HIGH);
    for(int i=0; i<NUM_FIN; i++){
        for(int j=0; j<NUM_SAMPLES; j++){
            Serial.write(indici[j]);
            // toggle(TRSM, 50);
        }
        // delay(10);
    }

    digitalWrite(TRSM,LOW);

    // aspetto i risultati dello spettrogramma 
    for(int i=0; i<NUM_FIN; i++){
        digitalWrite(REQ,HIGH);
        while(digitalRead(ACK) == LOW){

        }
        data_rec=Serial.readBytes(indici, 128);
        digitalWrite(REQ,LOW);
    }


}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.println(indici[count]);
    toggle(TRSM,500);
    count++;
}

void genera_sinusoide_quantizzata(float freq, float freq_camp, int num_samples, uint8_t * indici){
    for(int i=0; i<num_samples; i++){
        indici[i] = (uint8_t)((sin(TWO_PI*freq*i/freq_camp)+1)/2*255);  //  [-1,1] -> [0,255]
    }
}

void toggle(int n_pin, int time){
    digitalWrite(n_pin, HIGH);
    delay(time);
    digitalWrite(n_pin, LOW);
    delay(time);
}