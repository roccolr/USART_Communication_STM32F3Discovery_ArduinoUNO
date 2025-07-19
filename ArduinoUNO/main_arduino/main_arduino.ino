#define NUM_SAMPLES 256
#define SIGNAL_FREQ 10          // Hz
#define SAMPLE_RATE (SIGNAL_FREQ * NUM_SAMPLES) // 10240 Hz, numero campioni per periodo
#define TWO_PI 6.28318530718

uint8_t indici[NUM_SAMPLES];


void genera_sinusoide_indicizzata(float freq, uint8_t* output, int num_samples, float sample_rate){
    for(int i=0; i<num_samples; i++){
    float t = (float)i / SAMPLE_RATE;
    output[i] = (uint8_t)((sin(TWO_PI * freq* t)+1)/2*255);
  }
}
//void indicizza(float* input, uint8_t* output, int dim){
//  for(int i=0; i<dim; i++){
//    output[i] = (uint8_t)((input[i]+1)/2*255);
//  }
//}

void setup() {
  // put your setup code here, to run once:
  genera_sinusoide_indicizzata(SIGNAL_FREQ, indici, NUM_SAMPLES, SAMPLE_RATE);
  
  pinMode(13, OUTPUT);  // il pin 13 sarà usato per capire come sta andando la tramissione
  digitalWrite(13, LOW);
  Serial.begin(9600);
  Serial.setTimeout(100000);

  for(int i=0; i<NUM_SAMPLES; i++){
    Serial.write(indici[i]);
  }

  // aspetto i risultati dello spettrogramma
  Serial.readBytes(indici, 256);
}

void loop() {
  // put your main code here, to run repeatedly:
    digitalWrite(13,HIGH);
    delay(100);
    digitalWrite(13,LOW);
    delay(100);
}
