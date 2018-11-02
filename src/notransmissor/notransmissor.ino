#include "RF24.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "LowPower.h"
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
Adafruit_BME280 bme;
#define LIMITERESPOSTA 1000
#define PINOPRESENCA 3

RF24 radio(9,8);
const uint64_t PIPE_COMANDO = 0xE8E8F0F0E1LL;
const uint64_t PIPE_RESPOSTA = 0xE8E8F0F0E2LL;

typedef struct{
  long temperatura;
  long umidade;
  long pressao;
  long nivelBateria;
  long id;
  bool presenca;
}payload;

typedef struct{
  long tempoEspera;
  bool recebido;
  bool novaConfiguracao;
}resposta;

payload informacoes;


void setup() {
   
  pinMode (PINOPRESENCA,INPUT);
  inicializaSensorTemp();
  inicializaTransmissorRF();
  int valor = digitalRead(PINOPRESENCA);


}

void inicializaSensorTemp(){
    if (! bme.begin()) {

    }
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
}


void inicializaTransmissorRF(){
    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);  
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.openWritingPipe(PIPE_COMANDO);
    radio.openReadingPipe(1,PIPE_RESPOSTA);
    radio.powerDown();
    delay(5);
}


void transmitirDados(int presenca){
  radio.powerUp();
  delay(5);
  atualizarInformacoes(presenca);
  radio.stopListening();
  bool ok = radio.write( &informacoes, sizeof(payload) );
}


void aguardarResposta(){
  radio.startListening();
  delay(5);
  unsigned long inicioEspera = millis();
  bool timeout = false;
  while (!radio.available() && !timeout){
    if (millis()-inicioEspera >= LIMITERESPOSTA){
      timeout = true;
    }
  }
  if (timeout == true){
  }
  else{
    resposta respostaGateway;
    radio.read(&respostaGateway,sizeof(resposta));

  }
  radio.stopListening();
  radio.powerDown();
}


void loop() {
 
  for (int i=0; i<1600; i++){
     LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
     checarLampada();
  }
  
  bme.takeForcedMeasurement();
  transmitirDados(0);
  aguardarResposta();   
}


void checarLampada(){
  int valor = digitalRead(PINOPRESENCA);
  if (valor==1){
      bme.takeForcedMeasurement();
      transmitirDados(1);
      aguardarResposta();
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

void atualizarInformacoes(int presenca){
  informacoes.id = 1;
  informacoes.temperatura = (long) 100*bme.readTemperature();
  informacoes.pressao = (long) (bme.readPressure());
  informacoes.umidade = (long) 100*bme.readHumidity();
  informacoes.nivelBateria = (long) readVcc();
  if (presenca==1) {
    informacoes.presenca =true;
  }
  else{
    informacoes.presenca =false;
  }
}

int readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}







