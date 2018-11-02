#include "RF24.h"
RF24 radio(9,10);
const uint64_t PIPE_COMANDO = 0xE8E8F0F0E1LL;
const uint64_t PIPE_RESPOSTA = 0xE8E8F0F0E2LL;

typedef struct{
  int sleepTime;
  bool recebido;
  bool newConfig;
}resposta;


typedef struct{
  long temperatura;
  long umidade;
  long pressao;
  long nivelBateria;
  long id;
  bool presenca;
}payload;

payload meusDados;


void inicializaTransmissorRF(){
    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.openWritingPipe(PIPE_RESPOSTA);
    radio.openReadingPipe(1,PIPE_COMANDO); 
}


void mostraInfomacoes (payload info){
  Serial.print("Id: ");
  Serial.println(info.id);
  Serial.print("Temperatura: ");
  Serial.println((float)info.temperatura/100);
  Serial.print("umidade: ");
  Serial.println((float)info.umidade/100);
  Serial.print("Pressão: ");
  Serial.println((float)info.pressao/100);
  Serial.print("Tensão: ");
  Serial.println((float)info.nivelBateria/1000);
}


void verificaPacotesRF(){
  radio.startListening();
  if (radio.available()>0){
    radio.read(&meusDados,sizeof(payload));
    mostraInfomacoes(meusDados);
    radio.stopListening();
    resposta resp;
    resp.recebido = true;
    resp.newConfig = false;
    resp.sleepTime=0;
    radio.write(&resp,sizeof(resposta));
    delay(5);
  }
}


void setup() {
  Serial.begin(9600);
  inicializaTransmissorRF();
}

void loop() {
  verificaPacotesRF();
  delay(1000);
}
