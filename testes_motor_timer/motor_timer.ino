#include <Wire.h>
#include <SpinTimer.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include "Nextion.h"

HardwareSerial Serial1(PC11,PC10);

const unsigned int MOTOR_MILLIS = 5;
const unsigned int DISPLAY_MILLIS = 1;
const unsigned int CONTROL_MILLIS = 1000;


// Declarando as variáveis para programação das Telas

//-----Menu Inicial

int menu_inicial = 1;

//-----Configurar_Experimento

int config = 2;

NexButton decremento1 = NexButton(config, 8, "b0");
NexButton incremento1 = NexButton(config, 9, "b1");
NexText vazao1_text = NexText(config, 6, "t9");
NexButton habilita_seringa1 = NexButton(config, 22, "b4");

NexButton incremento2 = NexButton(config, 11, "b6");
NexButton decremento2 = NexButton(config, 10, "b5");
NexText vazao2_text = NexText(config, 19, "t11");
NexButton habilita_seringa2 = NexButton(config, 23, "b7");

NexButton habilita_aquecimento = NexButton(config, 21, "b3");

NexText TEMP1 = NexText(config, 26, "t2");

NexButton ligar_auto = NexButton(config, 12, "b2");

//-----Iniciar Experimento

//int start = 3;

//NexButton play = NexButton(start, 9, "b0");
//NexButton pause = NexButton(start, 10, "b1");
//NexText temp_chip_text = NexText(start, 4, "t2");
//NexText temp_cham_text = NexText(start, 8, "t4");

//Usar switch case para evitar conflitos com nome de botões ou textos
//Sempre que a tela carregar no display enviar qual tela foi carregada
//a partir da tela carregada serão utilizadas as variáveis daquela tela
//NexText vazao1_text = NexText(start, 15, "t9");
//NexText vazao2_text = NexText(start, 16, "t11");




NexTouch *nex_listen_list[] = {
    &decremento1,
    &incremento1,
    &habilita_seringa1,
    &decremento2,
    &incremento2,
    &habilita_seringa2,
    &habilita_aquecimento,
    //&play,
    //&pause,
    //&temp_chip_text,
    //&temp_cham_text, 
    NULL
  };
;

const int TERMAL_PWM = PB3; 

// Definindo as portas a serem utilizadas para o fuso 1 e os seus respectivos nomes
const int clk_pos_1 = PC2;
const int cw_pos_1 = PC3;
const int en_pos_1 = PC1;
const int ligar_motor_1 = PC0;
const int fdc_sup_1 = PC7;
const int fdc_inf_1 = PB6;

// Definindo as portas a serem utilizadas para o fuso 2 e os seus respectivos nomes
const int clk_pos_2 = PA4;
const int cw_pos_2 = PB0;
const int en_pos_2 = PA0;
const int ligar_motor_2 = PA1;
const int fdc_sup_2 = PA7;
const int fdc_inf_2 = PA6;

//Definindo as variáves de temperatura
Adafruit_TMP117  tmp117;
bool Heating = 0;
bool aquecimento_hab =0;

int Saida_teste;
unsigned long tempo;
float a = -1.0/120000.0;
float h = 1200;
float k = 37;
float Setpoint = 25.0;
float start;
double Output = 0;
int contador = 0;


// Definindo demais variáveis para o fuso 1
float vazao_1 = 80;
float passos_1 = 0;
float tempo_1 = 0;
float intervalo_1 = 0;
float contador_tempo_1 = 0;
float contador_passos_1 = 0;

// Definindo demais variáveis para o fuso 2
float vazao_2 = 80;
float passos_2 = 0;
float tempo_2 = 0;
float intervalo_2 = 0;
float contador_tempo_2 = 0;
float contador_passos_2 = 0;

float agora1;
float antes1;

float agora2;
float antes2;

bool AUTO = 1;

bool seringa1_hab = 0;
bool seringa2_hab = 0;


char buffer[10];
char buffer2[10];


class ControlAction : public SpinTimerAction
{
public:
  void timeExpired()
  {
    sensors_event_t temp; // create an empty event to be filled
    tmp117.getEvent(&temp);
    if(Heating == 0 && contador < 10){
      start = millis()/1e3;
      Serial.println(start);
      contador ++;
      if(contador >=10){
        Heating = 1;
      }
    }
    if(Heating == 1){
      float now = millis()/1e3;
      float elaps_time = now-start;
      if(Setpoint <=36.9){
        Setpoint = a * (elaps_time-h) * (elaps_time-h) + k;
      }
      else{
        Setpoint = 36.95;
      }
      
    if(temp.temperature< Setpoint){
      Output = 0.40*PWM_RESOLUTION;
      analogWrite(TERMAL_PWM, Output);
      }
    else{
      Output = 0;
      analogWrite(TERMAL_PWM, Output);
      }

    sensors_event_t temp; // create an empty event to be filled
    //tmp117.getEvent(&temp);
    double temperatura = temp.temperature;
    dtostrf(temperatura,3,1,buffer2);
    const char* Value = buffer2;
    TEMP1.setText(Value);

    }
  }
};

class MotorAction : public SpinTimerAction
{
public:
  void timeExpired()
  {
    //removi as contas da condição (AUTO==0) pois ele só fazia cálculos e mais nenhuma ação
    passos_1 = vazao_1 / 0.03125;
    tempo_1 = 3600000.0 / (2.0 * passos_1);
    intervalo_1 = 3600000.0 / passos_1;
    passos_2 = vazao_2 / 0.03125;
    tempo_2 = 3600000.0 / (2.0 * passos_2);
    intervalo_2 = 3600000.0 / passos_2;

    if(AUTO == 1){
      agora1 = millis();
      if(agora1-antes1 >= tempo_1){
        //Serial.println(agora1);
        //entrando no if ok

        digitalWrite(en_pos_1, HIGH);
        digitalWrite(clk_pos_1, !digitalRead(clk_pos_1)); 
        Serial.println(digitalRead(clk_pos_1));
        digitalWrite(en_pos_1, LOW);
        contador_passos_1++;
        antes1 = agora1; 
      }
      agora2 = millis();
      if(agora2-antes2 >= tempo_2){
        antes2 = agora2;
        digitalWrite(en_pos_2, HIGH);
        digitalWrite(clk_pos_2, !digitalRead(clk_pos_2)); 
        digitalWrite(en_pos_2, LOW);
        contador_passos_2++;
      }

    }

  }
};

class DisplayAction : public SpinTimerAction
{
public:
  void timeExpired()
  {
    nexLoop(nex_listen_list);
  }
};

void decremento1PopCallback(void *ptr){
  if(seringa1_hab){
  vazao_1 --;
  if(vazao_1 < 10){
    vazao_1 = 10;
  }
  dtostrf(vazao_1, 2, 0, buffer);
  const char* valor_vazao1 = buffer;
  vazao1_text.setText(valor_vazao1);
}
}

void incremento1PopCallback(void *ptr){
  if(seringa1_hab){
  vazao_1 ++;
  
  if(vazao_1 > 100){
    vazao_1 = 100;
  }
  dtostrf(vazao_1, 2, 0, buffer);
  const char* valor_vazao1 = buffer;
  vazao1_text.setText(valor_vazao1);
}
}

void decremento2PopCallback(void *ptr){
  if(seringa2_hab){
  vazao_2 --;
  if(vazao_2 < 10){
    vazao_2 = 10;
  }
  dtostrf(vazao_2, 2, 0, buffer);
  const char* valor_vazao2 = buffer;
  vazao2_text.setText(valor_vazao2);
}
}

void incremento2PopCallback(void *ptr){
  if(seringa2_hab){
  vazao_2 ++;

    if(vazao_2 > 100){
      vazao_2 = 100;
    }

  dtostrf(vazao_2, 2, 0, buffer);
  const char* valor_vazao2 = buffer;
  vazao2_text.setText(valor_vazao2);
  }
}

void habilita_seringa1PushCallback(void *ptr){
  if(seringa1_hab == 0){
  
  habilita_seringa1.Set_background_image_pic(5);
  seringa1_hab = 1;
}
  else{
    
    habilita_seringa1.Set_background_image_pic(4);
    seringa1_hab = 0;
  }
  }

void habilita_seringa2PushCallback(void *ptr){
  if(seringa2_hab == 0){
    habilita_seringa2.Set_background_image_pic(5);
    seringa2_hab = 1;
  }
  else{
    habilita_seringa2.Set_background_image_pic(4);
    seringa2_hab = 0;
  }
}

void habilita_aquecimento2PopCallback(void *ptr){

  if (aquecimento_hab == 0){
    habilita_aquecimento.Set_background_image_pic(5);
    aquecimento_hab = 1;
  }
  
  else{
    habilita_aquecimento.Set_background_image_pic(4);
    aquecimento_hab = 0;
  }

}

void setup() {

  Serial.begin(9600);
  tmp117.begin(0x48) ;

  pinMode(TERMAL_PWM, OUTPUT);
  analogWriteResolution(16);
  analogWriteFrequency(200);
  analogWrite(TERMAL_PWM, Output);
  

  // Configurando os pinos do Fuso 1 como entradas ou saídas
  pinMode(clk_pos_1, OUTPUT); 
  pinMode(cw_pos_1, OUTPUT); 
  pinMode(en_pos_1, OUTPUT); 
  pinMode(ligar_motor_1, INPUT); 
  pinMode(fdc_sup_1, INPUT); 
  pinMode(fdc_inf_1, INPUT); 

  // Configurando os pinos do Fuso 2 como entradas ou saídas
  pinMode(clk_pos_2, OUTPUT); 
  pinMode(cw_pos_2, OUTPUT); 
  pinMode(en_pos_2, OUTPUT); 
  pinMode(ligar_motor_2, INPUT); 
  pinMode(fdc_sup_2, INPUT); 
  pinMode(fdc_inf_2, INPUT);
  nexInit();
  tmp117.begin();
  incremento1.attachPop(incremento1PopCallback, &incremento1);
  decremento1.attachPop(decremento1PopCallback, &decremento1);
  incremento2.attachPop(incremento2PopCallback, &incremento2);
  decremento2.attachPop(decremento2PopCallback, &decremento2);
  habilita_seringa1.attachPush(habilita_seringa1PushCallback, &habilita_seringa1);
  habilita_seringa2.attachPush(habilita_seringa2PushCallback, &habilita_seringa2);
  habilita_aquecimento.attachPop(habilita_aquecimento2PopCallback, &habilita_aquecimento);

  // Referenciamento do fuso 1
  Serial.println("Referenciando fuso 1...");
  while (digitalRead(fdc_sup_1) == HIGH) {
  digitalWrite(cw_pos_1, LOW);
    if (digitalRead(ligar_motor_1) == HIGH) {
      digitalWrite(clk_pos_1, LOW);
      delayMicroseconds(300);
      digitalWrite(clk_pos_1, HIGH);
      delayMicroseconds(300);
  }
}

  digitalWrite(cw_pos_1, HIGH);
  Serial.println("Posicionando fuso 1...");
  for (int i = 0; i <= 12800; i++){
      digitalWrite(clk_pos_1, LOW);
      delayMicroseconds(300);
      digitalWrite(clk_pos_1, HIGH);
      delayMicroseconds(300);
}

// Referenciamento do fuso 2
  Serial.println("Referenciando fuso 2...");
  while (digitalRead(fdc_sup_2) == HIGH) {
  digitalWrite(cw_pos_2, LOW);
  if (digitalRead(ligar_motor_2) == HIGH) {
    digitalWrite(clk_pos_2, LOW);
    delayMicroseconds(300);
    digitalWrite(clk_pos_2, HIGH);
    delayMicroseconds(300);
  }
}

  digitalWrite(cw_pos_2, HIGH);
  Serial.println("Posicionando fuso 2...");
  for (int i = 0; i <= 12800; i++){
    digitalWrite(clk_pos_2, LOW);
    delayMicroseconds(300);
    digitalWrite(clk_pos_2, HIGH);
    delayMicroseconds(300);
  }

  Serial.println("Referenciamento terminado!");

  new SpinTimer(MOTOR_MILLIS, new MotorAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  new SpinTimer(DISPLAY_MILLIS, new DisplayAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  new SpinTimer(CONTROL_MILLIS, new ControlAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
}// fim do setup

void loop() {
  scheduleTimers();
}



