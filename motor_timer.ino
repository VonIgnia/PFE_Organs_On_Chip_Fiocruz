#include <Wire.h>
#include <SpinTimer.h>


const unsigned int MOTOR_MILLIS = 5;


// Definindo as portas a serem utilizadas para o fuso 1 e os seus respectivos nomes
const int clk_pos_1 = D9;
const int cw_pos_1 = D3;
const int en_pos_1 = D4;
const int ligar_motor_1 = D5;
const int fdc_sup_1 = D6;
const int fdc_inf_1 = D7;

// Definindo as portas a serem utilizadas para o fuso 2 e os seus respectivos nomes
const int clk_pos_2 = D15;
const int cw_pos_2 = D14;
const int en_pos_2 = D10;
const int ligar_motor_2 = D11;
const int fdc_sup_2 = D12;
const int fdc_inf_2 = A0;

// Definindo demais variáveis para o fuso 1
float vazao_1 = 1000;
float passos_1 = 0;
float tempo_1 = 0;
float intervalo_1 = 0;
float contador_tempo_1 = 0;
float contador_passos_1 = 0;

// Definindo demais variáveis para o fuso 2
float vazao_2 = 500;
float passos_2 = 0;
float tempo_2 = 0;
float intervalo_2 = 0;
float contador_tempo_2 = 0;
float contador_passos_2 = 0;

float agora1;
float antes1;

float agora2;
float antes2;

bool AUTO = 0;


class MotorAction : public SpinTimerAction
{
public:
  void timeExpired()
  {
    if( AUTO == 0){
      passos_1 = vazao_1 / 0.03125;
      tempo_1 = 3600000.0 / (2.0 * passos_1);
      intervalo_1 = 3600000.0 / passos_1;
      passos_2 = vazao_2 / 0.03125;
      tempo_2 = 3600000.0 / (2.0 * passos_2);
      intervalo_2 = 3600000.0 / passos_2;

    }

    if(AUTO == 1){
      agora1 = millis();
      if(agora1-antes1 >= tempo_1){
        antes1 = agora1;
        digitalWrite(en_pos_1, HIGH);
        digitalWrite(clk_pos_1, !digitalRead(clk_pos_1)); 
        digitalWrite(en_pos_1, LOW);
        contador_passos_1++;
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








void setup() {
 
  Serial.begin(9600);

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



  new SpinTimer(MOTOR_MILLIS, new MotorAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  scheduleTimers();

}
