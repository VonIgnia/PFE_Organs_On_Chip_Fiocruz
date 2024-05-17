#include <Wire.h>
#include <SpinTimer.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include "Nextion.h"

//Portas RX e TX, respectivamente nas quais o display está conectado com a placa núcleo
HardwareSerial Serial1(PC11,PC10);

#define PWM_RESOLUTION 65535

const unsigned int MOTOR_MILLIS = 5;
const unsigned int DISPLAY_MILLIS = 1;
const unsigned int CONTROL_MILLIS = 1000;


//Variáveis da tela de controle manual
const int conman = 2;
NexButton habilita_seringa1 = NexButton(conman, 11, "hab1");
NexButton referencia_seringa1 = NexButton(conman, 7, "ref1");
NexButton sobe1 = NexButton(conman, 3, "up1");
NexButton desce1 = NexButton(conman, 4, "down1");

NexButton habilita_seringa2 = NexButton(conman, 12, "hab2");
NexButton referencia_seringa2 = NexButton(conman, 8, "ref2");
NexButton sobe2 = NexButton(conman, 5, "up2");
NexButton desce2 = NexButton(conman, 6, "down2");

NexPicture habilitado_1 = NexPicture(conman,15,"ledhab1");
NexPicture habilitado_2 = NexPicture(conman,21,"ledhab2");
NexPicture referenciado_1_pic = NexPicture(conman,16,"ledref1");
NexPicture referenciado_2_pic = NexPicture(conman,22,"ledref2");
//NexButton ligar_auto = NexButton(conman, 12, "b2");


//Variáveis da tela de configurar experimento
const int configexp = 3;
NexButton play = NexButton(configexp, 20, "playbt");
NexButton pause = NexButton(configexp, 21, "pausebt");

NexButton incremento1 = NexButton(configexp, 8, "inc1");
NexButton decremento1 = NexButton(configexp, 7, "dec1");
NexText vazao1_text = NexText(configexp, 5, "vaz1");
    
NexButton incremento2 = NexButton(configexp, 10, "inc2");
NexButton decremento2 = NexButton(configexp, 9, "dec2");
NexText vazao2_text = NexText(configexp, 15, "vaz2");

NexText temp_chip = NexText(configexp, 19, "tempchip");
NexText temp_chamber = NexText(configexp, 14, "tempcam");


//Listen-list do nextion
NexTouch *nex_listen_list[] = {
    &habilita_seringa1,
    &referencia_seringa1,
    &sobe1,
    &desce1,
    &habilita_seringa2,
    &referencia_seringa2,
    &sobe2,
    &desce2,
    &incremento1,
    &decremento1,
    &incremento2,
    &decremento2,
    &play,
    &pause,
    //&ligar_auto,
    NULL // This line indicates the end of the list
};

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

//Adicionando as variáveis do sensor de temperatura
    Adafruit_TMP117  tmp117;
    Adafruit_TMP117  tmp117_monitor;

//Adicionando a saída do aquecimento
const int TERMAL_PWM = PB3;
//Variaveis de controle do aquecimento
    unsigned long tempo;
    float a = -1.0/120000.0;
    float h = 1200;
    float k = 37;
    float Setpoint = 25.0;
    float start;
    double Output = 0;





// Definindo demais variáveis para o fuso 1
    float vazao_1 = 10;
    float passos_1 = 0;
    float tempo_1 = 0;
    float intervalo_1 = 0;
    float contador_tempo_1 = 0;
    float contador_passos_1 = 0;

// Definindo demais variáveis para o fuso 2
    float vazao_2 = 10;
    float passos_2 = 0;
    float tempo_2 = 0;
    float intervalo_2 = 0;
    float contador_tempo_2 = 0;
    float contador_passos_2 = 0;

//Definindo as variáveis para a operação manual
    float agora1;
    float antes1;
    float agora2;
    float antes2;

    bool AUTO = 0;
    bool heating = 0;

    bool seringa1_hab = 0;
    bool seringa2_hab = 0;
    bool referenciado_1 = 0;
    bool referenciado_2 = 0;

    int tela_atual = 0;
    bool subindo_manual_1 = 0;
    bool descendo_manual_1 = 0;
    bool subindo_manual_2 = 0;
    bool descendo_manual_2 = 0;

    char buffer[10];
    char buffer2[10];
    char buffer3[10];

class MotorAction : public SpinTimerAction{
    public:
    void timeExpired(){
        if (seringa1_hab == 1){
        digitalWrite(ligar_motor_1,HIGH);}

        if (seringa2_hab == 1){
        digitalWrite(ligar_motor_2,HIGH);}

        digitalWrite(cw_pos_1, HIGH);//descendo
        digitalWrite(cw_pos_2, HIGH);//descendo

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
        if(agora1-antes1 >= tempo_1 && seringa1_hab == 1){
            antes1 = agora1;
            
            
            digitalWrite(en_pos_1, LOW); //enable pra rodar tem que ser LOW
            
            digitalWrite(clk_pos_1, !digitalRead(clk_pos_1));

            
            //digitalWrite(en_pos_1, HIGH);
            contador_passos_1++;
        }
        else{digitalWrite(en_pos_1, HIGH);}
        agora2 = millis();
        if(agora2-antes2 >= tempo_2 && seringa2_hab == 1){
            antes2 = agora2;
            
            digitalWrite(en_pos_2, LOW);

            digitalWrite(clk_pos_2, !digitalRead(clk_pos_2));

            
            //digitalWrite(en_pos_2, HIGH);
            contador_passos_2++;
        }
        else{digitalWrite(en_pos_2, HIGH);}
        }

    }
};

class DisplayAction : public SpinTimerAction{
    public:
    void timeExpired()
    {

        nexLoop(nex_listen_list);
    }
};

class ControlAction : public SpinTimerAction{
    public:
    void timeExpired()
    {
        sensors_event_t temp; // create an empty event to be filled
        tmp117.getEvent(&temp);
        double temperatura = temp.temperature;
        dtostrf(temperatura,3,1,buffer2);
        const char* Value = buffer2;
        temp_chip.setText(Value);

        sensors_event_t temp2; // create an empty event to be filled
        tmp117_monitor.getEvent(&temp2);
        double temperatura2 = temp2.temperature;
        dtostrf(temperatura2,3,1,buffer3);
        const char* Value2 = buffer3;
        temp_chamber.setText(Value2);

        if(AUTO == 0 ){
        start = millis()/1e3;
        }

        if(AUTO == 1){
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
        }

    }
};

void setup() {
 
  Serial.begin(9600);
  

  // Configurando os pinos do Fuso 1 como entradas ou saídas
  pinMode(clk_pos_1, OUTPUT); 
  pinMode(cw_pos_1, OUTPUT); 
  pinMode(en_pos_1, OUTPUT);
   
  pinMode(ligar_motor_1, OUTPUT);
  digitalWrite(ligar_motor_1, HIGH);

  pinMode(fdc_sup_1, INPUT); 
  pinMode(fdc_inf_1, INPUT); 

  // Configurando os pinos do Fuso 2 como entradas ou saídas
  pinMode(clk_pos_2, OUTPUT); 
  pinMode(cw_pos_2, OUTPUT); 
  pinMode(en_pos_2, OUTPUT);

  pinMode(ligar_motor_2, OUTPUT);
  digitalWrite(ligar_motor_2, HIGH);

  pinMode(fdc_sup_2, INPUT); 
  pinMode(fdc_inf_2, INPUT);

  pinMode(TERMAL_PWM, OUTPUT);
  analogWriteResolution(16);
  analogWriteFrequency(200);
  analogWrite(TERMAL_PWM, Output);

  nexInit();
  tmp117.begin(0x48);
  tmp117_monitor.begin(0x49);

    //Attachs da tela de configuração de experimento
        habilita_seringa1.attachPop(habilita_seringa1PopCallback, &habilita_seringa1);
        referencia_seringa1.attachPop(referencia_seringa1PopCallback, &referencia_seringa1);
        sobe1.attachPop(sobe1PopCallback, &sobe1);
        desce1.attachPop(desce1PopCallback, &desce1);
        habilita_seringa2.attachPop(habilita_seringa2PopCallback, &habilita_seringa2);
        referencia_seringa2.attachPop(referencia_seringa2PopCallback, &referencia_seringa2);
        sobe2.attachPop(sobe2PopCallback, &sobe2);
        desce2.attachPop(desce2PopCallback, &desce2);

    //ligar_auto.attachPop(ligar_autoPopCallback, &ligar_auto);
    //Attachs da tela de controle manual
        
        incremento1.attachPop(incremento1PopCallback, &incremento1);
        decremento1.attachPop(decremento1PopCallback, &decremento1);
        incremento2.attachPop(incremento2PopCallback, &incremento2);
        decremento2.attachPop(decremento2PopCallback, &decremento2);
        play.attachPop(playPopCallback, &play);
        pause.attachPop(pausePopCallback, &pause);


  //ligando os SpinTimers
  new SpinTimer(MOTOR_MILLIS, new MotorAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  new SpinTimer(DISPLAY_MILLIS, new DisplayAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  new SpinTimer(CONTROL_MILLIS, new ControlAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);

  digitalWrite(ligar_motor_1, LOW);
  digitalWrite(ligar_motor_2, LOW);
}

void loop() {
  scheduleTimers();
}

//Callbacks da tela de configuração de experimento
void habilita_seringa1PopCallback(void *ptr) {
        if(seringa1_hab == 0){
          seringa1_hab = 1;
          habilita_seringa1.Set_background_image_pic(5);
          habilitado_1.Set_background_image_pic(18);
        }
         
        else {
            habilita_seringa1.Set_background_image_pic(4);
            habilitado_1.Set_background_image_pic(17);
            seringa1_hab = 0;
        }

    }

    void referencia_seringa1PopCallback(void *ptr) {
        // Implementação para o callback referencia_seringa1PopCallback
        referenciado_1 = 0;
        if (seringa1_hab){
        while (referenciado_1 == 0){
            while (digitalRead(fdc_sup_1) == HIGH) {
            //Serial.println("Entrou no while"); - chegou aqui ok
            digitalWrite(cw_pos_1, LOW);//cw_pos_1 = LOW -> Motores sobem
            digitalWrite(ligar_motor_1, HIGH);
            
            digitalWrite(en_pos_1, LOW);
            //Serial.println("move motor"); chegou ok mas não está movendo o motor
            digitalWrite(clk_pos_1, LOW);
            delayMicroseconds(300);
            digitalWrite(clk_pos_1, HIGH);
            delayMicroseconds(300);
        
        }

            digitalWrite(cw_pos_1, HIGH);
            for (int i = 0; i <= 12800; i++){
            digitalWrite(clk_pos_1, LOW);
            delayMicroseconds(300);
            digitalWrite(clk_pos_1, HIGH);
            delayMicroseconds(300);
            }
            
            digitalWrite(ligar_motor_1, LOW);
            digitalWrite(en_pos_1, HIGH);
            referenciado_1 = 1;
            }
        }
    }



void sobe1PopCallback(void *ptr) {
    // Implementação para o callback sobe1PopCallback
}

void desce1PopCallback(void *ptr) {
    // Implementação para o callback desce1PopCallback
}

void habilita_seringa2PopCallback(void *ptr) {
    // Implementação para o callback habilita_seringa2PopCallback
    if(seringa2_hab == 0 ){
        habilita_seringa2.Set_background_image_pic(5);
        seringa2_hab = 1;
    }

    else{
        
        habilita_seringa2.Set_background_image_pic(4);
        seringa2_hab = 0;
    }
}

void referencia_seringa2PopCallback(void *ptr) {
    // Implementação para o callback referencia_seringa2PopCallback
    referenciado_2 = 0;
        if (seringa2_hab){
            while (referenciado_2 == 0){
                while (digitalRead(fdc_sup_2) == HIGH) {
                    digitalWrite(cw_pos_2, LOW);
                    digitalWrite(ligar_motor_2, HIGH);
                    
                    digitalWrite(en_pos_2, LOW);
                    
                    digitalWrite(clk_pos_2, LOW);
                    delayMicroseconds(300);
                    digitalWrite(clk_pos_2, HIGH);
                    delayMicroseconds(300);
                }

                digitalWrite(cw_pos_2, HIGH);
                for (int i = 0; i <= 12800; i++){
                    digitalWrite(clk_pos_2, LOW);
                    delayMicroseconds(300);
                    digitalWrite(clk_pos_2, HIGH);
                    delayMicroseconds(300);
                }
                
                digitalWrite(ligar_motor_2, LOW);
                digitalWrite(en_pos_2, HIGH);
                referenciado_2 = 1;
        }
    }
}

void sobe2PopCallback(void *ptr) {
    // Implementação para o callback sobe2PopCallback
}

void desce2PopCallback(void *ptr) {
    // Implementação para o callback desce2PopCallback
}

//Calbacks tela 2
void playPopCallback(void *ptr) {
    //inicia a operação automática do experimento
    AUTO = 1;
}

void pausePopCallback(void *ptr) {
    //pausa a operação automática do experimento
    AUTO = 0;
}

void incremento1PopCallback(void *ptr) {
    if (seringa1_hab) {
        vazao_1++;
        
        if (vazao_1 > 100) {
            vazao_1 = 100;
            }
        
        dtostrf(vazao_1, 2, 0, buffer);
        const char* valor_vazao1 = buffer;
        vazao1_text.setText(valor_vazao1);
        }
}

void decremento1PopCallback(void *ptr) {
    if (seringa1_hab) {
        vazao_1--;
        
        if (vazao_1 < 0) {
            vazao_1 = 0;
            }
        
        dtostrf(vazao_1, 2, 0, buffer);
        const char* valor_vazao1 = buffer;
        vazao1_text.setText(valor_vazao1);
        }
}

void incremento2PopCallback(void *ptr) {
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

void decremento2PopCallback(void *ptr) {
    if (seringa2_hab) {
        vazao_2--;
        
        if (vazao_2 < 0) {
            vazao_2 = 0;
        }
        
        dtostrf(vazao_2, 2, 0, buffer);
        const char* valor_vazao2 = buffer;
        vazao2_text.setText(valor_vazao2);
    }
}