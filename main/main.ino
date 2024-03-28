#include <Wire.h> //Serial do Sensor

#include <Adafruit_Sensor.h>
#include <Adafruit_TMP117.h>
Adafruit_TMP117  tmp117;

//Variáveis Iniciais NEXTION
#include "Nextion.h"
  HardwareSerial Serial1(PA10,PA9); //Portas RXTX da núcleo onde o Display está conectado
  //Declarando os objetos Nextion -- Exemplo: [page id:0,component id:1, component name: "b0"]
  //Objetos na página de Controle manual (page id:1)
  int ConMan = 1;
  NexButton UP1     = NexButton(ConMan, 3, "b0");
  NexButton DOWN1   = NexButton(ConMan, 8, "b1");
  NexButton SWITCH1 = NexButton(ConMan,5,"b4");
  NexText TEMP1     = NexText  (ConMan, 7, "t1");

  //Registrar quais elementos da tela são interativos, passivos ao toque.
  NexTouch *nex_listen_list[] = {
      &UP1, //UP1
      &DOWN1, //DOWN1
      &SWITCH1, //SWITCH1
      NULL
  };
;

//Variáveis Iniciais Controle de Temperatura
  #define PWM_RESOLUTION 65535
  const int TERMAL_PWM = PB3;

  double Setpoint = 37.0;
  double Input;
  double Output;


//Ganhos do Controlador PID responsável pelo chaveamento da potênicia na PCB de resistência
  bool isHeating = false; //variável booleana que indica se aquecimento está ligado ou não
  #include <PID_v1.h>
  double Kp = 0.45*PWM_RESOLUTION;
  double Ki = 0;
  double Kd = 0;
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
;

//Tempos do Spintimer
#include <SpinTimer.h>
const unsigned int MILLIS_UPDATE = 1000;
const unsigned int MILLIS_PID = 100;

//SpinTimer Motores (a configurar - Andressa e Gabi)
  class Motores : public SpinTimerAction {
        public:
      void timeExpired() {
        if (){
          update_configMenu();
        } 
        else {
          update_menu();
        }
      }
  }

//SpinTimer Update Display (a configurar - Giorno e Lucca)
  class UpdateDisplay : public SpinTimerAction {
    public:
      void timeExpired() {
        if (){
          update_configMenu();
        } 
        else {
          update_menu();
        }
      }
  };

//SpinTimer PID
  class PID : public SpinTimerAction {
    public:
      void timeExpired() {
        if (){
          update_configMenu();
        } 
        else {
          update_menu();
        }
      }
  };

void setup(void) {
  //declarar Spintimers
  new SpinTimer(MILLIS_UPDATE, new  UpdateDisplay(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  new SpinTimer(MILLIS_PID   , new  PID()          , SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);

  // Inicializar sensores
  if (!tmp117.begin()) {
    while (1) { delay(10); }
  }

  // Abrir e aguardar por porta serial
  // Suspeito que para funcionar em paralelo com o Nextion será necessário trocar Serial por Serialdb (testar para confirmar)
  Serial.begin(115200);
  while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens
  
  //Inicializar Display, objetos e funções Nextion
  nexInit(); //Configura o baudrate de debug e comunicação com o Display Nextion

  /* Register the pop event callback function of the current button component. */
  SWITCH1.attachPop(SWITCH1PopCallback, &SWITCH1);
  //pinMode(luz, OUTPUT);
  //dbSerialPrintln("setup done"); 

  //inicialização do PWM
  pinMode(TERMAL_PWM, OUTPUT);
  analogWriteResolution(16);
  analogWriteFrequency(200);
  analogWrite(TERMAL_PWM, PWM_RESOLUTION);

  myPID.SetOutputLimits(0, PWM_RESOLUTION);
  myPID.SetMode(AUTOMATIC);

  analogWrite(TERMAL_PWM, PWM_RESOLUTION);



}

void loop() {

  scheduleTimers();
  nexLoop(nex_listen_list);
  
  float time = micros()/1e6;
  sensors_event_t temp; // create an empty event to be filled
  tmp117.getEvent(&temp); 
  Input = temp.temperature;
  myPID.Compute();
  Output = PWM_RESOLUTION - Output; //Alterei a sintaxe dessa linha para eliminar a variável "Saída"
  analogWrite(TERMAL_PWM, Output);


  //fill the empty event object with the current measurements
  Serial.print(time);
  Serial.print(", ");
  Serial.println(temp.temperature);

  

}

//





const int luz = PB3;



/*
 * Button component pop callback function. 
 * In this example,the button's text value will plus one every time when it is released. 
 */
void SWITCH1PopCallback(void *ptr){
  if (Heating == false){
    digitalWrite(luz, HIGH);
    TEMP1.setText("On");
    SWITCH1.Set_background_image_pic(4);
    Heating = true;
  }

  else{
    digitalWrite(luz, LOW);
    TEMP1.setText("Off");
    SWITCH1.Set_background_image_pic(3);
    Heating = false;
  }
}










