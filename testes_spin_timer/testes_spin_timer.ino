#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>
#include <SpinTimer.h>

#define PWM_RESOLUTION 65535

//HardwareSerial Serial1(PC11,PC10); //(RX, TX)Portas RXTX da núcleo onde o Display está conectado
HardwareSerial Serial1(PA10,PA9); //(RX, TX)Portas RXTX da núcleo onde o Display está conectado
#include "Nextion.h"
bool Heating = 0;
int ConMan = 2;

NexButton SWITCH1 = NexButton(ConMan,5,"b4");
NexText TEMP1     = NexText  (ConMan, 7, "t1");

NexTouch *nex_listen_list[] = {
    &SWITCH1, //SWITCH1
    NULL
  };
;









const unsigned int TEMP_UPDATE_MILLIS = 1500;
const unsigned int  CONTROL_MILLIS = 1000;
const unsigned int DISPLAY_MILLIS = 10;

const int TERMAL_PWM = PB3;

double Setpoint = 37.0;
double Input;
double Output;
int Saida;
//int Saida_teste;
//double K = PWM_RESOLUTION;
//double Ti = Setpoint;
//double Td = 0;
//double Kp = 0.6 * K, Ki = Ti * 0.5, Kd = 0.125 * Td;
double Kp = 0.19*PWM_RESOLUTION;
//double Ki = 0.02*PWM_RESOLUTION;
double Ki = 0*PWM_RESOLUTION;
//double Kd = 0.1*PWM_RESOLUTION;
double Kd = 0*PWM_RESOLUTION;

//double Ki_escalonado = 0*PWM_RESOLUTION;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
Adafruit_TMP117  tmp117;

class ControlAction : public SpinTimerAction
{
public:
  void timeExpired()
  {
    if(Heating){
      float time = millis()/1e3;
      sensors_event_t temp; // create an empty event to be filled
      tmp117.getEvent(&temp); 
      Input = temp.temperature;
      myPID.Compute();
      Saida = PWM_RESOLUTION - Output;
      
      analogWrite(TERMAL_PWM, Saida);

      //fill the empty event object with the current measurements
      Serial.print(time);
      Serial.print(", ");
      Serial.print(Output);
      Serial.print(", ");
      Serial.println(temp.temperature);
    }
    else{
      analogWrite(TERMAL_PWM, PWM_RESOLUTION);
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

class TempUpdateAction : public SpinTimerAction
{
public:
  void timeExpired()
  {
    char buffer[10];
    sensors_event_t temp; // create an empty event to be filled
    tmp117.getEvent(&temp);
    double temperatura = temp.temperature;
    dtostrf(temperatura,4,2,buffer);
    const char* Value = buffer;
    TEMP1.setText(Value);
  }
};









void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  //Serial.println("Adafruit TMP117 test!");
  nexInit();
  SWITCH1.attachPop(SWITCH1PopCallback, &SWITCH1);
  //pinMode(luz, OUTPUT);
  // Try to initialize!
  if (!tmp117.begin()) {
    //Serial.println("Failed to find TMP117 chip");
    while (1) { delay(10); }
  }
  //Serial.println("TMP117 Found!");

  pinMode(TERMAL_PWM, OUTPUT);
  analogWriteResolution(16);
  analogWriteFrequency(200);
  analogWrite(TERMAL_PWM, PWM_RESOLUTION);
  //double Saida_teste = 0.80*PWM_RESOLUTION;
  myPID.SetOutputLimits(0, PWM_RESOLUTION);
  myPID.SetSampleTime(1000);

  myPID.SetMode(AUTOMATIC);

  new SpinTimer(CONTROL_MILLIS, new ControlAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  new SpinTimer(DISPLAY_MILLIS, new DisplayAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  new SpinTimer(TEMP_UPDATE_MILLIS, new TempUpdateAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  

  

}
void loop() {

  scheduleTimers();
  
  
}


void SWITCH1PopCallback(void *ptr){
  
  if (Heating == false){
    
    
    SWITCH1.Set_background_image_pic(4);
    Heating = true;
  }

  else{
    
    
    SWITCH1.Set_background_image_pic(3);
    Heating = false;
  }
}


