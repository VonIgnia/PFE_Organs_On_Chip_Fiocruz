#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>

#include <SpinTimer.h>

#define PWM_RESOLUTION 65535


bool Heating = 0;
Adafruit_TMP117  tmp117;

const unsigned int  CONTROL_MILLIS = 1000;


const int TERMAL_PWM = PB3;

int Saida_teste;

unsigned long tempo;

float a = -1.0/120000.0;

float h = 1200;

float k = 37;

float Setpoint = 25.0;

float start;

double Output = 0;

int contador = 0;


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
    Serial.print(Setpoint);
    Serial.print(", ");
    Serial.print(temp.temperature);
    Serial.print(", ");
    Serial.println(Output);
    }


  }
};













void setup(void) {
  Serial.begin(115200);
 
  tmp117.begin(0x48) ;

  pinMode(TERMAL_PWM, OUTPUT);
  analogWriteResolution(16);
  analogWriteFrequency(200);
  analogWrite(TERMAL_PWM, Output);
  

  new SpinTimer(CONTROL_MILLIS, new ControlAction(), SpinTimer::IS_RECURRING, SpinTimer::IS_AUTOSTART);
  

  

}
void loop() {

  scheduleTimers();
  
}


