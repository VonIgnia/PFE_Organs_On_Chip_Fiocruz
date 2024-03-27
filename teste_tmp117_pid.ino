/**
 * @file basic_test.ino
 * @author Bryan Siepert for Adafruit Industries
 * @brief Shows how to specify a
 * @date 2020-11-10
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#include <Wire.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

#define PWM_RESOLUTION 65535

const int TERMAL_PWM = PB3;

double Setpoint = 37.0;
double Input;
double Output;
int Saida;

//double K = PWM_RESOLUTION;
//double Ti = Setpoint;
//double Td = 0;
//double Kp = 0.6 * K, Ki = Ti * 0.5, Kd = 0.125 * Td;
double Kp = 0.45*PWM_RESOLUTION;
//double Ki = 0.005*PWM_RESOLUTION;
double Ki = 0;
//double Kd = 0.003*PWM_RESOLUTION;
double Kd = 0;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Adafruit_TMP117  tmp117;
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  //Serial.println("Adafruit TMP117 test!");

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

  myPID.SetOutputLimits(0, PWM_RESOLUTION);

  myPID.SetMode(AUTOMATIC);

  analogWrite(TERMAL_PWM, PWM_RESOLUTION);

  

}
void loop() {
  float time = micros()/1e6;
  sensors_event_t temp; // create an empty event to be filled
  tmp117.getEvent(&temp); 
  Input = temp.temperature;
  myPID.Compute();
  Saida = PWM_RESOLUTION - Output;
  analogWrite(TERMAL_PWM, Saida);

  //double erro = Setpoint-Input;

  //fill the empty event object with the current measurements
  Serial.print(time);
  Serial.print(", ");
  Serial.println(temp.temperature);

  delay(1000);
}
