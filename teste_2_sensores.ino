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

Adafruit_TMP117  tmp117_control;
Adafruit_TMP117 tmp117_monitor;
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit TMP117 test!");

  // Try to initialize!
  if (!tmp117_monitor.begin(0x49)) {
    Serial.println("Failed to find TMP117 MONITOR chip");
    while (1) { delay(10); }
  }
  Serial.println("TMP117 MONITOR Found!");

  // Try to initialize!
  if (!tmp117_control.begin(0x48)) {
    Serial.println("Failed to find TMP117 CONTROL chip");
    while (1) { delay(10); }
  }
  Serial.println("TMP117 CONTROL Found!");

}
void loop() {

  sensors_event_t temp_monitor; // create an empty event to be filled
  sensors_event_t temp_control;
  tmp117_monitor.getEvent(&temp_monitor); //fill the empty event object with the current measurements
  tmp117_control.getEvent(&temp_control);
  Serial.print("Temperature monitor  "); Serial.print(temp_monitor.temperature);Serial.println(" degrees C");
  Serial.print("Temperature control  "); Serial.print(temp_control.temperature);Serial.println(" degrees C");
  Serial.println("");

  delay(1000);
}
