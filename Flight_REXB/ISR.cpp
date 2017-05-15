#include "Arduino.h"
#include "ISR.h"

void goProTriggerISR() {
  // gopro ISR turns on both gopro cameras and LED
  digitalWrite(GOPRO_1_PWR, HIGH);
  digitalWrite(GOPRO_2_PWR, HIGH);
  digitalWrite(GOPRO_LED_PIN, HIGH);
}

void motorTriggerISR() {
  digitalWrite(FRONT_MOTOR_PWR, HIGH);
}

void releaseTriggerISR() {
  digitalWrite(REAR_MOTOR_PWR, HIGH);
}

