/*
 *  ISR.h - Interrupt Service Routine's for Flight REX_B
 */

const int GOPRO_1_PWR = 22;
const int GOPRO_2_PWR = 24;
const int GOPRO_LED_PIN = 26;
const int FRONT_MOTOR_PWR = 28;
const int REAR_MOTOR_PWR = 30;


void goProTriggerISR();
void motorTriggerISR();
void releaseTriggerISR();
