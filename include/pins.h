#ifndef PINS_H
#define PINS_H

#define LED_PIN 25

// motor driver pins 
#define IN1_PIN 2
#define IN2_PIN 3
#define IN3_PIN 4
#define IN4_PIN 5

// I2C Bus 0 
// left encoder + IMU
#define ENC_LEFT_SDA_PIN 20
#define ENC_LEFT_SCL_PIN 21
#define IMU_SDA_PIN 20
#define IMU_SCL_PIN 21

// I2C Bus 1
// right encoder
#define ENC_RIGHT_SDA_PIN 26
#define ENC_RIGHT_SCL_PIN 27 

// I2C Device Adresses
#define ENC_LEFT_ADDR 0x06
#define ENC_RIGHT_ADDR 0x06
#define IMU_ADDR 0x68
// start switch 
#define START_BTN_PIN 6
#endif
