// radioComm.h
#ifndef RADIOCOMM_H
#define RADIOCOMM_H

#include <Arduino.h>

// ================================================================
// KHAI BÁO EXTERN CHO CÁC PIN
// (Để file radio.cpp hiểu được các biến ch1Pin... đang nằm ở main.cpp)
// ================================================================
extern const int ch1Pin;
extern const int ch2Pin;
extern const int ch3Pin;
extern const int ch4Pin;
extern const int ch5Pin;
extern const int ch6Pin;
extern const int PPM_Pin;

// ================================================================
// KHAI BÁO HÀM (FUNCTION PROTOTYPES)
// ================================================================

// Hàm cài đặt cho chế độ PPM
void readPPM_setup(int pin);

// Hàm cài đặt cho chế độ PWM
void readPWM_setup(int ch1, int ch2, int ch3, int ch4, int ch5, int ch6);

// Hàm lấy giá trị tín hiệu (trả về độ rộng xung)
unsigned long getRadioPWM(int ch_num);

#endif