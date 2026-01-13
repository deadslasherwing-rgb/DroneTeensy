//Arduino/Teensy Flight Controller - dRehmFlight
//File: radioComm.cpp
//Cleaned by: Gemini for PPM Only

#include "radioComm.h"
#include <Arduino.h>

// ===================================================================================
// 1. BIẾN TOÀN CỤC
// ===================================================================================
// Biến lưu giá trị kênh đọc được (Volatile là bắt buộc cho ngắt)
volatile unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;

// Biến hỗ trợ tính toán thời gian PPM
volatile unsigned long last_ppm_time = 0;
volatile int ppm_counter = 0;

// Khai báo trước hàm ngắt để dùng trong setup
void getPPM(); 

// ===================================================================================
// 2. HÀM SETUP (Chỉ giữ lại PPM)
// ===================================================================================

void readPPM_setup(int pin) {
  pinMode(pin, INPUT_PULLUP);
  delay(20);
  
  // Gán ngắt vào chân PPM. 
  // RISING (Cạnh lên) là chuẩn phổ biến. Nếu không nhận tín hiệu, thử đổi thành FALLING.
  attachInterrupt(digitalPinToInterrupt(pin), getPPM, RISING);
}

// ===================================================================================
// 3. HÀM LẤY DỮ LIỆU AN TOÀN (GETTER)
// ===================================================================================

unsigned long getRadioPWM(int ch_num) {
  unsigned long returnPWM = 0;
  
  // Tắt ngắt tạm thời để copy giá trị, tránh bị lỗi dữ liệu khi đang đọc dở
  noInterrupts(); 
  switch(ch_num) {
    case 1: returnPWM = channel_1_raw; break;
    case 2: returnPWM = channel_2_raw; break;
    case 3: returnPWM = channel_3_raw; break;
    case 4: returnPWM = channel_4_raw; break;
    case 5: returnPWM = channel_5_raw; break;
    case 6: returnPWM = channel_6_raw; break;
  }
  interrupts(); // Bật lại ngắt ngay lập tức
  
  return returnPWM;
}

// ===================================================================================
// 4. HÀM XỬ LÝ NGẮT (ISR) - TRÁI TIM CỦA PPM
// ===================================================================================

void getPPM() {
  unsigned long current_time = micros();
  unsigned long dt_ppm = current_time - last_ppm_time;
  last_ppm_time = current_time;

  // 1. Tìm Sync Pulse (Khoảng nghỉ giữa các gói tin > 2.5ms)
  if (dt_ppm > 2500) { 
    ppm_counter = 0; // Reset đếm kênh để bắt đầu gói mới
    return;
  }

  // 2. Đọc độ rộng xung kênh (Chỉ nhận giá trị hợp lệ 700us - 2200us)
  if (dt_ppm > 700 && dt_ppm < 2200) {
    ppm_counter++; 

    // 3. Gán vào kênh tương ứng (Mapping AETR phổ biến)
    // Nếu tay cầm bị sai kênh, bạn tráo đổi vị trí các biến channel_x_raw ở đây
    // 3. Gán vào biến (MAPPING LẠI THEO TAY CẦM CỦA BẠN)
    // Cấu hình hiện tại của bạn: CH1=Roll, CH2=Pitch, CH3=Throttle, CH4=Yaw

    if (ppm_counter == 1) channel_2_raw = dt_ppm;      // Kênh 1 là ROLL -> Gán vào biến channel_2 (Biến Roll)
    else if (ppm_counter == 2) channel_3_raw = dt_ppm; // Kênh 2 là PITCH -> Gán vào biến channel_3 (Biến Pitch)
    else if (ppm_counter == 3) channel_1_raw = dt_ppm; // Kênh 3 là THROTTLE -> Gán vào biến channel_1 (Biến Throttle)
    else if (ppm_counter == 4) channel_4_raw = dt_ppm; // Kênh 4 là YAW -> Gán vào biến channel_4 (Biến Yaw)
    
    // Các kênh phụ (AUX)
    else if (ppm_counter == 5) channel_5_raw = dt_ppm; // AUX1
    else if (ppm_counter == 6) channel_6_raw = dt_ppm; // AUX2
  }
}