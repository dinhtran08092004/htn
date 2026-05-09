#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// --- Thay bằng chân GPIO thực tế của bạn ---
#define SERVO_X_PIN     17 
#define SERVO_Y_PIN     21 

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT

// 1. Khai báo cấu trúc tin nhắn sẽ gửi qua Queue
typedef struct {
    char axis;    // Trục cần quay ('X' hoặc 'Y')
    int angle;    // Góc mục tiêu (0 - 180)
} motor_msg_t;

// 2. Khai báo biến Queue để các file khác (ldr_logic.c) có thể gọi
extern QueueHandle_t motor_queue;

esp_err_t motor_init(void);
void servo_all_stop(void);

#endif // MOTOR_CONTROL_H