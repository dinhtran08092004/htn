#include "motor_control.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "MOTOR_TASK";

#define SERVO_MIN_WIDTH_US 350.0f
#define SERVO_MAX_WIDTH_US 3550.0f
#define SERVO_MAX_ANGLE    180.0f
#define SERVO_FREQ         50

static uint32_t g_full_duty = 0;    
QueueHandle_t motor_queue = NULL;

static uint32_t calculate_duty(float angle) {
    float angle_us = angle / SERVO_MAX_ANGLE * (SERVO_MAX_WIDTH_US - SERVO_MIN_WIDTH_US) + SERVO_MIN_WIDTH_US;
    return (uint32_t)((float)g_full_duty * angle_us * SERVO_FREQ / 1000000.0f);
}

void motor_task(void *pvParameters) {
    motor_msg_t msg;
    ESP_LOGI(TAG, "Motor Task chạy. Sẽ tự động ngắt PWM nếu LDR không có tín hiệu.");
    
    while (1) {
        // Đợi lệnh từ LDR trong tối đa 500ms (Nửa giây)
        if (xQueueReceive(motor_queue, &msg, pdMS_TO_TICKS(500)) == pdTRUE) {
            // CÓ TÍN HIỆU -> Cấp xung PWM để quay servo
            ledc_channel_t chan = (msg.axis == 'X') ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;
            uint32_t duty = calculate_duty((float)msg.angle);
            
            ledc_set_duty(LEDC_MODE, chan, duty);
            ledc_update_duty(LEDC_MODE, chan);
        } else {
            // HẾT 500ms KHÔNG CÓ TÍN HIỆU -> LDR đã cân bằng hoặc bị che tối
            // Giải pháp: Ghi Duty = 0 để ngắt hoàn toàn PWM -> Servo thả lỏng, hết rung giật
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
            
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);
        }
    }
}

esp_err_t motor_init(void) {
    motor_queue = xQueueCreate(10, sizeof(motor_msg_t));
    
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES, .freq_hz = SERVO_FREQ, .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t x_ch = {
        .gpio_num = SERVO_X_PIN, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER, .duty = 0, .speed_mode = LEDC_MODE
    };
    ledc_channel_config(&x_ch);

    ledc_channel_config_t y_ch = {
        .gpio_num = SERVO_Y_PIN, .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER, .duty = 0, .speed_mode = LEDC_MODE
    };
    ledc_channel_config(&y_ch);
    
    g_full_duty = (1 << LEDC_DUTY_RES) - 1;

    // Lúc vừa cấp nguồn: KHÔNG xuất xung PWM ngay để tránh giật. 
    // Hệ thống sẽ nằm im chờ đến khi LDR có tín hiệu ánh sáng.

    xTaskCreate(motor_task, "Motor_Task", 2048, NULL, 6, NULL);
    return ESP_OK;
}

void servo_all_stop(void) {}
