// ==========================
// FILE: motor_control.h
// ==========================

// Nếu MOTOR_CONTROL_H chưa được định nghĩa
#ifndef MOTOR_CONTROL_H

// Định nghĩa MOTOR_CONTROL_H
// để tránh include nhiều lần
#define MOTOR_CONTROL_H

// ==========================
// Include thư viện PWM LEDC
// ==========================
#include "driver/ledc.h"

// Thư viện kiểu lỗi esp_err_t
#include "esp_err.h"

// FreeRTOS core
#include "freertos/FreeRTOS.h"

// Queue FreeRTOS
#include "freertos/queue.h"

// ==========================
// KHAI BÁO GPIO SERVO
// ==========================

// Servo trục X nối GPIO17
#define SERVO_X_PIN 17

// Servo trục Y nối GPIO21
#define SERVO_Y_PIN 21

// ==========================
// CẤU HÌNH PWM LEDC
// ==========================

// Sử dụng timer số 0 của LEDC
#define LEDC_TIMER LEDC_TIMER_0

// Chế độ PWM tốc độ thấp
#define LEDC_MODE LEDC_LOW_SPEED_MODE

// Độ phân giải PWM 10 bit
// Duty từ 0 -> 1023
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT

// ==========================
// STRUCT DỮ LIỆU GỬI QUA QUEUE
// ==========================

// Định nghĩa kiểu dữ liệu motor_msg_t
typedef struct
{
    // Trục cần quay
    // 'X' hoặc 'Y'
    char axis;

    // Góc servo cần quay tới
    int angle;

} motor_msg_t;

// ==========================
// QUEUE TOÀN CỤC
// ==========================

// Queue dùng gửi lệnh servo
extern QueueHandle_t motor_queue;

// ==========================
// PROTOTYPE HÀM
// ==========================

// Hàm khởi tạo servo
esp_err_t motor_init(void);

// Hàm dừng toàn bộ servo
void servo_all_stop(void);

// Kết thúc header guard
#endif




// ==========================
// FILE: motor_control.c
// ==========================

// Include file header chính
#include "motor_control.h"

// Include log ESP32
#include "esp_log.h"

// Include task FreeRTOS
#include "freertos/task.h"

// ==========================
// TAG LOG DEBUG
// ==========================

// Tên tag hiển thị log
static const char *TAG = "MOTOR_TASK";

// ==========================
// THÔNG SỐ SERVO
// ==========================

// Độ rộng xung nhỏ nhất
// tương ứng góc 0°
#define SERVO_MIN_WIDTH_US 350.0f

// Độ rộng xung lớn nhất
// tương ứng góc 180°
#define SERVO_MAX_WIDTH_US 3550.0f

// Góc quay tối đa của servo
#define SERVO_MAX_ANGLE 180.0f

// Tần số PWM servo
// Servo chuẩn dùng 50Hz
#define SERVO_FREQ 50

// ==========================
// BIẾN TOÀN CỤC
// ==========================

// Giá trị duty max
// Ví dụ 10 bit = 1023
static uint32_t g_full_duty = 0;

// Queue chứa lệnh điều khiển servo
QueueHandle_t motor_queue = NULL;

// ==========================
// HÀM TÍNH DUTY PWM
// ==========================

// Hàm chuyển góc servo -> duty PWM
static uint32_t calculate_duty(float angle)
{
    // ====================================================
    // TÍNH ĐỘ RỘNG XUNG PWM
    // ====================================================

    // Nội suy tuyến tính:
    // 0°   -> 350us
    // 180° -> 3550us

    float angle_us =
        angle / SERVO_MAX_ANGLE *
        (SERVO_MAX_WIDTH_US - SERVO_MIN_WIDTH_US)
        + SERVO_MIN_WIDTH_US;

    // ====================================================
    // CHUYỂN ĐỘ RỘNG XUNG -> DUTY
    // ====================================================

    return (uint32_t)(
        (float)g_full_duty *
        angle_us *
        SERVO_FREQ /
        1000000.0f
    );
}

// ==========================
// TASK ĐIỀU KHIỂN SERVO
// ==========================

void motor_task(void *pvParameters)
{
    // Biến chứa dữ liệu queue nhận được
    motor_msg_t msg;

    // In log
    ESP_LOGI(TAG,
             "Motor Task chạy. "
             "Sẽ tự động ngắt PWM nếu LDR không có tín hiệu.");

    // Loop vô hạn
    while (1)
    {
        // ====================================================
        // CHỜ LỆNH TỪ QUEUE
        // ====================================================

        // Chờ tối đa 500ms
        if (xQueueReceive(
                motor_queue,            // Queue cần đọc
                &msg,                   // Biến nhận dữ liệu
                pdMS_TO_TICKS(500)      // Timeout
            ) == pdTRUE)
        {
            // ====================================================
            // CÓ LỆNH ĐIỀU KHIỂN
            // ====================================================

            // Nếu axis = X
            // dùng channel 0
            //
            // Nếu axis = Y
            // dùng channel 1

            ledc_channel_t chan =
                (msg.axis == 'X')
                ? LEDC_CHANNEL_0
                : LEDC_CHANNEL_1;

            // Tính duty PWM từ góc
            uint32_t duty =
                calculate_duty((float)msg.angle);

            // Ghi duty PWM vào channel
            ledc_set_duty(
                LEDC_MODE,
                chan,
                duty
            );

            // Cập nhật PWM ra chân GPIO
            ledc_update_duty(
                LEDC_MODE,
                chan
            );
        }
        else
        {
            // ====================================================
            // KHÔNG CÓ TÍN HIỆU TRONG 500ms
            // ====================================================

            // Điều này nghĩa là:
            // - LDR đã cân bằng
            // HOẶC
            // - trời tối

            // ====================================================
            // TẮT PWM SERVO X
            // ====================================================

            ledc_set_duty(
                LEDC_MODE,
                LEDC_CHANNEL_0,
                0
            );

            ledc_update_duty(
                LEDC_MODE,
                LEDC_CHANNEL_0
            );

            // ====================================================
            // TẮT PWM SERVO Y
            // ====================================================

            ledc_set_duty(
                LEDC_MODE,
                LEDC_CHANNEL_1,
                0
            );

            ledc_update_duty(
                LEDC_MODE,
                LEDC_CHANNEL_1
            );

            // ====================================================
            // KẾT QUẢ:
            // ====================================================

            // Servo không còn nhận xung PWM
            // -> thả lỏng
            // -> giảm rung giật
            // -> tiết kiệm điện
        }
    }
}

// ==========================
// HÀM KHỞI TẠO MOTOR
// ==========================

esp_err_t motor_init(void)
{
    // ====================================================
    // TẠO QUEUE
    // ====================================================

    // Queue chứa tối đa 10 message
    motor_queue =
        xQueueCreate(
            10,
            sizeof(motor_msg_t)
        );

    // ====================================================
    // CẤU HÌNH TIMER PWM
    // ====================================================

    ledc_timer_config_t timer =
    {
        // Chế độ PWM
        .speed_mode = LEDC_MODE,

        // Timer số 0
        .timer_num = LEDC_TIMER,

        // Độ phân giải PWM
        .duty_resolution = LEDC_DUTY_RES,

        // Tần số PWM 50Hz
        .freq_hz = SERVO_FREQ,

        // Clock tự động
        .clk_cfg = LEDC_AUTO_CLK
    };

    // Apply config timer
    ledc_timer_config(&timer);

    // ====================================================
    // CẤU HÌNH SERVO X
    // ====================================================

    ledc_channel_config_t x_ch =
    {
        // GPIO servo X
        .gpio_num = SERVO_X_PIN,

        // PWM channel 0
        .channel = LEDC_CHANNEL_0,

        // Dùng timer 0
        .timer_sel = LEDC_TIMER,

        // Duty ban đầu = 0
        .duty = 0,

        // Chế độ PWM
        .speed_mode = LEDC_MODE
    };

    // Apply config servo X
    ledc_channel_config(&x_ch);

    // ====================================================
    // CẤU HÌNH SERVO Y
    // ====================================================

    ledc_channel_config_t y_ch =
    {
        // GPIO servo Y
        .gpio_num = SERVO_Y_PIN,

        // PWM channel 1
        .channel = LEDC_CHANNEL_1,

        // Dùng timer 0
        .timer_sel = LEDC_TIMER,

        // Duty ban đầu = 0
        .duty = 0,

        // Chế độ PWM
        .speed_mode = LEDC_MODE
    };

    // Apply config servo Y
    ledc_channel_config(&y_ch);

    // ====================================================
    // TÍNH DUTY MAX
    // ====================================================

    // Ví dụ:
    // 10 bit -> 1023
    g_full_duty =
        (1 << LEDC_DUTY_RES) - 1;

    // ====================================================
    // LÚC KHỞI ĐỘNG:
    // ====================================================

    // KHÔNG phát PWM ngay
    // để tránh servo giật lúc cấp nguồn

    // Servo sẽ chỉ quay khi:
    // - LDR phát hiện ánh sáng
    // - Queue gửi lệnh điều khiển

    // ====================================================
    // TẠO MOTOR TASK
    // ====================================================

    xTaskCreate(
        motor_task,       // Hàm task
        "Motor_Task",     // Tên task
        2048,             // Stack size
        NULL,             // Tham số truyền
        6,                // Priority
        NULL              // Không cần handle
    );

    // Trả về thành công
    return ESP_OK;
}

// ==========================
// HÀM DỪNG TOÀN BỘ SERVO
// ==========================

void servo_all_stop(void)
{
    // Hiện đang để trống
    // Có thể bổ sung:
    // - tắt PWM
    // - reset servo
    // - disable channel
}
