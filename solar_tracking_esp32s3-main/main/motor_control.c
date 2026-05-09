/*
========================================================
FILE: motor_control.c
Chức năng:
- Điều khiển 2 servo trục X/Y
- Nhận lệnh quay từ Queue
- Tạo PWM bằng LEDC của ESP32
- Tự ngắt PWM khi không có tín hiệu
========================================================
*/


// =====================================================
// INCLUDE THƯ VIỆN
// =====================================================

// Include file header của motor
#include "motor_control.h"

// Include thư viện log ESP32
#include "esp_log.h"

// Include thư viện task FreeRTOS
#include "freertos/task.h"


// =====================================================
// TAG DEBUG
// =====================================================

// TAG dùng khi in log
static const char *TAG = "MOTOR_TASK";


// =====================================================
// THÔNG SỐ SERVO
// =====================================================

// Độ rộng xung nhỏ nhất của servo
// ~ tương ứng góc 0°
#define SERVO_MIN_WIDTH_US 350.0f

// Độ rộng xung lớn nhất của servo
// ~ tương ứng góc 180°
#define SERVO_MAX_WIDTH_US 3550.0f

// Góc tối đa servo
#define SERVO_MAX_ANGLE 180.0f

// Tần số PWM servo
// Servo tiêu chuẩn dùng 50Hz
#define SERVO_FREQ 50


// =====================================================
// BIẾN TOÀN CỤC
// =====================================================

// Lưu duty PWM lớn nhất
// Ví dụ:
// 10-bit -> 1023
static uint32_t g_full_duty = 0;


// Queue dùng nhận lệnh điều khiển servo
QueueHandle_t motor_queue = NULL;


// =====================================================
// HÀM CHUYỂN GÓC -> DUTY PWM
// =====================================================

// Hàm static:
// chỉ dùng trong file này
static uint32_t calculate_duty(float angle)
{
    /*
    ====================================================
    TÍNH ĐỘ RỘNG XUNG PWM
    ====================================================

    Servo dùng PWM:

    0°    -> 350us
    180°  -> 3550us

    Công thức nội suy tuyến tính
    */

    float angle_us =
        angle / SERVO_MAX_ANGLE *
        (SERVO_MAX_WIDTH_US - SERVO_MIN_WIDTH_US)
        + SERVO_MIN_WIDTH_US;

    /*
    ====================================================
    CHUYỂN ĐỘ RỘNG XUNG -> DUTY PWM
    ====================================================

    Duty = duty_max × pulse_width × freq

    /1000000 vì us -> s
    */

    return (uint32_t)(
        (float)g_full_duty *
        angle_us *
        SERVO_FREQ /
        1000000.0f
    );
}


// =====================================================
// TASK ĐIỀU KHIỂN MOTOR
// =====================================================

void motor_task(void *pvParameters)
{
    // Biến nhận dữ liệu từ Queue
    motor_msg_t msg;

    // In log báo task bắt đầu
    ESP_LOGI(
        TAG,
        "Motor Task chạy. "
        "Sẽ tự động ngắt PWM nếu LDR không có tín hiệu."
    );

    // Loop vô hạn
    while (1)
    {
        /*
        =================================================
        CHỜ DỮ LIỆU TỪ QUEUE
        =================================================

        Timeout:
        500ms

        Nếu có dữ liệu:
        -> servo quay

        Nếu hết timeout:
        -> tắt PWM
        */

        if (
            xQueueReceive(
                motor_queue,            // Queue cần đọc
                &msg,                   // Biến nhận dữ liệu
                pdMS_TO_TICKS(500)      // Timeout 500ms
            )
            == pdTRUE
        )
        {
            /*
            =============================================
            CÓ TÍN HIỆU TỪ LDR
            =============================================
            */

            // Chọn channel PWM

            // Nếu axis = X
            // -> dùng channel 0

            // Nếu axis = Y
            // -> dùng channel 1

            ledc_channel_t chan =
                (msg.axis == 'X')
                ? LEDC_CHANNEL_0
                : LEDC_CHANNEL_1;

            /*
            =============================================
            TÍNH DUTY PWM TỪ GÓC
            =============================================
            */

            uint32_t duty =
                calculate_duty((float)msg.angle);

            /*
            =============================================
            GHI DUTY PWM
            =============================================
            */

            // Ghi duty vào LEDC
            ledc_set_duty(
                LEDC_MODE,
                chan,
                duty
            );

            // Cập nhật PWM ra GPIO
            ledc_update_duty(
                LEDC_MODE,
                chan
            );
        }
        else
        {
            /*
            =============================================
            KHÔNG CÓ TÍN HIỆU TRONG 500ms
            =============================================

            Điều này nghĩa là:

            - LDR đã cân bằng
            HOẶC
            - trời tối
            HOẶC
            - không cần quay nữa

            Giải pháp:
            Tắt PWM hoàn toàn
            */

            /*
            =============================================
            TẮT PWM SERVO X
            =============================================
            */

            ledc_set_duty(
                LEDC_MODE,
                LEDC_CHANNEL_0,
                0
            );

            ledc_update_duty(
                LEDC_MODE,
                LEDC_CHANNEL_0
            );

            /*
            =============================================
            TẮT PWM SERVO Y
            =============================================
            */

            ledc_set_duty(
                LEDC_MODE,
                LEDC_CHANNEL_1,
                0
            );

            ledc_update_duty(
                LEDC_MODE,
                LEDC_CHANNEL_1
            );

            /*
            =============================================
            KẾT QUẢ
            =============================================

            Servo:
            - hết rung giật
            - giảm nóng
            - giảm tiêu thụ điện
            - kéo dài tuổi thọ
            */
        }
    }
}


// =====================================================
// HÀM KHỞI TẠO MOTOR
// =====================================================

esp_err_t motor_init(void)
{
    /*
    ====================================================
    TẠO QUEUE
    ====================================================

    Queue:
    - chứa tối đa 10 message
    - mỗi message có kiểu motor_msg_t
    */

    motor_queue =
        xQueueCreate(
            10,
            sizeof(motor_msg_t)
        );



    /*
    ====================================================
    CẤU HÌNH TIMER PWM
    ====================================================
    */

    ledc_timer_config_t timer =
    {
        // Chế độ PWM
        .speed_mode = LEDC_MODE,

        // Timer số 0
        .timer_num = LEDC_TIMER,

        // Độ phân giải PWM
        .duty_resolution = LEDC_DUTY_RES,

        // Tần số PWM
        .freq_hz = SERVO_FREQ,

        // Clock tự động
        .clk_cfg = LEDC_AUTO_CLK
    };

    // Apply cấu hình timer
    ledc_timer_config(&timer);



    /*
    ====================================================
    CẤU HÌNH SERVO X
    ====================================================
    */

    ledc_channel_config_t x_ch =
    {
        // GPIO servo X
        .gpio_num = SERVO_X_PIN,

        // PWM channel 0
        .channel = LEDC_CHANNEL_0,

        // Dùng timer số 0
        .timer_sel = LEDC_TIMER,

        // Duty ban đầu = 0
        .duty = 0,

        // PWM low speed mode
        .speed_mode = LEDC_MODE
    };

    // Apply config servo X
    ledc_channel_config(&x_ch);



    /*
    ====================================================
    CẤU HÌNH SERVO Y
    ====================================================
    */

    ledc_channel_config_t y_ch =
    {
        // GPIO servo Y
        .gpio_num = SERVO_Y_PIN,

        // PWM channel 1
        .channel = LEDC_CHANNEL_1,

        // Dùng timer số 0
        .timer_sel = LEDC_TIMER,

        // Duty ban đầu = 0
        .duty = 0,

        // PWM low speed mode
        .speed_mode = LEDC_MODE
    };

    // Apply config servo Y
    ledc_channel_config(&y_ch);



    /*
    ====================================================
    TÍNH DUTY MAX
    ====================================================

    Ví dụ:
    10-bit

    2^10 - 1
    = 1023
    */

    g_full_duty =
        (1 << LEDC_DUTY_RES) - 1;



    /*
    ====================================================
    KHỞI ĐỘNG:
    ====================================================

    KHÔNG xuất PWM ngay.

    Mục đích:
    - tránh servo giật lúc cấp nguồn
    - tránh quay lung tung
    - giảm dòng khởi động
    */



    /*
    ====================================================
    TẠO TASK MOTOR
    ====================================================
    */

    xTaskCreate(
        motor_task,       // Hàm task
        "Motor_Task",     // Tên task
        2048,             // Stack size
        NULL,             // Tham số
        6,                // Priority
        NULL              // Không cần handle
    );



    /*
    ====================================================
    TRẢ VỀ THÀNH CÔNG
    ====================================================
    */

    return ESP_OK;
}


// =====================================================
// HÀM DỪNG TOÀN BỘ SERVO
// =====================================================

void servo_all_stop(void)
{
    /*
    Hàm hiện đang để trống.

    Sau này có thể bổ sung:
    - disable PWM
    - reset góc servo
    - stop timer
    - deinit LEDC
    */
}
