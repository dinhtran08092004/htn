#include <stdio.h>                  // Thư viện chuẩn C dùng cho printf, snprintf

#include "freertos/FreeRTOS.h"     // Thư viện lõi FreeRTOS
#include "freertos/task.h"         // Quản lý Task
#include "freertos/timers.h"       // Software Timer
#include "freertos/semphr.h"       // Semaphore và Mutex
#include "freertos/queue.h"        // Queue truyền dữ liệu giữa các task

#include "esp_log.h"               // Dùng để in log ESP_LOGI()
#include "esp_adc/adc_oneshot.h"   // Driver ADC chế độ One-shot

// ================== Include các module tự viết ==================

#include "i2c_lcd.h"               // Driver LCD I2C
#include "ntc_sensor.h"            // Xử lý cảm biến nhiệt độ NTC
#include "ldr_logic.h"             // Logic dò hướng ánh sáng
#include "motor_control.h"         // Điều khiển motor
#include "pump_system.h"           // Điều khiển bơm

// ================== Biến toàn cục từ file khác ==================

// extern = biến được khai báo ở file khác
extern int g_lcd_max_light;        // Giá trị ánh sáng lớn nhất để hiển thị LCD
extern bool g_lcd_is_tracking;     // Trạng thái tracking ON/OFF

// TAG dùng cho log debug
static const char *TAG = "MAIN_SYSTEM";

// ================== Macro cấu hình ==================

#define LIGHT_CHECK_MS 3000        // Kiểm tra ánh sáng mỗi 3000ms = 3 giây

// ================== Biến toàn cục hệ thống ==================

// Handle quản lý ADC Unit
static adc_oneshot_unit_handle_t g_adc_handle;

// Mutex bảo vệ ADC tránh nhiều task truy cập cùng lúc
SemaphoreHandle_t adc_mutex = NULL;

// Handle quản lý task tracking mặt trời
TaskHandle_t ldr_task_handle = NULL;

/**
 * ============================================================
 * TASK ĐỌC NHIỆT ĐỘ
 * ============================================================
 */
void ntc_task(void *pvParameters)
{
    // In log báo task đã khởi động
    ESP_LOGI(TAG, "NTC Task started.");

    // Vòng lặp vô hạn của task
    while (1)
    {
        // Đọc nhiệt độ từ NTC
        // Hàm này:
        // - đọc ADC
        // - tính nhiệt độ
        // - gửi queue
        // - kiểm tra quá nhiệt
        ntc_read_temp(g_adc_handle);

        // Delay 5 giây
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/**
 * ============================================================
 * TIMER CALLBACK KIỂM TRA ÁNH SÁNG
 * ============================================================
 */
static void light_check_callback(TimerHandle_t xTimer)
{
    // Biến lưu giá trị ADC đọc từ LDR
    int val = 0;

    // Xin quyền sử dụng ADC thông qua Mutex
    if (xSemaphoreTake(ntc_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        // Đọc giá trị ánh sáng từ LDR_XP_CH
        adc_oneshot_read(g_adc_handle, LDR_XP_CH, &val);

        // Trả quyền sử dụng ADC
        xSemaphoreGive(ntc_mutex);
    }

    // Nếu ánh sáng lớn hơn ngưỡng SUN_LIMIT
    if (val >= SUN_LIMIT)
    {
        // Nếu task tracking đang bị suspend
        if (eTaskGetState(ldr_task_handle) == eSuspended)
        {
            // In log
            ESP_LOGI(TAG,
                     "Troi sang (LDR=%d), khoi phuc Sun Tracking Task!",
                     val);

            // Resume lại task tracking
            vTaskResume(ldr_task_handle);
        }
    }
}

/**
 * ============================================================
 * TASK HIỂN THỊ LCD
 * ============================================================
 */
void lcd_task(void *pvParameters)
{
    // Khởi tạo I2C Master
    i2c_master_init();

    // Khởi tạo LCD
    lcd_init();

    // Xóa màn hình LCD
    lcd_clear();

    // Buffer chứa chuỗi hiển thị
    char buffer[17];

    // Biến nhiệt độ hiện tại
    float current_temp = 0.0f;

    // Loop vô hạn
    while (1)
    {
        // Đọc dữ liệu mới nhất từ Queue
        // Peek = đọc nhưng KHÔNG xóa queue
        if (xQueuePeek(ntc_queue, &current_temp, 0) != pdTRUE)
        {
            // Nếu queue chưa có dữ liệu
            // giữ nguyên current_temp
        }

        // ====================================================
        // DÒNG 1 LCD
        // ====================================================
        // Hiển thị:
        // T:xx.xC P:ON/OFF

        snprintf(buffer,
                 sizeof(buffer),
                 "T:%5.1fC P:%-3s",
                 current_temp,
                 (gpio_get_level(PUMP_PIN) ? "ON" : "OFF"));

        // Đưa con trỏ tới dòng 0 cột 0
        lcd_put_cur(0, 0);

        // Gửi chuỗi ra LCD
        lcd_send_string(buffer);

        // ====================================================
        // DÒNG 2 LCD
        // ====================================================
        // Hiển thị:
        // L:xxxx Trk:RUN/OFF

        snprintf(buffer,
                 sizeof(buffer),
                 "L:%-4d Trk:%s ",
                 g_lcd_max_light,
                 g_lcd_is_tracking ? "RUN" : "OFF");

        // Đưa con trỏ tới dòng 1 cột 0
        lcd_put_cur(1, 0);

        // Hiển thị chuỗi
        lcd_send_string(buffer);

        // Delay 1 giây
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * ============================================================
 * HÀM CHÍNH app_main()
 * ============================================================
 */
void app_main(void)
{
    // In log khởi động hệ thống
    ESP_LOGI(TAG,
             "=== KHOI DONG SOLAR TRACKER ESP32-S3 ===");

    // ========================================================
    // 1. KHỞI TẠO PHẦN CỨNG
    // ========================================================

    // Khởi tạo motor/PWM/GPIO
    motor_init();

    // ========================================================
    // 2. KHỞI TẠO RTOS OBJECT
    // ========================================================

    // Tạo Queue/Semaphore cho NTC
    ntc_init_rtos();

    // Tạo Queue/Semaphore cho Pump
    pump_init_rtos();

    // Tạo Mutex bảo vệ ADC
    adc_mutex = xSemaphoreCreateMutex();

    // DÒNG NÀY BỊ DƯ
    // Bạn đã gọi ntc_init_rtos() phía trên rồi
    ntc_init_rtos();

    // ========================================================
    // 3. KHỞI TẠO ADC
    // ========================================================

    // Config ADC Unit 1
    adc_oneshot_unit_init_cfg_t init_config =
    {
        .unit_id = ADC_UNIT_1
    };

    // Tạo ADC Unit
    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(
            &init_config,
            &g_adc_handle
        )
    );

    // Cấu hình channel ADC
    adc_oneshot_chan_cfg_t adc_config =
    {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Độ phân giải mặc định

        .atten = ADC_ATTEN_DB_12          // Đo tối đa khoảng 3.1V
    };

    // ========================================================
    // CẤU HÌNH ADC CHO NTC
    // ========================================================

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            g_adc_handle,
            NTC_ADC_CHANNEL,
            &adc_config
        )
    );

    // ========================================================
    // CẤU HÌNH ADC CHO LDR
    // ========================================================

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            g_adc_handle,
            LDR_XP_CH,
            &adc_config
        )
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            g_adc_handle,
            LDR_XM_CH,
            &adc_config
        )
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            g_adc_handle,
            LDR_YP_CH,
            &adc_config
        )
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            g_adc_handle,
            LDR_YM_CH,
            &adc_config
        )
    );

    // ========================================================
    // 4. TẠO TASK
    // ========================================================

    // Task dò nắng
    // Priority = 5 (cao nhất)
    xTaskCreate(
        ldr_task,              // Hàm task
        "Sun_Tracking",        // Tên task
        4096,                  // Stack size
        (void*)g_adc_handle,   // Tham số truyền vào
        5,                     // Priority
        &ldr_task_handle       // Handle task
    );

    // Task đọc nhiệt độ
    xTaskCreate(
        ntc_task,
        "NTC_Task",
        4096,
        NULL,
        4,
        NULL
    );

    // Task điều khiển bơm
    xTaskCreate(
        pump_task,
        "Pump_Task",
        4096,
        NULL,
        4,
        NULL
    );

    // Task LCD
    xTaskCreate(
        lcd_task,
        "LCD_Task",
        4096,
        NULL,
        2,
        NULL
    );

    // ========================================================
    // 5. TẠO SOFTWARE TIMER
    // ========================================================

    TimerHandle_t light_timer = xTimerCreate(
        "LightCheck",                     // Tên timer
        pdMS_TO_TICKS(LIGHT_CHECK_MS),   // Chu kỳ 3 giây
        pdTRUE,                          // Auto reload
        NULL,                            // Timer ID
        light_check_callback             // Callback function
    );

    // Nếu tạo timer thành công
    if (light_timer != NULL)
    {
        // Start timer
        xTimerStart(light_timer, 0);
    }

    // In log báo hệ thống sẵn sàng
    ESP_LOGI(TAG,
             "He thong da san sang van hanh.");

    // ========================================================
    // LOOP CHÍNH
    // ========================================================

    while (1)
    {
        // app_main không làm gì thêm
        // Các task đã chạy độc lập
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
