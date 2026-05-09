#include <stdio.h>
#include "freertos/FreeRTOS.h"  
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// Include các module dự án
#include "i2c_lcd.h"
#include "ntc_sensor.h"
#include "ldr_logic.h"
#include "motor_control.h"
#include "pump_system.h"

// Biến toàn cục từ các module khác
extern int g_lcd_max_light;
extern bool g_lcd_is_tracking;

static const char *TAG = "MAIN_SYSTEM";

// Cấu hình kiểm tra ánh sáng
#define LIGHT_CHECK_MS  3000    

// Handle ADC dùng chung toàn hệ thống
static adc_oneshot_unit_handle_t g_adc_handle;
SemaphoreHandle_t adc_mutex = NULL; 
TaskHandle_t ldr_task_handle = NULL;

/**
 * @brief Task đọc nhiệt độ định kỳ
 * Sử dụng hàm ntc_read_temp đã tích hợp sẵn Mutex bảo vệ ADC và Queue gửi dữ liệu.
 */
void ntc_task(void *pvParameters) {
    ESP_LOGI(TAG, "NTC Task started.");
    while (1) {
        // Hàm này tự động gửi giá trị vào ntc_queue và kích hoạt ntc_threshold_sem nếu quá nóng
        ntc_read_temp(g_adc_handle); 
        vTaskDelay(pdMS_TO_TICKS(5000)); // Cập nhật mỗi 5 giây
    }
}

/**
 * @brief Timer callback kiểm tra ánh sáng
 * Nếu trời sáng lại (dựa trên SUN_LIMIT), sẽ resume Task dò hướng nắng.
 */
static void light_check_callback(TimerHandle_t xTimer) {
    int val = 0;
    // Sử dụng ntc_mutex (khai báo extern trong ntc_sensor.h) để dùng chung bộ ADC
    if (xSemaphoreTake(ntc_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        adc_oneshot_read(g_adc_handle, LDR_XP_CH, &val);
        xSemaphoreGive(ntc_mutex);
    }

    if (val >= SUN_LIMIT) {
        if (eTaskGetState(ldr_task_handle) == eSuspended) {
            ESP_LOGI(TAG, "Troi sang (LDR=%d), khoi phuc Sun Tracking Task!", val);
            vTaskResume(ldr_task_handle);
        }
    }
}

/**
 * @brief Task hiển thị LCD
 * Lấy dữ liệu từ Queue và trạng thái thực tế của thiết bị để cập nhật màn hình.
 */
void lcd_task(void *pvParameters) {
    i2c_master_init();
    lcd_init();
    lcd_clear();
    
    char buffer[17];
    float current_temp = 0.0f;
    
    while (1) {
        // Xem giá trị mới nhất từ Queue mà không xóa nó khỏi hàng đợi (Peek)
        if (xQueuePeek(ntc_queue, &current_temp, 0) != pdTRUE) {
            // Nếu chưa có dữ liệu, giữ nguyên current_temp = 0
        }

        // Dòng 1: Nhiệt độ & Trạng thái Bơm
        // Kiểm tra trực tiếp mức GPIO của chân PUMP_PIN để biết bơm đang ON hay OFF
        snprintf(buffer, sizeof(buffer), "T:%5.1fC P:%-3s", 
                 current_temp, (gpio_get_level(PUMP_PIN) ? "ON" : "OFF"));
        lcd_put_cur(0, 0);
        lcd_send_string(buffer);
        
        // Dòng 2: Cường độ sáng & Trạng thái Tracking
        snprintf(buffer, sizeof(buffer), "L:%-4d Trk:%s ", 
                 g_lcd_max_light, g_lcd_is_tracking ? "RUN" : "OFF");
        lcd_put_cur(1, 0);
        lcd_send_string(buffer);
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Cập nhật LCD mỗi 1 giây
    }
}

/**
 * @brief Hàm chính khởi tạo toàn bộ hệ thống
 */
void app_main(void) {
    ESP_LOGI(TAG, "=== KHOI DONG SOLAR TRACKER ESP32-S3 ===");

    // 1. Khởi tạo phần cứng cơ bản
    motor_init();
    
    // 2. Khởi tạo các cơ chế RTOS (Mutex, Queue, Semaphore)
    // Rất quan trọng: Phải gọi trước khi tạo Task
    ntc_init_rtos();
    pump_init_rtos();
adc_mutex = xSemaphoreCreateMutex();
    ntc_init_rtos();
    // 3. Cấu hình bộ ADC Unit 1
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &g_adc_handle));
    
    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12, // Dải đo lên tới ~3.1V cho ESP32-S3
    };
    
    // Cấu hình các kênh ADC cho NTC và 4 quang trở LDR
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, NTC_ADC_CHANNEL, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, LDR_XP_CH, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, LDR_XM_CH, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, LDR_YP_CH, &adc_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc_handle, LDR_YM_CH, &adc_config));

    // 4. Tạo các Task xử lý đa nhiệm
    // Task dò nắng (Ưu tiên cao nhất: 5)
    xTaskCreate(ldr_task, "Sun_Tracking", 4096, (void*)g_adc_handle, 5, &ldr_task_handle);
    
    // Task cảm biến nhiệt độ (Ưu tiên: 4)
    xTaskCreate(ntc_task, "NTC_Task", 4096, NULL, 4, NULL);
    
    // Task quản lý bơm (Ưu tiên: 4 - Chờ lệnh từ Semaphore/Queue)
    xTaskCreate(pump_task, "Pump_Task", 4096, NULL, 4, NULL);
    
    // Task hiển thị (Ưu tiên thấp hơn: 2)
    xTaskCreate(lcd_task, "LCD_Task", 4096, NULL, 2, NULL);
    
    // 5. Khởi tạo Software Timer kiểm tra ánh sáng định kỳ
    TimerHandle_t light_timer = xTimerCreate(
        "LightCheck", pdMS_TO_TICKS(LIGHT_CHECK_MS), pdTRUE, NULL, light_check_callback
    );
    if (light_timer != NULL) {
        xTimerStart(light_timer, 0);
    }
    
    ESP_LOGI(TAG, "He thong da san sang van hanh.");
    
    // Loop trống cho app_main
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
