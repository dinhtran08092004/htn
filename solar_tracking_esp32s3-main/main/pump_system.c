#include "pump_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ntc_sensor.h" // Để sử dụng ntc_threshold_sem

static const char *TAG = "PUMP_SYSTEM";

QueueHandle_t pump_queue = NULL;
SemaphoreHandle_t pump_mutex = NULL;

void pump_init_rtos(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PUMP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Tạo Mutex bảo vệ GPIO
    pump_mutex = xSemaphoreCreateMutex();
    // Tạo Queue nhận trạng thái (bool)
    pump_queue = xQueueCreate(5, sizeof(bool));
    
    // Đảm bảo bơm tắt khi khởi động
    pump_control(false); 
}

void pump_control(bool state) {
    if (pump_mutex != NULL) {
        if (xSemaphoreTake(pump_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            gpio_set_level(PUMP_PIN, state ? 1 : 0); 
            xSemaphoreGive(pump_mutex);
        }
    }
}

// Task xử lý logic bơm
void pump_task(void *pvParameters) {
    bool pump_state = false;
    
    while (1) {
        // 1. Kiểm tra Queue: Có lệnh bật/tắt thủ công hoặc theo lịch không?
        if (xQueueReceive(pump_queue, &pump_state, 0) == pdTRUE) {
            ESP_LOGI(TAG, "Nhận lệnh từ Queue: %s", pump_state ? "BẬT" : "TẮT");
            pump_control(pump_state);
        }

        // 2. Kiểm tra Semaphore: Nhiệt độ từ NTC có vượt ngưỡng khẩn cấp không?
        // Đợi semaphore trong 100ms, nếu ntc_threshold_sem được "Give", bơm sẽ bật ngay
        if (xSemaphoreTake(ntc_threshold_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGW(TAG, "CẢNH BÁO NHIỆT ĐỘ! Kích hoạt bơm khẩn cấp.");
            pump_control(true);
            vTaskDelay(pdMS_TO_TICKS(5000)); // Bật bơm trong 5 giây rồi kiểm tra lại
            pump_control(false);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Tránh chiếm dụng CPU
    }
}
