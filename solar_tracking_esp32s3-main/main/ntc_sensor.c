#include "ntc_sensor.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "NTC_SENSOR";

// Khởi tạo các handle
QueueHandle_t ntc_queue = NULL;
SemaphoreHandle_t ntc_mutex = NULL;
SemaphoreHandle_t ntc_threshold_sem = NULL;

#define R_NOMINAL    10000.0f  
#define T_NOMINAL    298.15f   
#define BETA         3950.0f   
#define R_REF        10000.0f  
#define ADC_MAX      4095.0f   
#define TEMP_THRESHOLD 40.0f   // Ngưỡng nhiệt độ để kích hoạt Semaphore (ví dụ 40 độ C)

void ntc_init_rtos(void) {
    // Tạo Mutex để bảo vệ tài nguyên ADC
    ntc_mutex = xSemaphoreCreateMutex();
    // Tạo Queue chứa tối đa 5 giá trị nhiệt độ (float)
    ntc_queue = xQueueCreate(5, sizeof(float));
    // Tạo Binary Semaphore để báo động nhiệt độ cao
    ntc_threshold_sem = xSemaphoreCreateBinary();
}

float ntc_read_temp(adc_oneshot_unit_handle_t adc_handle) {
    int raw_adc;
    esp_err_t ret;

    // 1. Dùng Mutex để tranh chấp quyền đọc ADC
    if (xSemaphoreTake(ntc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        ret = adc_oneshot_read(adc_handle, NTC_ADC_CHANNEL, &raw_adc);
        xSemaphoreGive(ntc_mutex); // Nhả Mutex ngay sau khi đọc xong ADC thô
    } else {
        ESP_LOGW(TAG, "Không thể lấy Mutex để đọc ADC");
        return -99.0f;
    }

    if (ret != ESP_OK || raw_adc <= 0 || raw_adc >= 4095) {
        ESP_LOGW(TAG, "Lỗi đọc ADC (ADC: %d)", raw_adc);
        return -99.0f; 
    }

    // --- Tính toán nhiệt độ ---
    float r_ntc = R_REF / ((ADC_MAX / (float)raw_adc) - 1.0f);
    float temp = r_ntc / R_NOMINAL;
    temp = log(temp);
    temp /= BETA;
    temp += 1.0f / T_NOMINAL;
    temp = 1.0f / temp - 273.15f;

    // 2. Gửi dữ liệu vào Queue (không chờ nếu queue đầy để tránh block task cảm biến)
    if (ntc_queue != NULL) {
        xQueueSend(ntc_queue, &temp, 0);
    }

    // 3. Kích hoạt Semaphore nếu nhiệt độ quá cao (ví dụ để bật bơm phun sương ngay lập tức)
    if (temp > TEMP_THRESHOLD && ntc_threshold_sem != NULL) {
        xSemaphoreGive(ntc_threshold_sem);
    }

    ESP_LOGI(TAG, "Temp: %.2f C", temp);
    return temp;
}
