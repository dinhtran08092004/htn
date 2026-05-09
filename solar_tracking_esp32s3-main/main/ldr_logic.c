#include "ldr_logic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "motor_control.h"
#include <stdlib.h>

static const char *TAG = "LDR_LOGIC";
extern SemaphoreHandle_t adc_mutex;
extern TaskHandle_t ldr_task_handle;   // THÊM MỚI - handle từ main.c

#define TOL_START   300
#define TOL_STOP    100
#define SUN_LIMIT   600
#define LOOP_MS     50
#define STEP_ANGLE  1

static int angle_x = 90;
static int angle_y = 90;
static bool is_tracking_x = false;
static bool is_tracking_y = false;
// Biến toàn cục để share cho LCD
int g_lcd_max_light = 0;
bool g_lcd_is_tracking = false;
static int adc_read_stable(adc_oneshot_unit_handle_t handle, adc_channel_t channel) {
    int val = 0;
    adc_oneshot_read(handle, channel, &val);
    vTaskDelay(pdMS_TO_TICKS(5));
    adc_oneshot_read(handle, channel, &val);
    return val;
}

void ldr_task(void *pvParameters) {
    ESP_LOGI(TAG, "LDR Task chạy...");
    adc_oneshot_unit_handle_t adc_handle = (adc_oneshot_unit_handle_t)pvParameters;

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        int right = 0, left = 0, top = 0, bottom = 0;

        if (xSemaphoreTake(adc_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            right  = adc_read_stable(adc_handle, LDR_XP_CH);
            left   = adc_read_stable(adc_handle, LDR_XM_CH);
            top    = adc_read_stable(adc_handle, LDR_YP_CH);
            bottom = adc_read_stable(adc_handle, LDR_YM_CH);
            xSemaphoreGive(adc_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
            continue;
        }

        int max_val = left;
        if (right > max_val) max_val = right;
        if (top > max_val) max_val = top;
        if (bottom > max_val) max_val = bottom;

        ESP_LOGI(TAG, "R:%d L:%d T:%d B:%d | max:%d", right, left, top, bottom, max_val);
        g_lcd_max_light = max_val;
        g_lcd_is_tracking = (is_tracking_x || is_tracking_y);
        // THAY ĐỔI - Trời tối: Suspend task thay vì delay 1 giây
        if (max_val < SUN_LIMIT) {
            ESP_LOGI(TAG, "Trời tối, LDR Task tạm dừng...");
            vTaskSuspend(ldr_task_handle);  // Tự suspend chính mình
            // Khi được Resume từ bên ngoài, tiếp tục chạy từ đây
            ESP_LOGI(TAG, "Trời sáng, LDR Task tiếp tục!");
            continue;
        }

        int diff_x = left - right;
        int diff_y = top - bottom;

        // --- XỬ LÝ TRỤC X ---
        if (!is_tracking_x && abs(diff_x) > TOL_START) is_tracking_x = true;
        else if (is_tracking_x && abs(diff_x) < TOL_STOP) is_tracking_x = false;

        if (is_tracking_x) {
            if (diff_x > 0) angle_x -= STEP_ANGLE;
            else            angle_x += STEP_ANGLE;
            if (angle_x > 180) angle_x = 180;
            if (angle_x < 0)   angle_x = 0;
            motor_msg_t msg_x = {'X', angle_x};
            xQueueSend(motor_queue, &msg_x, 0);
        }

        // --- XỬ LÝ TRỤC Y ---
        if (!is_tracking_y && abs(diff_y) > TOL_START) is_tracking_y = true;
        else if (is_tracking_y && abs(diff_y) < TOL_STOP) is_tracking_y = false;

        if (is_tracking_y) {
            if (diff_y > 0) angle_y -= STEP_ANGLE;
            else            angle_y += STEP_ANGLE;
            if (angle_y > 180) angle_y = 180;
            if (angle_y < 0)   angle_y = 0;
            motor_msg_t msg_y = {'Y', angle_y};
            xQueueSend(motor_queue, &msg_y, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}