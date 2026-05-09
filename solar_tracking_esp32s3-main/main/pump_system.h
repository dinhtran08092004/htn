#ifndef PUMP_SYSTEM_H
#define PUMP_SYSTEM_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdbool.h>

#define PUMP_PIN 18 

// Khai báo các handle dùng chung
extern QueueHandle_t pump_queue;
extern SemaphoreHandle_t pump_mutex;

void pump_init_rtos(void);
void pump_control(bool state);
void pump_task(void *pvParameters);

#endif
