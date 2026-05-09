#ifndef NTC_SENSOR_H
#define NTC_SENSOR_H

#include "esp_adc/adc_oneshot.h"

// Đổi lại channel đúng với chân GPIO bạn cắm NTC nhé
#define NTC_ADC_CHANNEL ADC_CHANNEL_0 

float ntc_read_temp(adc_oneshot_unit_handle_t adc_handle);

#endif