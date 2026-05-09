#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "driver/i2c.h"

// --- ĐÃ SỬA LẠI CHÂN CHO ESP32-S3 ---
// Bạn có thể đổi sang chân khác miễn là nó còn trống trên board của bạn
#define I2C_MASTER_SCL_IO           15      // Chân SCL gán vào GPIO 15
#define I2C_MASTER_SDA_IO           16      // Chân SDA gán vào GPIO 16

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define LCD_SLAVE_ADDR              0x27    // Địa chỉ I2C (có thể là 0x3F tùy module)

esp_err_t i2c_master_init(void);
void lcd_init(void);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_send_string(char *str);

#endif