#include "i2c_lcd.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LCD_I2C";

esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

// Hàm gửi dữ liệu I2C chuẩn ESP-IDF
static esp_err_t i2c_lcd_send(uint8_t *data_t, size_t size) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_SLAVE_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data_t, size, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C;  // en=1, rs=0, rw=0, bl=1
    data_t[1] = data_u | 0x08;  // en=0, rs=0, rw=0, bl=1
    data_t[2] = data_l | 0x0C;  // en=1, rs=0, rw=0, bl=1
    data_t[3] = data_l | 0x08;  // en=0, rs=0, rw=0, bl=1
    i2c_lcd_send(data_t, 4);
}

static void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D;  // en=1, rs=1, rw=0, bl=1
    data_t[1] = data_u | 0x09;  // en=0, rs=1, rw=0, bl=1
    data_t[2] = data_l | 0x0D;  // en=1, rs=1, rw=0, bl=1
    data_t[3] = data_l | 0x09;  // en=0, rs=1, rw=0, bl=1
    i2c_lcd_send(data_t, 4);
}

void lcd_init(void) {
    vTaskDelay(pdMS_TO_TICKS(100)); // Chờ IC PCF8574 ổn định nguồn
    
    // Reset trình tự 3 bước của HD44780
    lcd_send_cmd(0x30); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_cmd(0x30); vTaskDelay(pdMS_TO_TICKS(1));
    lcd_send_cmd(0x30); vTaskDelay(pdMS_TO_TICKS(10));
    
    lcd_send_cmd(0x20); vTaskDelay(pdMS_TO_TICKS(10)); // Chế độ 4 bit
    
    // Cấu hình hiển thị
    lcd_send_cmd(0x28); vTaskDelay(pdMS_TO_TICKS(1)); // 2 dòng, font 5x8
    lcd_send_cmd(0x08); vTaskDelay(pdMS_TO_TICKS(1)); // Tắt màn hình
    lcd_send_cmd(0x01); vTaskDelay(pdMS_TO_TICKS(5)); // Xóa màn hình
    lcd_send_cmd(0x06); vTaskDelay(pdMS_TO_TICKS(1)); // Dịch con trỏ sang phải
    lcd_send_cmd(0x0C); vTaskDelay(pdMS_TO_TICKS(1)); // Bật màn hình, tắt con trỏ
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5)); // Xóa màn hình cần nhiều thời gian hơn
}

void lcd_put_cur(int row, int col) {
    switch (row) {
        case 0: lcd_send_cmd(0x80 | (col + 0x00)); break;
        case 1: lcd_send_cmd(0x80 | (col + 0x40)); break;
    }
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}