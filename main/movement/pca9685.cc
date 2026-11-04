#include "movement/pca9685.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

static const char* TAG = "PCA9685";

#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06

PCA9685& PCA9685::GetInstance() {
    static PCA9685 inst;
    return inst;
}

bool PCA9685::Initialize(int sda_pin, int scl_pin, uint8_t address, int i2c_port) {
    if (inited_) return true;

    device_address_ = address;

    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = i2c_port;
    bus_config.sda_io_num = (gpio_num_t)sda_pin;
    bus_config.scl_io_num = (gpio_num_t)scl_pin;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus create failed: %d", err);
        return false;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = 100000;
    
    err = i2c_master_bus_add_device(bus_handle_, &dev_cfg, &dev_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C add device failed: %d", err);
        return false;
    }

    // Reset PCA9685
    WriteRegister(PCA9685_MODE1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));

    inited_ = true;
    ESP_LOGI(TAG, "PCA9685 initialized at 0x%02x on SDA:%d SCL:%d", address, sda_pin, scl_pin);
    return true;
}

void PCA9685::SetPWMFreq(float freq) {
    if (!inited_) return;
    freq *= 0.9f;  // Correct for overshoot in the frequency setting (see adafruit library)
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;
    uint8_t prescale = floor(prescaleval + 0.5f);
    
    uint8_t oldmode = ReadRegister(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    WriteRegister(PCA9685_MODE1, newmode); // go to sleep
    WriteRegister(PCA9685_PRESCALE, prescale); // set the prescaler
    WriteRegister(PCA9685_MODE1, oldmode);
    vTaskDelay(pdMS_TO_TICKS(5));
    WriteRegister(PCA9685_MODE1, oldmode | 0xa0);  // This sets the MODE1 register to turn on auto increment.
}

void PCA9685::SetPWM(uint8_t num, uint16_t on, uint16_t off) {
    if (!inited_) return;
    uint8_t buf[5];
    buf[0] = LED0_ON_L + 4 * num;
    buf[1] = on & 0xFF;
    buf[2] = on >> 8;
    buf[3] = off & 0xFF;
    buf[4] = off >> 8;
    i2c_master_transmit(dev_handle_, buf, sizeof(buf), -1);
}

esp_err_t PCA9685::WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(dev_handle_, buf, sizeof(buf), -1);
}

uint8_t PCA9685::ReadRegister(uint8_t reg) {
    uint8_t value = 0;
    i2c_master_transmit_receive(dev_handle_, &reg, 1, &value, 1, -1);
    return value;
}
