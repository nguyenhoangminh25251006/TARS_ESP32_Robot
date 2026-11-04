#pragma once

#include <stdint.h>
#include <driver/i2c_master.h>
#include <esp_err.h>

class PCA9685 {
public:
    static PCA9685& GetInstance();

    // Initialize I2C and PCA9685. Default to SDA=21, SCL=22.
    bool Initialize(int sda_pin = 21, int scl_pin = 22, uint8_t address = 0x40, int i2c_port = -1);

    // Set the PWM frequency (e.g. 50Hz for servos)
    void SetPWMFreq(float freq);

    // Set PWM pulse (0-4095) for a specific channel (0-15)
    void SetPWM(uint8_t num, uint16_t on, uint16_t off);

private:
    PCA9685() = default;
    ~PCA9685() = default;

    uint8_t device_address_ = 0x40;
    i2c_master_bus_handle_t bus_handle_ = nullptr;
    i2c_master_dev_handle_t dev_handle_ = nullptr;
    bool inited_ = false;

    esp_err_t WriteRegister(uint8_t reg, uint8_t value);
    uint8_t ReadRegister(uint8_t reg);
};
