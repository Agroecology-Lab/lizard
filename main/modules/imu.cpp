#include "imu.h"

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

Imu::Imu(const std::string name, i2c_port_t i2c_port, gpio_num_t sda_pin, gpio_num_t scl_pin, uint8_t address, int clk_speed)
    : Module(imu, name), i2c_port(i2c_port), address(address) {
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = sda_pin,
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_io_num = scl_pin;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = clk_speed;
    config.clk_flags = 0;
    if (i2c_param_config(i2c_port, &config) != ESP_OK) {
        throw std::runtime_error("could not configure i2c port");
    }
    if (i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_TX_BUF_DISABLE, I2C_MASTER_RX_BUF_DISABLE, 0) != ESP_OK) {
        throw std::runtime_error("could not install i2c driver");
    }
    if (i2c_set_timeout(i2c_port, 30000) != ESP_OK) {
        throw std::runtime_error("could not set i2c timeout");
    }
    this->bno = std::make_shared<BNO055>((i2c_port_t)i2c_port, address);
    try {
        this->bno->begin();
        this->bno->enableExternalCrystal();
        this->bno->setOprModeNdof();
    } catch (BNO055BaseException &ex) {
        throw std::runtime_error(std::string("imu setup failed: ") + ex.what());
    } catch (std::exception &ex) {
        throw std::runtime_error(std::string("imu setup failed: ") + ex.what());
    }
}

void Imu::step() {
    bno055_vector_t v = this->bno->getVectorEuler();
    printf("%8.3f %8.3f %8.3f\n", v.x, v.y, v.z);

    Module::step();
}