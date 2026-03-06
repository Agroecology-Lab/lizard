#pragma once

#include "driver/i2c.h"
#include "module.h"
#include <cstdint>

/**
 * Lizard module for the Zio Qwiic 4-Channel DC Motor Controller.
 *
 * Hardware: PCA9685 PWM generator + TB6612FNG dual H-bridge drivers.
 * The PCA9685 (I2C, default address 0x40) drives four motors via
 * three PWM channels each (IN1, IN2, PWM/speed), for a total of 12
 * channels out of the 16 available.
 *
 * Channel mapping (matches Zio demo code):
 *   Motor 1: ch0=IN1, ch1=IN2, ch2=PWM
 *   Motor 2: ch5=IN1, ch6=IN2, ch7=PWM
 *   Motor 3: ch8=IN1, ch9=IN2, ch10=PWM
 *   Motor 4: ch13=IN1, ch14=IN2, ch15=PWM
 *
 * Speed is a float in [-1.0, 1.0]:
 *   +1.0  → full forward
 *   -1.0  → full reverse
 *    0.0  → coast (stop)
 * Brake mode sets both IN pins HIGH and PWM to full.
 */

class ZioMotor;
using ZioMotor_ptr = std::shared_ptr<ZioMotor>;

class ZioMotor : public Module {
private:
    const i2c_port_t i2c_port;
    const uint8_t address;

    // PCA9685 register addresses
    static constexpr uint8_t PCA9685_MODE1       = 0x00;
    static constexpr uint8_t PCA9685_PRESCALE     = 0xFE;
    static constexpr uint8_t PCA9685_LED0_ON_L    = 0x06; // base of channel registers
    // Each channel occupies 4 bytes: ON_L, ON_H, OFF_L, OFF_H

    // TB6612 channel mapping: {IN1_ch, IN2_ch, PWM_ch}
    static constexpr uint8_t MOTOR_CHANNELS[4][3] = {
        {0,  1,  2},   // Motor 1
        {5,  6,  7},   // Motor 2
        {8,  9,  10},  // Motor 3
        {13, 14, 15},  // Motor 4
    };

    void write_register(uint8_t reg, uint8_t value) const;
    void set_pwm_channel(uint8_t channel, uint16_t on, uint16_t off) const;
    void set_channel_full_on(uint8_t channel) const;
    void set_channel_full_off(uint8_t channel) const;
    void apply_speed(uint8_t motor_index, float speed) const;
    void apply_brake(uint8_t motor_index) const;

public:
    ZioMotor(const std::string name,
             i2c_port_t i2c_port,
             gpio_num_t sda_pin,
             gpio_num_t scl_pin,
             uint8_t address,
             int clk_speed,
             float pwm_frequency);

    void step() override;
    void call(const std::string method_name,
              const std::vector<ConstExpression_ptr> arguments) override;
    static const std::map<std::string, Variable_ptr> get_defaults();
};
