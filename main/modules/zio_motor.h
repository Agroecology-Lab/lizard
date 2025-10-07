#pragma once

#include "driver/i2c.h"
#include "module.h"
#include <memory>

// Forward declarations
class PCA9685;
using PCA9685_ptr = std::shared_ptr<PCA9685>;

class ZioMotor;
using ZioMotor_ptr = std::shared_ptr<ZioMotor>;

/**
 * @brief PCA9685 PWM controller driver for I2C communication
 * 
 * Handles low-level I2C communication with the PCA9685 16-channel PWM controller.
 * Used by ZIO Motor Controller to drive TB6612 H-bridge motor drivers.
 */
class PCA9685 {
private:
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    bool initialized;

    // PCA9685 Register addresses
    static constexpr uint8_t MODE1 = 0x00;
    static constexpr uint8_t MODE2 = 0x01;
    static constexpr uint8_t SUBADR1 = 0x02;
    static constexpr uint8_t SUBADR2 = 0x03;
    static constexpr uint8_t SUBADR3 = 0x04;
    static constexpr uint8_t PRESCALE = 0xFE;
    static constexpr uint8_t LED0_ON_L = 0x06;
    static constexpr uint8_t LED0_ON_H = 0x07;
    static constexpr uint8_t LED0_OFF_L = 0x08;
    static constexpr uint8_t LED0_OFF_H = 0x09;
    static constexpr uint8_t ALL_LED_ON_L = 0xFA;
    static constexpr uint8_t ALL_LED_ON_H = 0xFB;
    static constexpr uint8_t ALL_LED_OFF_L = 0xFC;
    static constexpr uint8_t ALL_LED_OFF_H = 0xFD;

    // MODE1 bits
    static constexpr uint8_t MODE1_RESTART = 0x80;
    static constexpr uint8_t MODE1_SLEEP = 0x10;
    static constexpr uint8_t MODE1_ALLCALL = 0x01;
    static constexpr uint8_t MODE1_AI = 0x20; // Auto-increment

    // MODE2 bits
    static constexpr uint8_t MODE2_OUTDRV = 0x04;

    void write_register(uint8_t reg, uint8_t value);
    uint8_t read_register(uint8_t reg);

public:
    PCA9685(i2c_port_t port, uint8_t addr = 0x40);
    ~PCA9685();

    void begin();
    void reset();
    void set_pwm_freq(float freq);
    void set_pwm(uint8_t channel, uint16_t on, uint16_t off);
    void set_pin(uint8_t channel, uint16_t value, bool invert = false);
};

/**
 * @brief ZIO Motor Controller module for Lizard
 * 
 * Controls a single DC motor on the ZIOCC Qwiic 4-Channel DC Motor Controller.
 * The controller uses PCA9685 PWM driver + TB6612 H-bridge for motor control.
 * 
 * Motor numbering:
 *   0 = A101 (Motor A on driver 1)
 *   1 = B101 (Motor B on driver 1)
 *   2 = A201 (Motor A on driver 2)
 *   3 = B201 (Motor B on driver 2)
 * 
 * Usage in Lizard script:
 *   motor = ZioMotor motor_name motor_number i2c_port i2c_addr
 *   motor.power(0.5)  # 50% forward
 *   motor.power(-0.5) # 50% reverse
 *   motor.stop()      # Stop motor
 *   motor.brake()     # Active brake
 */
class ZioMotor : public Module {
private:
    const unsigned int motor_number;
    const PCA9685_ptr pca9685;
    bool enabled;
    double current_power;

    // Pin mapping for each motor
    struct MotorPins {
        uint8_t pwm;   // PWM speed control pin
        uint8_t in1;   // Direction control pin 1
        uint8_t in2;   // Direction control pin 2
        uint8_t stby;  // Standby (enable) pin
    };

    MotorPins pins;

    void init_pins();
    void set_direction(bool forward);
    void set_speed(uint16_t speed);

public:
    ZioMotor(const std::string name,
             const PCA9685_ptr pca9685,
             const unsigned int motor_number);

    void step() override;
    void call(const std::string method_name,
              const std::vector<ConstExpression_ptr> arguments) override;
    static const std::map<std::string, Variable_ptr> get_defaults();

    void enable();
    void disable();
    void power(double value);  // -1.0 to 1.0
    void stop();               // Coast stop (disable)
    void brake();              // Active brake
};
