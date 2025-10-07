#include "zio_motor.h"
#include "esp_log.h"
#include <cmath>
#include <stdexcept>

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static const char *TAG = "ZioMotor";

// ============================================================================
// PCA9685 Implementation
// ============================================================================

PCA9685::PCA9685(i2c_port_t port, uint8_t addr)
    : i2c_port(port), i2c_addr(addr), initialized(false) {
}

PCA9685::~PCA9685() {
    // Cleanup if needed
}

void PCA9685::write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
        throw std::runtime_error("PCA9685 I2C write failed");
    }
}

uint8_t PCA9685::read_register(uint8_t reg) {
    uint8_t data = 0;

    // Write register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register address 0x%02X: %s", reg, esp_err_to_name(ret));
        throw std::runtime_error("PCA9685 I2C write failed");
    }

    // Read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
        throw std::runtime_error("PCA9685 I2C read failed");
    }

    return data;
}

void PCA9685::begin() {
    if (initialized) {
        return;
    }

    reset();

    // Set MODE1 to enable auto-increment
    write_register(MODE1, MODE1_AI | MODE1_ALLCALL);
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for oscillator

    // Set MODE2 for totem-pole outputs
    write_register(MODE2, MODE2_OUTDRV);

    initialized = true;
    ESP_LOGI(TAG, "PCA9685 initialized at address 0x%02X", i2c_addr);
}

void PCA9685::reset() {
    write_register(MODE1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void PCA9685::set_pwm_freq(float freq) {
    // Constrain frequency to valid range
    freq = constrain(freq, 24.0f, 1526.0f);

    // Calculate prescaler value
    // Formula: prescale = round(25MHz / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)(25000000.0f / (4096.0f * freq) + 0.5f) - 1;

    ESP_LOGI(TAG, "Setting PWM frequency to %.2f Hz (prescale: %d)", freq, prescale);

    // Must be in sleep mode to change prescaler
    uint8_t oldmode = read_register(MODE1);
    uint8_t newmode = (oldmode & 0x7F) | MODE1_SLEEP; // Sleep
    write_register(MODE1, newmode);
    write_register(PRESCALE, prescale);
    write_register(MODE1, oldmode); // Restore old mode
    vTaskDelay(pdMS_TO_TICKS(5));

    // Restart
    write_register(MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}

void PCA9685::set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    if (channel > 15) {
        throw std::runtime_error("Invalid channel number");
    }

    // Calculate register addresses
    uint8_t reg_base = LED0_ON_L + 4 * channel;

    // Write all 4 registers using auto-increment
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_base, true);
    i2c_master_write_byte(cmd, on & 0xFF, true);        // ON_L
    i2c_master_write_byte(cmd, (on >> 8) & 0x0F, true); // ON_H
    i2c_master_write_byte(cmd, off & 0xFF, true);       // OFF_L
    i2c_master_write_byte(cmd, (off >> 8) & 0x0F, true); // OFF_H
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM on channel %d: %s", channel, esp_err_to_name(ret));
        throw std::runtime_error("PCA9685 set PWM failed");
    }
}

void PCA9685::set_pin(uint8_t channel, uint16_t value, bool invert) {
    if (channel > 15) {
        throw std::runtime_error("Invalid channel number");
    }

    // Constrain value to 12-bit range
    value = constrain(value, 0, 4095);

    if (invert) {
        value = 4095 - value;
    }

    if (value == 4095) {
        // Full ON
        set_pwm(channel, 4096, 0);
    } else if (value == 0) {
        // Full OFF
        set_pwm(channel, 0, 4096);
    } else {
        // PWM value
        set_pwm(channel, 0, value);
    }
}

// ============================================================================
// ZioMotor Implementation
// ============================================================================

REGISTER_MODULE_DEFAULTS(ZioMotor)

const std::map<std::string, Variable_ptr> ZioMotor::get_defaults() {
    return {
        {"power", std::make_shared<NumberVariable>(0.0)},
        {"enabled", std::make_shared<BooleanVariable>(true)},
    };
}

ZioMotor::ZioMotor(const std::string name,
                   const PCA9685_ptr pca9685,
                   const unsigned int motor_number)
    : Module(ModuleType::zio_motor, name),
      motor_number(constrain(motor_number, 0, 3)),
      pca9685(pca9685),
      enabled(true),
      current_power(0.0) {

    if (this->motor_number != motor_number) {
        throw std::runtime_error("Invalid motor number (must be 0-3)");
    }

    this->properties = ZioMotor::get_defaults();
    init_pins();

    ESP_LOGI(TAG, "ZioMotor '%s' created for motor %d", name.c_str(), motor_number);
}

void ZioMotor::init_pins() {
    // Pin mapping based on motor number
    // Motor 0 (A101): PWM=0, IN1=1, IN2=2, STBY=3
    // Motor 1 (B101): PWM=6, IN1=4, IN2=5, STBY=3
    // Motor 2 (A201): PWM=7, IN1=8, IN2=9, STBY=10
    // Motor 3 (B201): PWM=13, IN1=11, IN2=12, STBY=10

    switch (motor_number) {
    case 0: // A101
        pins.pwm = 0;
        pins.in1 = 1;
        pins.in2 = 2;
        pins.stby = 3;
        break;
    case 1: // B101
        pins.pwm = 6;
        pins.in1 = 4;
        pins.in2 = 5;
        pins.stby = 3;
        break;
    case 2: // A201
        pins.pwm = 7;
        pins.in1 = 8;
        pins.in2 = 9;
        pins.stby = 10;
        break;
    case 3: // B201
        pins.pwm = 13;
        pins.in1 = 11;
        pins.in2 = 12;
        pins.stby = 10;
        break;
    default:
        throw std::runtime_error("Invalid motor number");
    }

    // Initialize motor to stopped state
    pca9685->set_pin(pins.stby, 0, false); // Disable
    pca9685->set_pin(pins.pwm, 0, false);  // Zero speed
}

void ZioMotor::set_direction(bool forward) {
    if (forward) {
        // Forward: IN1=HIGH, IN2=LOW
        pca9685->set_pin(pins.in1, 4095, false);
        pca9685->set_pin(pins.in2, 0, false);
    } else {
        // Reverse: IN1=LOW, IN2=HIGH
        pca9685->set_pin(pins.in1, 0, false);
        pca9685->set_pin(pins.in2, 4095, false);
    }
}

void ZioMotor::set_speed(uint16_t speed) {
    speed = constrain(speed, 0, 4095);
    pca9685->set_pin(pins.pwm, speed, false);
}

void ZioMotor::step() {
    // Sync enabled state
    if (this->properties.at("enabled")->boolean_value != this->enabled) {
        if (this->properties.at("enabled")->boolean_value) {
            this->enable();
        } else {
            this->disable();
        }
    }

    // Update power property
    this->properties["power"]->number_value = this->current_power;

    Module::step();
}

void ZioMotor::call(const std::string method_name,
                    const std::vector<ConstExpression_ptr> arguments) {
    if (method_name == "power") {
        Module::expect(arguments, 1, numbery);
        this->power(arguments[0]->evaluate_number());
    } else if (method_name == "stop") {
        Module::expect(arguments, 0);
        this->stop();
    } else if (method_name == "brake") {
        Module::expect(arguments, 0);
        this->brake();
    } else if (method_name == "enable") {
        Module::expect(arguments, 0);
        this->enable();
    } else if (method_name == "disable") {
        Module::expect(arguments, 0);
        this->disable();
    } else {
        Module::call(method_name, arguments);
    }
}

void ZioMotor::enable() {
    enabled = true;
    pca9685->set_pin(pins.stby, 4095, false); // Enable standby
    this->properties.at("enabled")->boolean_value = true;
    ESP_LOGI(TAG, "Motor %d enabled", motor_number);
}

void ZioMotor::disable() {
    pca9685->set_pin(pins.stby, 0, false); // Disable standby
    enabled = false;
    current_power = 0.0;
    this->properties.at("enabled")->boolean_value = false;
    ESP_LOGI(TAG, "Motor %d disabled", motor_number);
}

void ZioMotor::power(double value) {
    if (!enabled) {
        ESP_LOGW(TAG, "Motor %d is disabled, ignoring power command", motor_number);
        return;
    }

    // Constrain power to -1.0 to 1.0
    value = constrain(value, -1.0, 1.0);
    current_power = value;

    // Determine direction and speed
    bool forward = (value >= 0);
    uint16_t speed = (uint16_t)(std::abs(value) * 4095.0);

    // Set direction
    set_direction(forward);

    // Set speed
    set_speed(speed);

    ESP_LOGD(TAG, "Motor %d power: %.2f (speed: %d, forward: %d)",
             motor_number, value, speed, forward);
}

void ZioMotor::stop() {
    // Coast stop - disable motor
    disable();
}

void ZioMotor::brake() {
    if (!enabled) {
        return;
    }

    // Active brake: IN1=LOW, IN2=LOW
    pca9685->set_pin(pins.in1, 0, false);
    pca9685->set_pin(pins.in2, 0, false);
    pca9685->set_pin(pins.pwm, 0, false);
    current_power = 0.0;

    ESP_LOGI(TAG, "Motor %d braking", motor_number);
}
