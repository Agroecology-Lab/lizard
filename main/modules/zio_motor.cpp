#include "zio_motor.h"
#include <cmath>
#include <stdexcept>

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

REGISTER_MODULE_DEFAULTS(ZioMotor)

// Static member definition
constexpr uint8_t ZioMotor::MOTOR_CHANNELS[4][3];

const std::map<std::string, Variable_ptr> ZioMotor::get_defaults() {
    return {
        {"m1_speed", std::make_shared<NumberVariable>(0.0)},
        {"m2_speed", std::make_shared<NumberVariable>(0.0)},
        {"m3_speed", std::make_shared<NumberVariable>(0.0)},
        {"m4_speed", std::make_shared<NumberVariable>(0.0)},
        {"m1_brake", std::make_shared<BooleanVariable>(false)},
        {"m2_brake", std::make_shared<BooleanVariable>(false)},
        {"m3_brake", std::make_shared<BooleanVariable>(false)},
        {"m4_brake", std::make_shared<BooleanVariable>(false)},
    };
}

ZioMotor::ZioMotor(const std::string name,
                   i2c_port_t i2c_port,
                   gpio_num_t sda_pin,
                   gpio_num_t scl_pin,
                   uint8_t address,
                   int clk_speed,
                   float pwm_frequency)
    : Module(zio_motor, name), i2c_port(i2c_port), address(address) {

    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = sda_pin;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_io_num = scl_pin;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = clk_speed;
    config.clk_flags = 0;

    if (i2c_param_config(i2c_port, &config) != ESP_OK) {
        throw std::runtime_error("zio_motor: could not configure i2c port");
    }
    if (i2c_driver_install(i2c_port, I2C_MODE_MASTER,
                           I2C_MASTER_TX_BUF_DISABLE,
                           I2C_MASTER_RX_BUF_DISABLE, 0) != ESP_OK) {
        throw std::runtime_error("zio_motor: could not install i2c driver");
    }

    this->properties = ZioMotor::get_defaults();

    // --- PCA9685 initialisation ---
    // Software reset via general call address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x00, true); // general call address
    i2c_master_write_byte(cmd, 0x06, true); // SWRST
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(this->i2c_port, cmd, 10 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    // Wake: clear SLEEP bit (bit4), set ALLCALL (bit0)
    this->write_register(PCA9685_MODE1, 0x01);

    // Set PWM frequency via prescaler.
    // Formula: prescale = round(25MHz / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)(roundf(25000000.0f / (4096.0f * pwm_frequency)) - 1);
    // Must be in SLEEP mode to write prescaler
    this->write_register(PCA9685_MODE1, 0x11); // SLEEP + ALLCALL
    this->write_register(PCA9685_PRESCALE, prescale);
    // Wake back up
    this->write_register(PCA9685_MODE1, 0x01);
    // Allow oscillator to stabilise (500 µs min per datasheet)
    vTaskDelay(1); // 1 tick is ≥1ms on ESP-IDF, sufficient

    // Set RESTART bit to re-enable PWM after frequency change
    this->write_register(PCA9685_MODE1, 0x81); // RESTART + ALLCALL

    // Ensure all motors are stopped
    for (int i = 0; i < 4; i++) {
        this->apply_speed(i, 0.0f);
    }
}

void ZioMotor::write_register(uint8_t reg, uint8_t value) const {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (this->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(this->i2c_port, cmd, 100 / portTICK_PERIOD_MS) != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        throw std::runtime_error("zio_motor: i2c write failed");
    }
    i2c_cmd_link_delete(cmd);
}

void ZioMotor::set_pwm_channel(uint8_t channel, uint16_t on, uint16_t off) const {
    uint8_t base = PCA9685_LED0_ON_L + 4 * channel;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (this->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, base, true);
    i2c_master_write_byte(cmd, on & 0xFF, true);
    i2c_master_write_byte(cmd, (on >> 8) & 0x1F, true);
    i2c_master_write_byte(cmd, off & 0xFF, true);
    i2c_master_write_byte(cmd, (off >> 8) & 0x1F, true);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(this->i2c_port, cmd, 100 / portTICK_PERIOD_MS) != ESP_OK) {
        i2c_cmd_link_delete(cmd);
        throw std::runtime_error("zio_motor: i2c pwm write failed");
    }
    i2c_cmd_link_delete(cmd);
}

// Set a digital output channel permanently HIGH (ON bit 4 = full-on)
void ZioMotor::set_channel_full_on(uint8_t channel) const {
    this->set_pwm_channel(channel, 0x1000, 0x0000); // LEDn_ON bit12 set = always on
}

// Set a digital output channel permanently LOW
void ZioMotor::set_channel_full_off(uint8_t channel) const {
    this->set_pwm_channel(channel, 0x0000, 0x1000); // LEDn_OFF bit12 set = always off
}

void ZioMotor::apply_speed(uint8_t motor_index, float speed) const {
    uint8_t in1_ch = MOTOR_CHANNELS[motor_index][0];
    uint8_t in2_ch = MOTOR_CHANNELS[motor_index][1];
    uint8_t pwm_ch = MOTOR_CHANNELS[motor_index][2];

    // Clamp speed to [-1.0, 1.0]
    if (speed > 1.0f) speed = 1.0f;
    if (speed < -1.0f) speed = -1.0f;

    uint16_t duty = (uint16_t)(fabsf(speed) * 4095.0f);

    if (speed == 0.0f) {
        // Coast: both IN pins LOW, PWM irrelevant
        this->set_channel_full_off(in1_ch);
        this->set_channel_full_off(in2_ch);
        this->set_pwm_channel(pwm_ch, 0, 0);
    } else if (speed > 0.0f) {
        // Forward: IN1=HIGH, IN2=LOW, PWM=duty
        this->set_channel_full_on(in1_ch);
        this->set_channel_full_off(in2_ch);
        this->set_pwm_channel(pwm_ch, 0, duty);
    } else {
        // Reverse: IN1=LOW, IN2=HIGH, PWM=duty
        this->set_channel_full_off(in1_ch);
        this->set_channel_full_on(in2_ch);
        this->set_pwm_channel(pwm_ch, 0, duty);
    }
}

void ZioMotor::apply_brake(uint8_t motor_index) const {
    uint8_t in1_ch = MOTOR_CHANNELS[motor_index][0];
    uint8_t in2_ch = MOTOR_CHANNELS[motor_index][1];
    uint8_t pwm_ch = MOTOR_CHANNELS[motor_index][2];

    // Short brake: IN1=HIGH, IN2=HIGH, PWM=full
    this->set_channel_full_on(in1_ch);
    this->set_channel_full_on(in2_ch);
    this->set_channel_full_on(pwm_ch);
}

void ZioMotor::step() {
    for (int i = 0; i < 4; i++) {
        const std::string brake_key = "m" + std::to_string(i + 1) + "_brake";
        const std::string speed_key = "m" + std::to_string(i + 1) + "_speed";

        if (this->properties.at(brake_key)->boolean_value) {
            this->apply_brake(i);
        } else {
            this->apply_speed(i, (float)this->properties.at(speed_key)->number_value);
        }
    }
    Module::step();
}

void ZioMotor::call(const std::string method_name,
                    const std::vector<ConstExpression_ptr> arguments) {
    if (method_name == "speed") {
        // speed(motor_num, value)  motor_num: 1-4, value: -1.0 to 1.0
        Module::expect(arguments, 2, integer, numbery);
        int motor = (int)arguments[0]->evaluate_integer();
        if (motor < 1 || motor > 4) {
            throw std::runtime_error("zio_motor: motor number must be 1-4");
        }
        float spd = (float)arguments[1]->evaluate_number();
        const std::string key = "m" + std::to_string(motor) + "_speed";
        this->properties.at(key)->number_value = spd;
        this->apply_speed(motor - 1, spd);
    } else if (method_name == "brake") {
        // brake(motor_num)  motor_num: 1-4
        Module::expect(arguments, 1, integer);
        int motor = (int)arguments[0]->evaluate_integer();
        if (motor < 1 || motor > 4) {
            throw std::runtime_error("zio_motor: motor number must be 1-4");
        }
        const std::string key = "m" + std::to_string(motor) + "_brake";
        this->properties.at(key)->boolean_value = true;
        this->apply_brake(motor - 1);
    } else if (method_name == "release") {
        // release(motor_num)  clears brake flag and coasts
        Module::expect(arguments, 1, integer);
        int motor = (int)arguments[0]->evaluate_integer();
        if (motor < 1 || motor > 4) {
            throw std::runtime_error("zio_motor: motor number must be 1-4");
        }
        const std::string brake_key = "m" + std::to_string(motor) + "_brake";
        const std::string speed_key = "m" + std::to_string(motor) + "_speed";
        this->properties.at(brake_key)->boolean_value = false;
        this->properties.at(speed_key)->number_value = 0.0;
        this->apply_speed(motor - 1, 0.0f);
    } else if (method_name == "stop_all") {
        // stop_all()  coast all motors and clear all brake flags
        Module::expect(arguments, 0);
        for (int i = 0; i < 4; i++) {
            const std::string brake_key = "m" + std::to_string(i + 1) + "_brake";
            const std::string speed_key = "m" + std::to_string(i + 1) + "_speed";
            this->properties.at(brake_key)->boolean_value = false;
            this->properties.at(speed_key)->number_value = 0.0;
            this->apply_speed(i, 0.0f);
        }
    } else {
        Module::call(method_name, arguments);
    }
}
