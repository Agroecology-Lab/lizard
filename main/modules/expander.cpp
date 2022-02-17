#include "expander.h"

#include "utils/echo.h"
#include "utils/timing.h"
#include <cstring>

void strip(char *buffer, int len) {
    buffer[len] = 0;
    if (buffer[len - 1] == 10) {
        buffer[len - 1] = 0;
        if (buffer[len - 2] == 13) {
            buffer[len - 2] = 0;
        }
    }
}

Expander::Expander(const std::string name, const ConstSerial_ptr serial, const gpio_num_t boot_pin, const gpio_num_t enable_pin)
    : Module(expander, name), serial(serial), boot_pin(boot_pin), enable_pin(enable_pin) {
    serial->enable_line_detection();
    gpio_reset_pin(boot_pin);
    gpio_reset_pin(enable_pin);
    gpio_set_direction(boot_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(enable_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(boot_pin, 1);
    gpio_set_level(enable_pin, 0);
    delay(100);
    gpio_set_level(enable_pin, 1);
    char buffer[1024] = "";
    int len = 0;
    const unsigned long int start = millis();
    do {
        if (millis_since(start) > 1000) {
            throw std::runtime_error("expander is not booting");
        }
        if (serial->available()) {
            len = serial->read_line(buffer);
            strip(buffer, len);
            echo("%s: %s", name.c_str(), buffer);
        }
    } while (strncmp("Ready.", &buffer[1], 6));
}

void Expander::step() {
    static char buffer[1024];
    if (this->serial->available()) {
        int len = this->serial->read_line(buffer);
        strip(buffer, len);
        echo("%s: %s", this->name.c_str(), buffer);
    }
    Module::step();
}
