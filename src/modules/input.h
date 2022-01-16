#pragma once

#include "driver/gpio.h"
#include "module.h"

class Input : public Module {
private:
    const gpio_num_t number;

public:
    Input(const std::string name, const gpio_num_t number);
    void step();
    void call(const std::string method_name, const std::vector<Expression_ptr> arguments);
    std::string get_output() const;
};
