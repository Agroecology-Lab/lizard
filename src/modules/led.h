#pragma once

#include <string>
#include <vector>

#include "../compilation/argument.h"
#include "module.h"

class Led : public Module
{
private:
    gpio_num_t number;

public:
    Led(gpio_num_t number);
    void call(std::string method, std::vector<Argument *> arguments);
};
