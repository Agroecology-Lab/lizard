#include "button.h"

Button::Button(std::string name, gpio_num_t number)
{
    this->name = name;
    this->number = number;
    gpio_reset_pin(number);
    gpio_set_direction(number, GPIO_MODE_INPUT);
}

void Button::call(std::string method, std::vector<Argument *> arguments)
{
    if (method == "get")
    {
        if (arguments.size() != 0)
        {
            printf("error: expecting no arguments for method \"%s.%s\"\n", this->name.c_str(), method.c_str());
            return;
        }
        printf("%s %d\n", this->name.c_str(), gpio_get_level(this->number));
    }
    else if (method == "pullup")
    {
        if (arguments.size() != 0)
        {
            printf("error: expecting no arguments for method \"%s.%s\"\n", this->name.c_str(), method.c_str());
            return;
        }
        gpio_set_pull_mode(this->number, GPIO_PULLUP_ONLY);
    }
    else if (method == "pulldown")
    {
        if (arguments.size() != 0)
        {
            printf("error: expecting no arguments for method \"%s.%s\"\n", this->name.c_str(), method.c_str());
            return;
        }
        gpio_set_pull_mode(this->number, GPIO_PULLDOWN_ONLY);
    }
    else
    {
        Module::call(method, arguments);
    }
}

double Button::get(std::string property_name)
{
    if (property_name == "level")
    {
        return gpio_get_level(this->number);
    }
    else
    {
        return Module::get(property_name);
    }
}

std::string Button::get_output()
{
    char buffer[256];
    std::sprintf(buffer, "%d", gpio_get_level(this->number));
    return buffer;
}
