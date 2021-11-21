#include "button.h"

Button::Button(std::string name, gpio_num_t number) : Module(button, name)
{
    this->number = number;
    gpio_reset_pin(number);
    gpio_set_direction(number, GPIO_MODE_INPUT);
    this->properties["level"] = new IntegerVariable();
}

void Button::step()
{
    this->properties["level"]->integer_value = gpio_get_level(this->number);
}

void Button::call(std::string method_name, std::vector<Expression *> arguments)
{
    if (method_name == "get")
    {
        Module::expect(arguments, 0);
        printf("%s %d\n", this->name.c_str(), gpio_get_level(this->number));
    }
    else if (method_name == "pullup")
    {
        Module::expect(arguments, 0);
        gpio_set_pull_mode(this->number, GPIO_PULLUP_ONLY);
    }
    else if (method_name == "pulldown")
    {
        Module::expect(arguments, 0);
        gpio_set_pull_mode(this->number, GPIO_PULLDOWN_ONLY);
    }
    else
    {
        Module::call(method_name, arguments);
    }
}

std::string Button::get_output()
{
    char buffer[256];
    std::sprintf(buffer, "%d", gpio_get_level(this->number));
    return buffer;
}
