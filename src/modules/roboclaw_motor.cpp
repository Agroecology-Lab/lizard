#include "roboclaw_motor.h"

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

RoboClawMotor::RoboClawMotor(std::string name, RoboClaw *roboclaw, unsigned int motor_number) : Module(roboclaw_motor, name)
{
    this->motor_number = constrain(motor_number, 1, 2);
    if (this->motor_number != motor_number)
    {
        throw std::runtime_error("illegal motor number");
    }
    this->roboclaw = roboclaw;
    this->properties["encoder"] = new IntegerVariable();
}

void RoboClawMotor::step()
{
    uint8_t status;
    bool valid;
    int32_t encoder = this->motor_number == 1 ? this->roboclaw->ReadEncM1(&status, &valid) : this->roboclaw->ReadEncM2(&status, &valid);
    if (!valid)
    {
        throw std::runtime_error("could not read encoder value");
    }
    this->properties["encoder"]->integer_value = encoder;
}

void RoboClawMotor::call(std::string method_name, std::vector<Expression *> arguments)
{
    if (method_name == "power")
    {
        Module::expect(arguments, 1, numbery);
        unsigned short int duty = (short int)(constrain(arguments[0]->evaluate_number(), -1, 1) * 32767);
        this->motor_number == 1 ? this->roboclaw->DutyM1(duty) : this->roboclaw->DutyM2(duty);
    }
    else if (method_name == "speed")
    {
        Module::expect(arguments, 1, numbery);
        unsigned int counts_per_second = arguments[0]->evaluate_number();
        this->motor_number == 1 ? this->roboclaw->DutyM1(counts_per_second) : this->roboclaw->DutyM2(counts_per_second);
    }
    else if (method_name == "zero")
    {
        bool success = this->motor_number == 1 ? this->roboclaw->SetEncM1(0) : this->roboclaw->SetEncM2(0);
        if (!success)
        {
            throw std::runtime_error("could not reset encoder");
        }
    }
    else
    {
        Module::call(method_name, arguments);
    }
}
