#pragma once

#include "module.h"
#include "can.h"

class Rp2040Motor : public Module {
public:
    Rp2040Motor(const std::string& name, Can_ptr can, int64_t node_id);
    ~Rp2040Motor() override = default;

    void step() override;
    void call(const std::string& method_name, const std::vector<ConstExpression_ptr>& arguments) override;

private:
    Can_ptr can;
    int64_t node_id;

    std::shared_ptr<IntegerVariable> position_variable;
    std::shared_ptr<IntegerVariable> velocity_variable;
    std::shared_ptr<BooleanVariable> enabled_variable;
    std::shared_ptr<BooleanVariable> fault_variable;

    int32_t target_velocity = 0;
    bool target_enabled = false;
    uint32_t last_send_time = 0;
    uint32_t last_receive_time = 0;

    static constexpr uint8_t COMMAND_SEND_BASIC_UPDATE = 0x02;
    static constexpr uint8_t COMMAND_REQUEST_SENSORS = 0x03;
    static constexpr uint8_t COMMAND_SIMPLE_PING = 0x09;
    static constexpr uint8_t COMMAND_CLEAR_ERRORS = 0x0E;
    static constexpr uint8_t REPLY_FLAG = 0x80;

    void send_basic_update();
    void send_simple_ping();
    void send_clear_errors();
    void handle_sensor_reply(const CanMessage& message);
};
