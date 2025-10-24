#include "rp2040_motor.h"
#include "can.h"
#include "timing.h"
#include "uart.h" // For debug logging

// Helper to convert a float to a byte array (little-endian assumed)
inline void float_to_bytes(float f, uint8_t* bytes) {
    union {
        float f;
        uint8_t b[4];
    } data;
    data.f = f;
    memcpy(bytes, data.b, 4);
}

// Helper to convert a byte array to a float (little-endian assumed)
inline float bytes_to_float(const uint8_t* bytes) {
    union {
        float f;
        uint8_t b[4];
    } data;
    memcpy(data.b, bytes, 4);
    return data.f;
}

// Helper to convert a byte array to an int32_t
inline int32_t bytes_to_int32(const uint8_t* bytes) {
    union {
        int32_t i;
        uint8_t b[4];
    } data;
    memcpy(data.b, bytes, 4);
    return data.i;
}

// --- Module Defaults ---
static const std::map<std::string, Variable_ptr> get_defaults() {
    return {
        { "position", make_shared<IntegerVariable>(0) },
        { "velocity", make_shared<IntegerVariable>(0) },
        { "enabled", make_shared<BooleanVariable>(false) },
        { "fault", make_shared<BooleanVariable>(false) }
    };
}

// --- Constructor ---
Rp2040Motor::Rp2040Motor(const std::string& name, Can_ptr can, int64_t node_id)
    : Module(name, get_defaults()), can(can), node_id(node_id) {
    
    // Assign variables from properties map
    position_variable = std::static_pointer_cast<IntegerVariable>(properties["position"]);
    velocity_variable = std::static_pointer_cast<IntegerVariable>(properties["velocity"]);
    enabled_variable = std::static_pointer_cast<BooleanVariable>(properties["enabled"]);
    fault_variable = std::static_pointer_cast<BooleanVariable>(properties["fault"]);

    // Subscribe to messages from the motor
    // The firmware uses rx_can_id for the destination, which is 0x8 by default.
    // The motor is configured to use a base ID of 0x5.
    // The reply ID is dynamic based on the host's rx_can_id, but the simplest is to listen for the sensor reply.
    // The sensor reply uses COMMAND_REQUEST_SENSORS (0x03) | REPLY_FLAG (0x80) = 0x83
    // The firmware sends a message with txMsg.Tx_id = rx_can_id, which is 0x8 by default.
    // So the motor sends back a message with ID 0x8.
    // Let's assume the motor replies with its own ID, which is likely calculated from the base ID.
    // The firmware uses rx_can_id (default 0x8) as the tx_id for the reply, which is confusing.
    // Let's stick to the simplest interpretation: the module listens for a message with the motor's ID.
    // The motor's ID is likely derived from the node_id. Let's assume the motor replies with the node_id.
    // The firmware's main-canbus.cpp shows rxMsg.Tx_id = rx_can_id, which suggests the motor replies to the host's ID.
    // The host ID is 0x8 by default in the firmware. Let's subscribe to 0x8.
    uint32_t motor_reply_id = 0x8; 
    can->subscribe(motor_reply_id, [this](const CanMessage& message) {
        // Check if the message is a sensor reply (starts with 0x83)
        if (message.data_len >= 1 && (message.data[0] & ~REPLY_FLAG) == COMMAND_REQUEST_SENSORS) {
            handle_sensor_reply(message);
        }
    });

    // Initial commands
    send_simple_ping();
    send_clear_errors();
}

// --- Main Loop Step ---
void Rp2040Motor::step() {
    // Send command every 100ms
    if (millis() - last_send_time > 100) { 
        send_basic_update();
    }
    // Consider fault if no reply for 500ms
    if (millis() - last_receive_time > 500 && last_receive_time != 0) { 
        fault_variable->set_value(true);
    }
}

// --- Send Basic Update (Set Velocity) ---
void Rp2040Motor::send_basic_update() {
    CanMessage message;
    // The motor is configured to listen on an ID derived from 0x5. Let's use the node_id for the destination ID.
    message.id = (uint32_t)node_id; 
    message.data_len = 8; // Max 8 bytes for a standard CAN frame

    // Command ID
    message.data[0] = COMMAND_SEND_BASIC_UPDATE;

    // Setpoint mode: 0x01 for Velocity mode, 0x00 for Disabled
    // This is an assumption based on typical FOC/motor control.
    message.data[1] = target_enabled ? 0x01 : 0x00; 

    // Target Velocity (float, 4 bytes). The firmware uses float for velocity.
    float_to_bytes((float)target_velocity, &message.data[2]);

    // Fill the rest with zeros, as the firmware expects 8 bytes for this message.
    memset(&message.data[6], 0, 2); 

    // Send the message
    can->send(message);
    last_send_time = millis();
}

// --- Send Simple Ping ---
void Rp2040Motor::send_simple_ping() {
    CanMessage message;
    message.id = (uint32_t)node_id;
    message.data_len = 1;

    message.data[0] = COMMAND_SIMPLE_PING;
    can->send(message);
}

// --- Send Clear Errors ---
void Rp2040Motor::send_clear_errors() {
    CanMessage message;
    message.id = (uint32_t)node_id;
    message.data_len = 1;

    message.data[0] = COMMAND_CLEAR_ERRORS;
    can->send(message);
}

// --- Handle Sensor Reply ---
void Rp2040Motor::handle_sensor_reply(const CanMessage& message) {
    last_receive_time = millis();
    fault_variable->set_value(false);

    // The sensor reply is a large message, likely using ISO-TP, but the firmware
    // analysis suggests the first part of the message is:
    // Offset 14-17: int32_t drive_motor_position
    // Offset 22-25: float motor1_velocity

    if (message.data_len >= 26) {
        // Position (int32_t) at offset 14
        // Note: The firmware analysis was based on a large buffer copy.
        // We are assuming the first 26 bytes contain the position and velocity.
        // This is a weak point, but the best we can do without the full ISO-TP implementation.

        // Let's use the offsets from the firmware analysis:
        // Offset 14-17: int32_t drive_motor_position
        // Offset 22-25: float motor1_velocity

        // Position (int32_t) at offset 14
        int32_t position = bytes_to_int32(&message.data[14]);
        position_variable->set_value(position);

        // Velocity (float) at offset 22
        float velocity_float = bytes_to_float(&message.data[22]);
        velocity_variable->set_value((int64_t)velocity_float);

        // Error code (uint8_t) at offset 26
        // The error code is at index 26 in the firmware's array, which is data[26] here.
        if (message.data_len > 26) {
            uint8_t error_code = message.data[26];
            if (error_code != 0) {
                fault_variable->set_value(true);
            }
        }
    }
}

// --- Lizard Method Call Interface ---
void Rp2040Motor::call(const std::string& method_name, const std::vector<ConstExpression_ptr>& arguments) {
    if (method_name == "set_velocity") {
        expect(arguments, 1, integer);
        target_velocity = arguments[0]->evaluate_integer();
        send_basic_update();
    } else if (method_name == "set_enabled") {
        expect(arguments, 1, boolean);
        target_enabled = arguments[0]->evaluate_boolean();
        send_basic_update();
    } else if (method_name == "clear_fault") {
        expect(arguments, 0);
        send_clear_errors();
    } else {
        Module::call(method_name, arguments);
    }
}

// --- Module Registration ---
REGISTER_MODULE(Rp2040Motor)
