#pragma once

#include <cstdint>
#include <string>

// LINAK Techline CAN SAE J1939 linear actuator driver
// One instance per actuator, send_command() must be called every <=200ms
// Set test_mode=true to print register values instead of writing to CAN
class LinakActuator
{
public:
    // actuator_addr: J1939 node address (set via hardware address pins, default 0xC8)
    // src_addr: our (Jetson) address
    explicit LinakActuator(const std::string& can_iface,
                           uint8_t actuator_addr = 0xC8,
                           uint8_t src_addr      = 0x01,
                           bool    test_mode     = false);
    ~LinakActuator();

    void send_command();   // transmit Proprietary A register, call every <=200ms
    void poll_feedback();  // read Proprietary B frames, update position/error state

    void stop();
    void run_out();
    void run_in();
    void run_to_position(float position_mm);  // input in mm, converted to 0.1mm/bit
    void clear_errors();                       // called automatically on startup
    void set_speed(uint8_t speed_percent);     // 0-100%

    float   get_position_mm() const { return position_mm_; }
    uint8_t get_error_code()  const { return error_code_; }

private:
    int     socket_;
    uint8_t actuator_addr_;
    uint8_t src_addr_;
    bool    test_mode_;

    // Proprietary A register (8 bytes), modified via bitmasking
    // [0-1] position, [2] current, [3] speed, [4] ramp up, [5] ramp down, [6-7] 0xFF
    uint8_t reg_a_[8];

    float   position_mm_;
    uint8_t error_code_;

    void     set_position_field(uint16_t raw_pos);
    uint32_t command_can_id() const;
    void     open_socket(const std::string& iface);

    static const char* error_name(uint8_t code);
};
