#pragma once

#include <cstdint>
#include <string>

/*
 * LinakActuator — LINAK Techline CAN SAE J1939 linear actuator driver
 *
 * Controls one actuator per instance over SocketCAN.
 *
 * Protocol overview (datasheet pp. 29-32):
 *   Proprietary A (command, PGN 0xEF00, PDU1 peer-to-peer):
 *     Must be sent every ≤200 ms. Contains 8 bytes:
 *       [0-1] Position UINT16  — 0.1 mm/bit for position; special codes for mode
 *       [2]   Current  UINT8   — 0.25 A/bit; 0xFB = default
 *       [3]   Speed    UINT8   — 0.5 %/bit;  0xFB = default
 *       [4]   RampUp   UINT8   — 0.05 s/bit; 0xFB = default
 *       [5]   RampDown UINT8   — 0.05 s/bit; 0xFB = default
 *       [6-7] Reserved         — must be 0xFF
 *
 *   Proprietary B (feedback, PGN 0xEF00, broadcast every 100 ms):
 *       [0-1] Position UINT16  — 0.1 mm/bit
 *       [3]   Status   UINT8   — bit flags (running, overcurrent, etc.)
 *       [4]   Error    UINT8   — active error code (0 = none)
 *
 * CAN ID (29-bit J1939 extended):
 *   Command  : 0x18EF{actuator_addr}{src_addr}
 *   Feedback : 0x18EFFF{actuator_addr}  (broadcast from actuator)
 *
 * Usage:
 *   LinakActuator act("can0", 0xC8);   // actuator at address 200
 *   act.set_speed(75);
 *   act.run_out();
 *   // In a ≤200 ms ROS timer:
 *   act.send_command();
 *   act.poll_feedback();
 *
 * Test mode (no CAN hardware required):
 *   LinakActuator act("can0", 0xC8, 0x01, true);
 *   Prints register contents instead of writing to the bus.
 */
class LinakActuator
{
public:
    /* can_iface     : SocketCAN interface name, e.g. "can0"
     * actuator_addr : J1939 node address of the actuator (128–247, default 200 / 0xC8)
     * src_addr      : Our (Jetson) J1939 source address, default 0x01
     * test_mode     : If true, replaces all CAN I/O with print statements */
    explicit LinakActuator(const std::string& can_iface,
                           uint8_t actuator_addr = 0xC8,
                           uint8_t src_addr      = 0x01,
                           bool    test_mode     = false);
    ~LinakActuator();

    /* Transmit the current Proprietary A register to the actuator.
     * Call from a ≤200 ms timer — the actuator stops if it misses 250 ms. */
    void send_command();

    /* Drain pending Proprietary B frames, update position and error state.
     * Prints to stderr when the error code changes. */
    void poll_feedback();

    // --- Motion commands ---
    // Each method modifies only its own byte(s) in reg_a_ via bitmasking;
    // the remaining bytes (speed, ramp, current) are left untouched.

    void stop();
    void run_out();
    void run_in();

    /* Move to an absolute position. Input is in mm; converted to 0.1 mm/bit internally.
     * Example: run_to_position(150.0f) → raw = 1500 = 0x05DC */
    void run_to_position(float position_mm);

    /* Send a Clear Error Code command (position field = 0xFB00).
     * Called automatically in the constructor before first use. */
    void clear_errors();

    /* Set motor speed. Input: 0–100 percent.
     * Encoding: 0.5 %/bit → reg byte = (percent << 1), stored in reg_a_[3] only. */
    void set_speed(uint8_t speed_percent);

    // --- Feedback accessors (updated by poll_feedback) ---
    float   get_position_mm() const { return position_mm_; }
    uint8_t get_error_code()  const { return error_code_; }

private:
    int     socket_;
    uint8_t actuator_addr_;
    uint8_t src_addr_;
    bool    test_mode_;

    /* Proprietary A register (8 bytes), stored in hex.
     * All modifications use bit operations so only the targeted field changes. */
    uint8_t reg_a_[8];

    float   position_mm_;   // last actuator position decoded from Proprietary B
    uint8_t error_code_;    // last error code from Proprietary B byte 4 (0 = none)

    /* Write a UINT16 into reg_a_[0] (LSB) and reg_a_[1] (MSB). */
    void set_position_field(uint16_t raw_pos);

    /* Returns the 29-bit J1939 extended CAN ID for Proprietary A (with EFF flag). */
    uint32_t command_can_id() const;

    void open_socket(const std::string& iface);

    /* Human-readable name for a Proprietary B error code. */
    static const char* error_name(uint8_t code);
};
