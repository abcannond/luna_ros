#include "luna_linaks/LinakActuator.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// =============================================================================
// J1939 CAN ID constants
// =============================================================================

// Priority=6 (bits 28-26), Reserved=0, Data Page=0, PF=0xEF (bits 23-16)
// Full command ID = J1939_BASE | (actuator_addr << 8) | src_addr
static constexpr uint32_t J1939_BASE = (6u << 26) | (0xEFu << 16);  // 0x18EF0000

// =============================================================================
// Proprietary A position-field command codes (UINT16, datasheet p.29)
// =============================================================================
static constexpr uint16_t POS_CLEAR_ERROR = 0xFB00;  // 64256 — clear error codes
static constexpr uint16_t POS_RUN_OUT     = 0xFB01;  // 64257 — run outward continuously
static constexpr uint16_t POS_RUN_IN      = 0xFB02;  // 64258 — run inward continuously
static constexpr uint16_t POS_STOP        = 0xFB03;  // 64259 — stop

// 0xFB in a parameter byte means "use the default configured in Actuator Connect"
static constexpr uint8_t  USE_DEFAULT = 0xFB;

// =============================================================================
// Constructor / Destructor
// =============================================================================

LinakActuator::LinakActuator(const std::string& can_iface,
                              uint8_t actuator_addr,
                              uint8_t src_addr,
                              bool    test_mode)
  : socket_(-1)
  , actuator_addr_(actuator_addr)
  , src_addr_(src_addr)
  , test_mode_(test_mode)
  , position_mm_(0.0f)
  , error_code_(0)
{
    // --- Initialise reg_a_ to a safe stopped state ---
    // All parameter bytes set to USE_DEFAULT so the actuator uses its own configured values.
    // Only the position field (bytes 0-1) carries the "Stop" command.
    set_position_field(POS_STOP);
    reg_a_[2] = USE_DEFAULT;  // current limit  (0.25 A/bit; 0xFB = actuator default)
    reg_a_[3] = USE_DEFAULT;  // speed          (0.5 %/bit;  0xFB = actuator default)
    reg_a_[4] = USE_DEFAULT;  // ramp-up time   (0.05 s/bit; 0xFB = actuator default)
    reg_a_[5] = USE_DEFAULT;  // ramp-down time (0.05 s/bit; 0xFB = actuator default)
    reg_a_[6] = 0xFF;         // reserved — must always be 0xFF
    reg_a_[7] = 0xFF;         // reserved — must always be 0xFF

    open_socket(can_iface);

    // Datasheet requirement: clear any latched errors before first motion command
    clear_errors();
}

LinakActuator::~LinakActuator()
{
    if (!test_mode_ && socket_ >= 0)
        close(socket_);
}

// =============================================================================
// CAN socket setup
// =============================================================================

void LinakActuator::open_socket(const std::string& iface)
{
    if (test_mode_) {
        printf("[LINAK TEST] addr=0x%02X  Opened on %s (test mode — no CAN hardware)\n",
               actuator_addr_, iface.c_str());
        return;
    }

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0)
        throw std::runtime_error("LinakActuator: failed to open CAN socket");

    struct ifreq ifr{};
    strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
        throw std::runtime_error("LinakActuator: interface not found: " + iface);

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("LinakActuator: bind failed on " + iface);

    // Filter: accept Proprietary B frames from our actuator only.
    // Checks: EFF (extended), priority bits 28-26, PF=0xEF (bits 23-16), SA=actuator_addr (bits 7-0).
    // The destination byte (bits 15-8) is not masked — we accept both broadcast and addressed replies.
    struct can_filter filt{};
    filt.can_id   = CAN_EFF_FLAG | J1939_BASE | static_cast<uint32_t>(actuator_addr_);
    filt.can_mask = CAN_EFF_FLAG | 0x18FF00FFu;
    setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt));

    // Non-blocking so poll_feedback() drains the buffer without stalling the ROS timer
    int fl = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, fl | O_NONBLOCK);
}

// =============================================================================
// Internal helpers
// =============================================================================

// Write a 16-bit value into reg_a_[0-1] using bitmasking (little-endian: LSB first)
void LinakActuator::set_position_field(uint16_t raw_pos)
{
    reg_a_[0] = static_cast<uint8_t>( raw_pos        & 0x00FFu);  // LSB
    reg_a_[1] = static_cast<uint8_t>((raw_pos >> 8u) & 0x00FFu);  // MSB
}

// Builds the 29-bit J1939 CAN ID for Proprietary A (peer-to-peer to actuator)
uint32_t LinakActuator::command_can_id() const
{
    // PDU1: PS byte (bits 15-8) = destination address (actuator)
    //       SA  byte (bits  7-0) = source address (us)
    return CAN_EFF_FLAG
           | J1939_BASE
           | (static_cast<uint32_t>(actuator_addr_) << 8u)
           | static_cast<uint32_t>(src_addr_);
}

// =============================================================================
// Transmit / Receive
// =============================================================================

void LinakActuator::send_command()
{
    if (test_mode_) {
        // Print the full register so callers can verify correct encoding
        printf("[LINAK TEST] send addr=0x%02X  "
               "pos=[%02X %02X] cur=%02X spd=%02X ru=%02X rd=%02X rsv=[%02X %02X]\n",
               actuator_addr_,
               reg_a_[0], reg_a_[1], reg_a_[2], reg_a_[3],
               reg_a_[4], reg_a_[5], reg_a_[6], reg_a_[7]);
        return;
    }

    struct can_frame frame{};
    frame.can_id  = command_can_id();
    frame.can_dlc = 8;
    memcpy(frame.data, reg_a_, 8);

    if (write(socket_, &frame, sizeof(frame)) < 0)
        perror("LinakActuator: send_command");
}

void LinakActuator::poll_feedback()
{
    if (test_mode_) return;  // no CAN in test mode — nothing to drain

    struct can_frame frame{};
    // Read all available Proprietary B frames (non-blocking loop exits on EAGAIN)
    while (read(socket_, &frame, sizeof(frame)) > 0) {
        if (frame.can_dlc < 5) continue;

        // Bytes 0-1: actuator position UINT16, resolution 0.1 mm/bit
        // Values above 0xFAFF are reserved (position lost, etc.) — ignore them
        uint16_t raw_pos = static_cast<uint16_t>(frame.data[0])
                         | (static_cast<uint16_t>(frame.data[1]) << 8u);
        if (raw_pos <= 0xFAFFu)
            position_mm_ = static_cast<float>(raw_pos) / 10.0f;

        // Byte 4: active error code (highest-priority error only, 0 = none)
        uint8_t new_err = frame.data[4];
        if (new_err != error_code_) {
            error_code_ = new_err;
            if (new_err != 0)
                fprintf(stderr, "[LINAK 0x%02X] Error %u: %s\n",
                        actuator_addr_, new_err, error_name(new_err));
            else
                fprintf(stderr, "[LINAK 0x%02X] Error cleared\n", actuator_addr_);
        }
    }
}

// =============================================================================
// Motion commands
// =============================================================================

// Each method touches only its own byte(s) in reg_a_ via bitmasking.
// Fields not listed (speed, ramp, current) are always preserved.

void LinakActuator::stop()
{
    set_position_field(POS_STOP);
}

void LinakActuator::run_out()
{
    set_position_field(POS_RUN_OUT);
}

void LinakActuator::run_in()
{
    set_position_field(POS_RUN_IN);
}

void LinakActuator::run_to_position(float position_mm)
{
    // Convert mm to raw register value: 1 bit = 0.1 mm → raw = mm * 10
    // e.g. 150.0 mm → raw = 1500 = 0x05DC
    uint16_t raw = static_cast<uint16_t>(position_mm * 10.0f);
    if (raw > 0xFAFFu) raw = 0xFAFFu;  // clamp to valid position range
    set_position_field(raw);
}

void LinakActuator::clear_errors()
{
    // Send a single Clear Error frame (position field = 0xFB00), then revert to Stop.
    // The send-then-revert ensures the next periodic send_command() resumes Stop state.
    set_position_field(POS_CLEAR_ERROR);
    send_command();
    set_position_field(POS_STOP);
}

void LinakActuator::set_speed(uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;

    // Encode: 0.5 %/bit → reg_value = percent * 2 = percent << 1
    // 0 % → 0x00,  100 % → 200 = 0xC8 (valid range 0x00–0xC8)
    // Only byte 3 is modified; all other reg_a_ bytes are preserved.
    reg_a_[3] = static_cast<uint8_t>((speed_percent << 1u) & 0xFFu);
}

// =============================================================================
// Error code lookup table (Proprietary B byte 4, datasheet pp. 31-32)
// =============================================================================

const char* LinakActuator::error_name(uint8_t code)
{
    static const char* const table[] = {
        /* 0x00 */ "No error",
        /* 0x01 */ "Position sensor",
        /* 0x02 */ "Overvoltage",
        /* 0x03 */ "Undervoltage",
        /* 0x04 */ "Communication sync",
        /* 0x05 */ "Endstop switch",
        /* 0x06 */ "Power on block state",
        /* 0x07 */ "Temperature",
        /* 0x08 */ "Motor controller",
        /* 0x09 */ "Internal power supply",
        /* 0x0A */ "Internal current measurement",
        /* 0x0B */ "Parallel arbitration",
        /* 0x0C */ "Position not changing",
        /* 0x0D */ "Position initialisation failed",
        /* 0x0E */ "Alone in parallel system",
        /* 0x0F */ "Incorrect number in parallel system",
        /* 0x10 */ "Hardware",
        /* 0x11 */ "BLDC motor",
        /* 0x12 */ "Parallel communication",
        /* 0x13 */ "Parallel running",
        /* 0x14 */ "Parallel setup stopped",
    };
    constexpr uint8_t TABLE_LEN = sizeof(table) / sizeof(table[0]);

    if (code < TABLE_LEN) return table[code];
    if (code == 0xFE)     return "Other internal error";
    if (code == 0xFF)     return "Other external error";
    return "Unknown error";
}
