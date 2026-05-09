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

// J1939 29-bit CAN ID base: priority=6, PF=0xEF (Proprietary A)
// full ID = J1939_BASE | (actuator_addr << 8) | src_addr
static constexpr uint32_t J1939_BASE = (6u << 26) | (0xEFu << 16);

// Proprietary A position field command codes (datasheet p.29)
static constexpr uint16_t POS_CLEAR_ERROR = 0xFB00;
static constexpr uint16_t POS_RUN_OUT     = 0xFB01;
static constexpr uint16_t POS_RUN_IN      = 0xFB02;
static constexpr uint16_t POS_STOP        = 0xFB03;

// 0xFB in any parameter byte = use default configured in Actuator Connect
static constexpr uint8_t USE_DEFAULT = 0xFB;

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
    set_position_field(POS_STOP);
    reg_a_[2] = USE_DEFAULT;  // current limit
    reg_a_[3] = USE_DEFAULT;  // speed
    reg_a_[4] = USE_DEFAULT;  // ramp up
    reg_a_[5] = USE_DEFAULT;  // ramp down
    reg_a_[6] = 0xFF;         // reserved
    reg_a_[7] = 0xFF;         // reserved

    open_socket(can_iface);
    clear_errors();  // required before first motion command per datasheet
}

LinakActuator::~LinakActuator()
{
    if (!test_mode_ && socket_ >= 0)
        close(socket_);
}

void LinakActuator::open_socket(const std::string& iface)
{
    if (test_mode_) {
        printf("[LINAK TEST] addr=0x%02X opened on %s\n", actuator_addr_, iface.c_str());
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

    // filter to receive Proprietary B feedback frames (PF=0xFF) from this actuator's SA
    // command uses PF=0xEF (Proprietary A); feedback uses PF=0xFF (Proprietary B)
    static constexpr uint32_t J1939_FEEDBACK_BASE = (6u << 26) | (0xFFu << 16);
    struct can_filter filt{};
    filt.can_id   = CAN_EFF_FLAG | J1939_FEEDBACK_BASE | static_cast<uint32_t>(actuator_addr_);
    filt.can_mask = CAN_EFF_FLAG | 0x1CFF00FFu;  // check EFF, priority, PF, SA; ignore GE
    setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt));

    // non-blocking so poll_feedback() doesn't stall
    int fl = fcntl(socket_, F_GETFL, 0);
    fcntl(socket_, F_SETFL, fl | O_NONBLOCK);
}

// writes 16-bit value into bytes 0-1, little-endian
void LinakActuator::set_position_field(uint16_t raw_pos)
{
    reg_a_[0] = static_cast<uint8_t>( raw_pos        & 0x00FFu);
    reg_a_[1] = static_cast<uint8_t>((raw_pos >> 8u) & 0x00FFu);
}

uint32_t LinakActuator::command_can_id() const
{
    return CAN_EFF_FLAG
           | J1939_BASE
           | (static_cast<uint32_t>(actuator_addr_) << 8u)
           | static_cast<uint32_t>(src_addr_);
}

void LinakActuator::send_command()
{
    if (test_mode_) {
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
    if (test_mode_) return;

    struct can_frame frame{};
    while (read(socket_, &frame, sizeof(frame)) > 0) {
        if (frame.can_dlc < 5) continue;

        // bytes 0-1: position (0.1mm/bit), ignore reserved values above 0xFAFF
        uint16_t raw_pos = static_cast<uint16_t>(frame.data[0])
                         | (static_cast<uint16_t>(frame.data[1]) << 8u);
        if (raw_pos <= 0xFAFFu)
            position_mm_ = static_cast<float>(raw_pos) / 10.0f;

        // byte 4: error code
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

// these only update the position field, speed/ramp/current bytes are untouched
void LinakActuator::stop()    { set_position_field(POS_STOP); }
void LinakActuator::run_out() { set_position_field(POS_RUN_OUT); }
void LinakActuator::run_in()  { set_position_field(POS_RUN_IN); }

void LinakActuator::run_to_position(float position_mm)
{
    // 0.1mm/bit -> raw = mm * 10
    uint16_t raw = static_cast<uint16_t>(position_mm * 10.0f);
    if (raw > 0xFAFFu) raw = 0xFAFFu;
    set_position_field(raw);
}

void LinakActuator::clear_errors()
{
    // send clear error then immediately revert to stop so the next periodic send is safe
    set_position_field(POS_CLEAR_ERROR);
    send_command();
    set_position_field(POS_STOP);
}

void LinakActuator::set_speed(uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;
    // 0.5%/bit encoding: value = percent << 1  (100% -> 0xC8)
    reg_a_[3] = static_cast<uint8_t>((speed_percent << 1u) & 0xFFu);
}

// error codes from Proprietary B byte 4 (datasheet pp.31-32)
const char* LinakActuator::error_name(uint8_t code)
{
    static const char* const table[] = {
        "No error", "Position sensor", "Overvoltage", "Undervoltage",
        "Communication sync", "Endstop switch", "Power on block state",
        "Temperature", "Motor controller", "Internal power supply",
        "Internal current measurement", "Parallel arbitration",
        "Position not changing", "Position initialisation failed",
        "Alone in parallel system", "Incorrect number in parallel system",
        "Hardware", "BLDC motor", "Parallel communication",
        "Parallel running", "Parallel setup stopped",
    };
    constexpr uint8_t TABLE_LEN = sizeof(table) / sizeof(table[0]);

    if (code < TABLE_LEN) return table[code];
    if (code == 0xFE)     return "Other internal error";
    if (code == 0xFF)     return "Other external error";
    return "Unknown error";
}
