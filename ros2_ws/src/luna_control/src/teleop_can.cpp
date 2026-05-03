#include "luna_control/teleop_can.hpp"

#include <cstring>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace luna_control::teleop_can {

namespace {

constexpr std::uint32_t kId800 = 0x18EF8000U;
constexpr std::uint32_t kIdC800 = 0x18EFC800U;
constexpr std::uint32_t kIdF600 = 0x18EFF600U;

constexpr std::array<std::uint8_t, 8> kPay03 =
    {0x03, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};
constexpr std::array<std::uint8_t, 8> kPay01 =
    {0x01, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};
constexpr std::array<std::uint8_t, 8> kPay02 =
    {0x02, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};

}  // namespace

TeleopCan::~TeleopCan() { close(); }

bool TeleopCan::open(const char* iface) {
    close();
    if (!iface || iface[0] == '\0') {
        return false;
    }

    const int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("teleop_can socket(PF_CAN)");
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("teleop_can SIOCGIFINDEX");
        ::close(s);
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("teleop_can bind(CAN)");
        ::close(s);
        return false;
    }

    sock_ = s;
    std::cout << "teleop_can: opened " << iface << "\n";
    return true;
}

void TeleopCan::close() {
    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
}

bool TeleopCan::send_ext_frame(std::uint32_t id29, const std::array<std::uint8_t, 8>& data) {
    if (sock_ < 0) {
        return false;
    }

    struct can_frame fr {};
    fr.can_id = id29 | CAN_EFF_FLAG;
    fr.can_dlc = 8;
    std::memcpy(fr.data, data.data(), 8);

    const ssize_t n = write(sock_, &fr, sizeof(fr));
    if (n != static_cast<ssize_t>(sizeof(fr))) {
        perror("teleop_can write");
        return false;
    }
    return true;
}

bool TeleopCan::move_linkages_up() {
    if (!send_ext_frame(kId800, kPay01)) return false;
    if (!send_ext_frame(kIdC800, kPay03)) return false;
    if (!send_ext_frame(kIdF600, kPay03)) return false;
    return true;
}

bool TeleopCan::move_linkages_down() {
    if (!send_ext_frame(kId800, kPay02)) return false;
    if (!send_ext_frame(kIdC800, kPay03)) return false;
    if (!send_ext_frame(kIdF600, kPay03)) return false;
    return true;
}

}  // namespace luna_control::teleop_can
