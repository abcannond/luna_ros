#include "luna_control/linak_actuator.hpp"

#include <cstring>
#include <iostream>
#include <limits>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

//CALLED BY METHODS -> MSGs FOR BUS
namespace {
constexpr std::array<std::uint8_t, 8> kRunInPayload = {
    0x01, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};

constexpr std::array<std::uint8_t, 8> kRunOutPayload = {
    0x02, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};
}

// Opens a SocketCAN raw socket on the given iface (e.g. can0) and binds it so
// subsequent writes from this object are transmitted on that bus.
LinakActuator::LinakActuator(std::string can_interface) : can_interface_(std::move(can_interface))
{
  const int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    perror("LinakActuator socket(PF_CAN)");
    return;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    perror("LinakActuator SIOCGIFINDEX");
    close(s);
    return;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    perror("LinakActuator bind(CAN)");
    close(s);
    return;
  }

  sock_ = s;
  clear_error_codes();
}

LinakActuator::~LinakActuator()
{
  if (sock_ >= 0) {
    close(sock_);
    sock_ = -1;
  }
}
// Builds one extended (29-bit) CAN frame and write()s it; the kernel driver sends it on the wire.
bool LinakActuator::send_ext_frame(std::uint32_t can_id29, const std::array<std::uint8_t, 8> & data)
{
  if (sock_ < 0) {
    return false;
  }

  struct can_frame fr {};
  fr.can_id = can_id29 | CAN_EFF_FLAG;
  fr.can_dlc = 8;
  std::memcpy(fr.data, data.data(), 8);

  const ssize_t n = write(sock_, &fr, sizeof(fr));
  if (n != static_cast<ssize_t>(sizeof(fr))) {
    perror("LinakActuator write(CAN)");
    return false;
  }
  return true;
}

void LinakActuator::tick_200ms() {}

void LinakActuator::stop() {}

void LinakActuator::set_speed(float) {}

//MOVES LINAK IN
void LinakActuator::run_in(std::uint32_t can_id)
{
  if (!send_ext_frame(can_id, kRunInPayload)) {
    std::cerr << "LinakActuator::run_in: send failed\n";
  }
}

//MOVES LINAK OUT
void LinakActuator::run_out(std::uint32_t can_id)
{
  if (!send_ext_frame(can_id, kRunOutPayload)) {
    std::cerr << "LinakActuator::run_out: send failed\n";
  }
}

void LinakActuator::run_to_position_mm(double) {}

void LinakActuator::clear_error_codes() {}

double LinakActuator::current_position_mm() const
{
  return position_valid_ ? position_mm_ : std::numeric_limits<double>::quiet_NaN();
}
