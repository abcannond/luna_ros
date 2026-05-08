#include "luna_control/linak_actuator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
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

constexpr std::array<std::uint8_t, 8> kStopPayload = {
    0x03, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};

constexpr std::array<std::uint8_t, 8> kClearErrorPayload = {
    0x00, 0xFB, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};

constexpr std::array<std::uint8_t, 8> kSpeedPayloadTemplate = {
    0xFF, 0xFF, 0xFB, 0x00, 0xFB, 0xFB, 0xFF, 0xFF};

constexpr std::array<std::uint8_t, 8> kPositionPayloadTemplate = {
    0x00, 0x00, 0xFB, 0xFB, 0xFB, 0xFB, 0xFF, 0xFF};

constexpr auto kHeartbeatPeriod = std::chrono::milliseconds{200};

constexpr std::size_t kSpeedByteIndex = 3;
constexpr std::size_t kPositionHighByteIndex = 0;
constexpr std::size_t kPositionLowByteIndex = 1;
}

static std::uint8_t speed_percent_to_raw(float speed_percent);
static std::uint16_t position_mm_to_raw(double position_mm);

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

  reg_a_shadow_ = kStopPayload;
  heartbeat_running_.store(true);
  heartbeat_thread_ = std::thread(&LinakActuator::heartbeat_loop_, this);
}

LinakActuator::~LinakActuator()
{
  heartbeat_running_.store(false);
  if (heartbeat_thread_.joinable()) {
    heartbeat_thread_.join();
  }

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
/*
//HEARTBEAT
*/
void LinakActuator::heartbeat_loop_()
{
  using clock = std::chrono::steady_clock;
  auto next = clock::now() + kHeartbeatPeriod;

  while (heartbeat_running_.load()) {
    std::this_thread::sleep_until(next);
    next += kHeartbeatPeriod;

    std::uint32_t ext_can_id_to_send = 0;
    bool has_can_id = false;
    std::array<std::uint8_t, 8> payload{};
    {
      std::lock_guard<std::mutex> lock(reg_mutex_);
      ext_can_id_to_send = target_ext_can_id_;
      has_can_id = has_can_id_;
      payload = reg_a_shadow_;
    }

    if (!has_can_id) {
      continue;
    }
    (void)send_ext_frame(ext_can_id_to_send, payload);
  }
}

// STOPS LINAK
void LinakActuator::stop(std::uint32_t can_id)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  target_ext_can_id_ = can_id;
  has_can_id_ = true;
  reg_a_shadow_ = kStopPayload;
}
//SETS SPEED OF LINAK
void LinakActuator::set_speed(std::uint32_t can_id, float speed_percent)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  target_ext_can_id_ = can_id;
  has_can_id_ = true;
  reg_a_shadow_[kSpeedByteIndex] = speed_percent_to_raw(speed_percent);
}
static std::uint8_t speed_percent_to_raw(float speed_percent)
{
  constexpr int kSpeedRawMax = 200;
  const float clamped = std::clamp(speed_percent, 0.0f, 100.0f);
  const long rounded = std::lround(clamped * 2.0f);
  const long raw = std::clamp(rounded, 0L, static_cast<long>(kSpeedRawMax));
  return static_cast<std::uint8_t>(raw);
}
//MOVES LINAK IN
void LinakActuator::run_in(std::uint32_t can_id)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  target_ext_can_id_ = can_id;
  has_can_id_ = true;
  reg_a_shadow_ = kRunInPayload;
}
//MOVES LINAK OUT
void LinakActuator::run_out(std::uint32_t can_id)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  target_ext_can_id_ = can_id;
  has_can_id_ = true;
  reg_a_shadow_ = kRunOutPayload;
}
//MOVES LINAK TO POSITION
void LinakActuator::run_to_position_mm(std::uint32_t can_id, double position_mm)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  target_ext_can_id_ = can_id;
  has_can_id_ = true;

  reg_a_shadow_ = kPositionPayloadTemplate;
  const std::uint16_t raw = position_mm_to_raw(position_mm);
  reg_a_shadow_[kPositionHighByteIndex] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
  reg_a_shadow_[kPositionLowByteIndex] = static_cast<std::uint8_t>(raw & 0xFF);
}
static std::uint16_t position_mm_to_raw(double position_mm)
{
  constexpr int kPositionRawMax = 0xFAFF;
  const double max_mm = static_cast<double>(kPositionRawMax) * 0.1;
  const double clamped = std::clamp(position_mm, 0.0, max_mm);
  const long rounded = std::lround(clamped * 10.0);
  const long raw = std::clamp(rounded, 0L, static_cast<long>(kPositionRawMax));
  return static_cast<std::uint16_t>(raw);
}
//CLEAR ERRORS
void LinakActuator::clear_error_codes(std::uint32_t can_id)
{
  std::lock_guard<std::mutex> lock(reg_mutex_);
  target_ext_can_id_ = can_id;
  has_can_id_ = true;
  reg_a_shadow_ = kClearErrorPayload;
}
//GETS POSITION OF LINAK
double LinakActuator::current_position_mm() const
{
  return std::numeric_limits<double>::quiet_NaN();
}
