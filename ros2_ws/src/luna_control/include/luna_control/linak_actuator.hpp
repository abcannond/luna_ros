#ifndef LINAK_ACTUATOR_HPP
#define LINAK_ACTUATOR_HPP
#pragma once

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

class LinakActuator
{
public:
  explicit LinakActuator(std::string can_interface);
  ~LinakActuator();

  LinakActuator(const LinakActuator &) = delete;
  LinakActuator & operator=(const LinakActuator &) = delete;

  void tick_200ms();
  void stop(std::uint32_t can_id);
  void set_speed(std::uint32_t can_id, float speed_percent); //speed_percent is rounded to closest 0.5%
  void run_in(std::uint32_t can_id);
  void run_out(std::uint32_t can_id);
  void run_to_position_mm(std::uint32_t can_id, double position_mm);
  void clear_error_codes(std::uint32_t can_id);
  double current_position_mm() const;

private:
  bool send_ext_frame(std::uint32_t can_id29, const std::array<std::uint8_t, 8> & data);
  void heartbeat_loop_();

  std::string can_interface_;
  int sock_{-1};
  std::mutex reg_mutex_;
  std::uint32_t target_ext_can_id_{0};
  bool has_can_id_{false};
  std::array<std::uint8_t, 8> reg_a_shadow_{};
  std::atomic<bool> heartbeat_running_{false};
  std::thread heartbeat_thread_;
};

#endif
