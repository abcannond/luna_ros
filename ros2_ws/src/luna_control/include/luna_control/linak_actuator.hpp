#ifndef LINAK_ACTUATOR_HPP
#define LINAK_ACTUATOR_HPP
#pragma once

#include <array>
#include <cstdint>
#include <string>

class LinakActuator
{
public:
  explicit LinakActuator(std::string can_interface);
  ~LinakActuator();

  LinakActuator(const LinakActuator &) = delete;
  LinakActuator & operator=(const LinakActuator &) = delete;

  void tick_200ms();
  void stop(std::uint32_t can_id);
  void set_speed(float speed);
  void run_in(std::uint32_t can_id);
  void run_out(std::uint32_t can_id);
  void run_to_position_mm(double position_mm);
  void clear_error_codes(std::uint32_t can_id);
  double current_position_mm() const;

private:
  bool send_ext_frame(std::uint32_t can_id29, const std::array<std::uint8_t, 8> & data);

  std::string can_interface_;
  int sock_{-1};
  std::uint32_t reg_a_shadow_{0};
  double position_mm_{0.0};
  bool position_valid_{false};
};

#endif
