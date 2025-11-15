#pragma once
#include <string>

class MotorInterface
{
public:
  MotorInterface(const std::string& joint_name);
  void set_velocity(double vel);
  double get_velocity();

private:
  std::string joint_name_;
  double current_velocity_;
};
