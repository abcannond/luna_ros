// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Enrique Fernández
 */

/*
 * This code has been adapted and/or modified for use in the 2024-25 WPI Lunabotics MQP.
 * Author: Sam Rooney
 */

#include "luna_control/odometry.hpp"

namespace luna_controller
{
  Odometry::Odometry(size_t velocity_rolling_window_size)
      : timestamp_(0.0),
        x_(0.0),
        y_(0.0),
        heading_(0.0),
        linear_(0.0),
        angular_(0.0),
        wheel_track_(0.0),
        wheel_base_(0.0),
        wheel_radius_(0.0),
        left_back_wheel_old_vel_(0.0),
        left_front_wheel_old_vel_(0.0),
        right_back_wheel_old_vel_(0.0),
        right_front_wheel_old_vel_(0.0),
        left_back_pod_old_pos_(0.0),
        left_front_pod_old_pos_(0.0),
        right_back_pod_old_pos_(0.0),
        right_front_pod_old_pos_(0.0),
        velocity_rolling_window_size_(velocity_rolling_window_size),
        linear_accumulator_(velocity_rolling_window_size),
        strafe_accumulator_(velocity_rolling_window_size),
        angular_accumulator_(velocity_rolling_window_size)
  {
  }

  void Odometry::init(const rclcpp::Time &time)
  {
    // Reset accumulators and timestamp:
    resetAccumulators();
    timestamp_ = time;
  }

  bool Odometry::update(double left_back_pod_pos, double left_front_pod_pos, double right_back_pod_pos, double right_front_pod_pos, double left_back_wheel_vel, double left_front_wheel_vel, double right_back_wheel_vel, double right_front_wheel_vel, const rclcpp::Time &time)
  {
    // We cannot estimate the speed with very small time intervals:
    const double dt = time.seconds() - timestamp_.seconds();
    if (dt < 0.0001)
    {
      return false; // Interval too small to integrate with
    }

    // Estimate the robot's velocity

    // Wheel positions relative to robot center
    const double x_fl =  wheel_base_ / 2.0;
    const double y_fl =  wheel_track_ / 2.0;
    const double x_fr =  wheel_base_ / 2.0;
    const double y_fr = -wheel_track_ / 2.0;
    const double x_bl = -wheel_base_ / 2.0;
    const double y_bl =  wheel_track_ / 2.0;
    const double x_br = -wheel_base_ / 2.0;
    const double y_br = -wheel_track_ / 2.0;

    // Wheel velocity vectors (robot frame)
    const double v_fl_x = left_front_wheel_vel  * wheel_radius_ * cos(left_front_pod_pos);
    const double v_fl_y = left_front_wheel_vel  * wheel_radius_ * sin(left_front_pod_pos);

    const double v_fr_x = right_front_wheel_vel * wheel_radius_ * cos(right_front_pod_pos);
    const double v_fr_y = right_front_wheel_vel * wheel_radius_ * sin(right_front_pod_pos);

    const double v_bl_x = left_back_wheel_vel   * wheel_radius_ * cos(left_back_pod_pos);
    const double v_bl_y = left_back_wheel_vel   * wheel_radius_ * sin(left_back_pod_pos);

    const double v_br_x = right_back_wheel_vel  * wheel_radius_ * cos(right_back_pod_pos);
    const double v_br_y = right_back_wheel_vel  * wheel_radius_ * sin(right_back_pod_pos);

    // Average linear velocity
    const double vx = (v_fl_x + v_fr_x + v_bl_x + v_br_x) / 4.0;
    const double vy = (v_fl_y + v_fr_y + v_bl_y + v_br_y) / 4.0;

    // Angular velocity from cross product
    const double r2 = pow(wheel_base_ / 2.0, 2) + pow(wheel_track_ / 2.0, 2);

    const double omega =
      ((x_fl * v_fl_y - y_fl * v_fl_x) +
      (x_fr * v_fr_y - y_fr * v_fr_x) +
      (x_bl * v_bl_y - y_bl * v_bl_x) +
      (x_br * v_br_y - y_br * v_br_x)) / (4.0 * r2);

    // Convert to deltas
    const double linear  = vx * dt;
    const double strafe  = vy * dt;
    const double angular = omega * dt;

    // Integrate
    integrateExact(linear, angular, strafe);

    // Update old position with current:
    left_back_wheel_old_vel_ = left_back_wheel_vel;
    left_front_wheel_old_vel_ = left_front_wheel_vel;
    right_back_wheel_old_vel_ = right_back_wheel_vel;
    right_front_wheel_old_vel_ = right_front_wheel_vel;
    left_back_pod_old_pos_ = left_back_pod_pos;
    left_front_pod_old_pos_ = left_front_pod_pos;
    right_back_pod_old_pos_ = right_back_pod_pos;
    right_front_pod_old_pos_ = right_front_pod_pos;

    timestamp_ = time;

    // Estimate speeds using a rolling mean to filter them out:
    linear_accumulator_.accumulate(linear / dt);
    strafe_accumulator_.accumulate(strafe / dt);
    angular_accumulator_.accumulate(angular / dt);

    linear_ = linear_accumulator_.getRollingMean();
    strafe_ = strafe_accumulator_.getRollingMean();
    angular_ = angular_accumulator_.getRollingMean();

    return true;
  }

  void Odometry::updateOpenLoop(double linear, double angular, double strafe, const rclcpp::Time &time)
  {
    /// Save last linear and angular velocity:
    linear_ = linear;
    strafe_ = strafe;
    angular_ = angular;

    /// Integrate odometry:
    const double dt = time.seconds() - timestamp_.seconds();
    timestamp_ = time;
    integrateExact(linear * dt, angular * dt, strafe * dt);
  }

  void Odometry::resetOdometry()
  {
    x_ = 0.0;
    y_ = 0.0;
    heading_ = 0.0;
  }

  void Odometry::setWheelParams(double wheel_track, double wheel_base, double wheel_radius)
  {
    wheel_track_ = wheel_track;
    wheel_base_ = wheel_base;
    wheel_radius_ = wheel_radius;
  }

  void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
  {
    velocity_rolling_window_size_ = velocity_rolling_window_size;

    resetAccumulators();
  }

  void Odometry::integrateRungeKutta2(double linear, double angular, double strafe)
  {
    const double direction = heading_ + angular * 0.5;

    /// Runge-Kutta 2nd order integration
    // used when angular is near zero
    x_ += (linear * cos(direction)) - (strafe * cos(direction - (M_PI / 2)));
    y_ += (linear * sin(direction)) - (strafe * sin(direction - (M_PI / 2)));
    heading_ += angular;
  }

  void Odometry::integrateExact(double linear, double angular, double strafe)
  {
    // all params are deltas - NOT velocities
    if (fabs(angular) < 1e-6)
    {
      integrateRungeKutta2(linear, angular, strafe);
    }
    else
    {
      const double r_0 = linear / angular;
      const double r_1 = r_0 + strafe;
      const double icc_x = x_ - (r_0 * cos(heading_ - M_PI_2));
      const double icc_y = y_ - (r_0 * sin(heading_ - M_PI_2));

      heading_ += angular;
      x_ = icc_x + (r_1 * cos(heading_ - M_PI_2));
      y_ = icc_y + (r_1 * sin(heading_ - M_PI_2));
    }
  }

  void Odometry::resetAccumulators()
  {
    linear_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    strafe_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
    angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  }

} // namespace luna_control