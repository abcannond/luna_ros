/*
 * Hardware interface for CAN control of C620 and SparkMax motors
 * 
 * Created for 2025-26 WPI Lunabotics MQP 
 * 
 * Author: John Larochelle 
 */

#include "luna_control/LunaCan.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace luna_can {

//get all parameters from URDF on init
CallbackReturn LunaCan::on_init(const hardware_interface::HardwareInfo & info) {

    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    //CAN interface
    can_interface_ = info_.hardware_parameters.count("can_interface")
        ? info_.hardware_parameters.at("can_interface")
        : "can1";

    //explode if not enough joints are found 
    if (info_.joints.size() != NUM_JOINTS) {
        RCLCPP_FATAL(
        rclcpp::get_logger("LunaCan"),
        "Expected %zu joints, got %zu. Check URDF <ros2_control> block.",
        NUM_JOINTS, info_.joints.size());
        return CallbackReturn::ERROR;
    }

    //find joint parameters for wheels
    //Note: order is left_front_wheel, right_front_wheel, left_back_wheel, right_back_wheel
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        const auto & j = info_.joints[i];

        //explode if CAN ID is missing
        if (j.parameters.count("can_id") == 0) {
        RCLCPP_FATAL(rclcpp::get_logger("LunaCan"),
            "Joint '%s' missing 'can_id' parameter.", j.name.c_str());
        return CallbackReturn::ERROR;
        }

        //get CAN ids
        wheel_can_ids_[i] = std::stoi(j.parameters.at("can_id"));
        
        //get reverse flags
        wheel_reverse_flags_[i] = j.parameters.count("reversed")
        ? (j.parameters.at("reversed") == "true")
        : false;

        //explode if there are multiple interfaces
        //make sure you're not launching sim and hardware at the same time...
        if (j.command_interfaces.size() != 1 ||
            j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
        RCLCPP_FATAL(rclcpp::get_logger("LunaCan"),
            "Wheel joint '%s' must have exactly one velocity command interface.", j.name.c_str());
        return CallbackReturn::ERROR;
        }
    }

    //find joint parameters for wheel pods
    //Note: order is left_front_pod,   right_front_pod,   left_back_pod,   right_back_pod
    for (std::size_t i = 0; i < NUM_PODS; ++i) {
        const auto & j = info_.joints[NUM_WHEELS + i];

        //explode if CAN id is missing 
        if (j.parameters.count("can_id") == 0) {
        RCLCPP_FATAL(rclcpp::get_logger("LunaCan"),
            "Joint '%s' missing 'can_id' parameter.", j.name.c_str());
        return CallbackReturn::ERROR;
        }

        //get CAN ids
        pod_can_ids_[i] = std::stoi(j.parameters.at("can_id"));

        //explode if there are multiple interfaces 
        if (j.command_interfaces.size() != 1 ||
            j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
        RCLCPP_FATAL(rclcpp::get_logger("LunaCan"),
            "Pod joint '%s' must have exactly one position command interface.", j.name.c_str());
        return CallbackReturn::ERROR;
        }
    }

    //initialize everything to 0
    wheel_cmd_vel_.fill(0.0);
    wheel_state_vel_.fill(0.0);
    pod_cmd_pos_.fill(0.0);
    pod_state_pos_.fill(0.0);
    current_ramp_.fill(0);

    RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "on_init OK  (CAN: %s)", can_interface_.c_str());
    return CallbackReturn::SUCCESS;
    }

//set up CAN socket and SparkMax motors on config
CallbackReturn LunaCan::on_configure(const rclcpp_lifecycle::State &)
{
  //explode if we can't open CAN connection
  if (!open_can()) {
    return CallbackReturn::ERROR;
  }

  //set a short receive timeout on feedback_loop()
  struct timeval tv{};
  tv.tv_sec  = 0;
  tv.tv_usec = 50000;
  setsockopt(can_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  //try to make SparkMax objects 
  try {
    for (std::size_t i = 0; i < NUM_PODS; ++i) {
      spark_[i] = std::make_unique<SparkMax>(can_interface_.c_str(), pod_can_ids_[i]);
      spark_[i]->SetDataPortConfig(1); //alt encoder mode
      spark_[i]->SetAltEncoderCountsPerRev(8192); //REV through bore v2
      spark_[i]->SetAltEncoderPositionFactor(2.0f * (float)M_PI); //convert to rads
      spark_[i]->SetAltEncoderInverted(false);
    }
    pod_reverse_flags_[0] = true;
    pod_reverse_flags_[1] = false;
    pod_reverse_flags_[2] = false;
    pod_reverse_flags_[3] = true;

  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("LunaCan"),
      "SparkMax init failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "on_configure OK");
  return CallbackReturn::SUCCESS;
}

//start motor feedback and heartbeat threads on activate
CallbackReturn LunaCan::on_activate(const rclcpp_lifecycle::State &)
{
  feedback_running_ = true;
  feedback_thread_  = std::thread(&LunaCan::feedback_loop, this);
  // Arm C620s by sending zero current for 500ms
  current_ramp_.fill(0);
  auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
      send_C620_frame();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "C620s armed");

  send_running_ = true;
  send_thread_ = std::thread(&LunaCan::send_loop, this);

  RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "on_activate OK — feedback thread started");
  return CallbackReturn::SUCCESS;
}

//stop feedback thread on deactivate and zero outputs on deactivate
CallbackReturn LunaCan::on_deactivate(const rclcpp_lifecycle::State &)
{
  feedback_running_ = false;
  if (feedback_thread_.joinable()) {
    feedback_thread_.join();
  }

  send_running_ = false;
  if (send_thread_.joinable()) {
      send_thread_.join();
  }

  zero_all_outputs();

  RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "on_deactivate OK");
  return CallbackReturn::SUCCESS;
}

//close CAN socket and get rid of SparkMax objects on cleanup
CallbackReturn LunaCan::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (auto & s : spark_) { s.reset(); }
  close_can();
  RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "on_cleanup OK");
  return CallbackReturn::SUCCESS;
}

//ramp current down on shutdown to save our motors
CallbackReturn LunaCan::on_shutdown(const rclcpp_lifecycle::State &)
{
  bool ramping = true;
  constexpr int RAMP_STEP = 100; //decrease in 100 mA increments 
  while (ramping) {
    ramping = false;
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      if (current_ramp_[i] > 0) {
        current_ramp_[i] = std::max(0, current_ramp_[i] - RAMP_STEP);
        ramping = true;
      } else if (current_ramp_[i] < 0) {
        current_ramp_[i] = std::min(0, current_ramp_[i] + RAMP_STEP);
        ramping = true;
      }
    }
    send_C620_frame();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  
  for (auto & s : spark_) { s.reset(); } //reset SparkMax motors
  close_can();
  return CallbackReturn::SUCCESS;
}

//export state interfaces for Luna Controller
std::vector<hardware_interface::StateInterface> LunaCan::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &wheel_state_vel_[i]);
  }

  for (std::size_t i = 0; i < NUM_PODS; ++i) {
    state_interfaces.emplace_back(
      info_.joints[NUM_WHEELS + i].name,
      hardware_interface::HW_IF_POSITION,
      &pod_state_pos_[i]);
  }

  return state_interfaces;
}

//export command interfaces for Luna Controller 
std::vector<hardware_interface::CommandInterface> LunaCan::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &wheel_cmd_vel_[i]);
  }

  for (std::size_t i = 0; i < NUM_PODS; ++i) {
    command_interfaces.emplace_back(
      info_.joints[NUM_WHEELS + i].name,
      hardware_interface::HW_IF_POSITION,
      &pod_cmd_pos_[i]);
  }

  return command_interfaces;
}

//read latest feedback from motors
hardware_interface::return_type LunaCan::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //use C620 feedback obtained by feedback thread
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    wheel_state_vel_[i] = c620_fb_[i].speed;
  }

  //get pos data from sparkmax motors
  for (std::size_t i = 0; i < NUM_PODS; ++i) {
    pod_state_pos_[i] = spark_[i]->GetAltEncoderPosition();
  }

  return return_type::OK;
}

//write commands to both drive and swerve motors
hardware_interface::return_type LunaCan::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    //Drive motor control loop 
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
        double target_vel = wheel_cmd_vel_[i];

        //apply reverse flags 
        if (wheel_reverse_flags_[i]) { target_vel = -target_vel; }

        //double vel_error = target_vel - wheel_state_vel_[i];

        double max_cmd = 5.0; //temp parameter since LunaController is being weird
        double max_cur = 10000.0; //see above
      
        current_ramp_[i] = std::clamp(static_cast<int>((target_vel/max_cmd) * max_cur), -C620_MAX_CURRENT, C620_MAX_CURRENT);

        if(current_ramp_[i] >= 2000){
          //RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "current_ramp_[%zu]: %d", i, current_ramp_[i]);
        }
        //
        //TODO: finish controller
    }

    //SparkMax motor control loop
    for (std::size_t i = 0; i < NUM_PODS; ++i) {
      //TODO: make position control work
      //spark_[i]->Heartbeat();

      double target_pos = pod_cmd_pos_[i];
      RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "pod_cmd_pos_[%zu]: %f", i, pod_cmd_pos_[i]);
      double pos_error = target_pos - pod_state_pos_[i];
      double duty = 0.0;
      if((pos_error > 0.1) or (pos_error < -0.1)) {
        duty = SPARK_kP * pos_error; 
    
        duty = std::clamp(duty, -0.1, 0.1);

        //apply reversed 
        if(pod_reverse_flags_[i] == true) {
          duty = -duty; 
        }
      }
      //protect data with mutex
      std::lock_guard<std::mutex> lock(spark_mutex);
      pod_duty_cycle_[i] = duty;
      RCLCPP_INFO(rclcpp::get_logger("LunaCan"), "pod_duty_cycle[%zu]: %f", i, pod_duty_cycle_[i]);
    }

    return return_type::OK;
}

//feedback loop to get C620 motor data 
void LunaCan::feedback_loop()
{
  while (feedback_running_) {
    can_frame fr{};
    int n = static_cast<int>(::read(can_sock_, &fr, sizeof(fr)));
    if (n <= 0) {
      continue;
    }

    //C620 feedback frames: 0x201(front right), 0x202(front left), 0x203(back left), 0x204(back right)
    int raw_id = static_cast<int>(fr.can_id) - C620_FB_ID;
    if (raw_id < 1 || raw_id > 4) {
      continue;
    }

    // Find which wheel index this CAN ID belongs to
    for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
      if (wheel_can_ids_[i] == raw_id) {
        C620_Feedback fb{};
        fb.angle = (fr.data[0] << 8) | fr.data[1];

        //RPM is signed int16, bytes 2-3
        int16_t rpm_raw = static_cast<int16_t>((fr.data[2] << 8) | fr.data[3]);
        //Convert RPM → rad/s
        fb.speed = static_cast<double>(rpm_raw) * (2.0 * M_PI / 60.0);

        if (wheel_reverse_flags_[i]) { fb.speed = -fb.speed; }

        fb.current  = static_cast<int16_t>((fr.data[4] << 8) | fr.data[5]);
        
        //std::lock_guard<std::mutex> lock(can_mutex_);
        c620_fb_[i] = fb;
        break;
      }
    }
  }
}

//periodically send heartbeat signal to motors
void LunaCan::send_loop()
{
  while (send_running_) {
    //send C620 frame
    //std::lock_guard<std::mutex> lock(can_mutex_);
    send_C620_frame();
    
    //send SparkMax duty cycle 
    for (std::size_t i = 0; i < NUM_PODS; ++i) {
      if (spark_[i]) {
        spark_[i]->Heartbeat();
        std::lock_guard<std::mutex> lock(spark_mutex);
        //spark_[i]->SetDutyCycle((float)pod_duty_cycle_[i]);
        if(i == 0) {
          spark_[i]->SetDutyCycle((float)pod_duty_cycle_[0]);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::microseconds(2000)); // 50Hz
  }
}

//send drive commands to C620s
void LunaCan::send_C620_frame()
{
  //std::lock_guard<std::mutex> lock(can_mutex_);
  can_frame fr{};
  fr.can_id  = C620_WRITE_ID;
  fr.can_dlc = 8;

  //Frame layout: [motor1_H, motor1_L, motor2_H, motor2_L, motor3_H, motor3_L, motor4_H, motor4_L]
  //wheel_can_ids_ holds the CAN IDs (1–4); the frame byte position = (id-1)*2
  for (std::size_t i = 0; i < NUM_WHEELS; ++i) {
    int slot = wheel_can_ids_[i] - 1;
    if (slot < 0 || slot > 3) { continue; }
    int val = current_ramp_[i];
    fr.data[slot * 2]     = static_cast<uint8_t>((val >> 8) & 0xFF);
    fr.data[slot * 2 + 1] = static_cast<uint8_t>(val & 0xFF);
  }

  //warn if can write gets messed up 
  if (::write(can_sock_, &fr, sizeof(fr)) != sizeof(fr)) {
    RCLCPP_ERROR(rclcpp::get_logger("LunaCan"),
        "CAN write failed: %s", strerror(errno));
  }
}

//stop everything
void LunaCan::zero_all_outputs()
{
  current_ramp_.fill(0);
  send_C620_frame();

  for (std::size_t i = 0; i < NUM_PODS; ++i) {
    if (spark_[i]) {
      spark_[i]->Heartbeat();
      spark_[i]->SetDutyCycle(0.0f);
    }
  }
}

bool LunaCan::open_can()
{
  can_sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_sock_ < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("LunaCan"), "socket(PF_CAN) failed: %s", strerror(errno));
    return false;
  }

  struct ifreq ifr{};
  strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("LunaCan"),
      "CAN interface '%s' not found or down: %s", can_interface_.c_str(), strerror(errno));
    ::close(can_sock_);
    can_sock_ = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family   = AF_CAN;
  addr.can_ifindex  = ifr.ifr_ifindex;
  if (::bind(can_sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(rclcpp::get_logger("LunaCan"), "bind(CAN) failed: %s", strerror(errno));
    ::close(can_sock_);
    can_sock_ = -1;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("LunaCan"),
    "CAN socket opened on %s", can_interface_.c_str());
  return true;
}

void LunaCan::close_can()
{
  if (can_sock_ >= 0) {
    ::close(can_sock_);
    can_sock_ = -1;
  }
}

}   //namespace luna_can

PLUGINLIB_EXPORT_CLASS(luna_can::LunaCan, hardware_interface::SystemInterface)