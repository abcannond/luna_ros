#pragma once 

/*
 * Hardware interface for CAN control of C620 and SparkMax motors
 * 
 * Created for 2025-26 WPI Lunabotics MQP 
 * 
 * Author: John Larochelle 
 */

#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "luna_control/SparkMax.hpp"
#include <mutex>

//CAN stuff 
#include <linux/can.h>
#include <linux/can/raw.h>

namespace luna_can {

    //basic parameters 
    constexpr std::size_t NUM_WHEELS = 4;
    constexpr std::size_t NUM_PODS   = 4;
    constexpr std::size_t NUM_JOINTS = NUM_WHEELS + NUM_PODS;

    //C620 parameters (drive motor speed controllers)
    constexpr int C620_WRITE_ID = 0x200; //base for all writes to C620s
    constexpr int C620_FB_ID = 0x200; //base for all reads from C620s 
    constexpr int C620_MAX_CURRENT = 16000; //max current in mA 
    constexpr double WHEEL_RADIUS_M = 0.1016; // wheel radius in m
    constexpr double C620_kP = 0.01; //TODO: tune this garbage 
    constexpr double C620_kI = 0.0; //TODO: see above 

    //SparkMax parameters (ported from swerve_debug after on-hardware tuning)
    constexpr double SPARK_kP = 1.0;
    constexpr double SPARK_kI = 0.2;
    constexpr double SPARK_kD = 0.0;
    constexpr double SPARK_OUTPUT_LIMIT = 0.3;   // duty cycle hitches above ~0.3 on real HW
    constexpr double SPARK_DEADBAND = 0.05;      // rad
    constexpr double SPARK_INTEGRAL_LIMIT = 0.04;

    //data read from C620s 
    struct C620_Feedback {
        double angle = 0.0;
        double speed = 0.0;
        double current = 0.0; 
    };

    class LunaCan : public hardware_interface::SystemInterface {

    public:
        
        //the shared pointers let the controller and interface share data 
        RCLCPP_SHARED_PTR_DEFINITIONS(LunaCan)

        //lifecycle stuff 
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State & previous_state) override;

        //set up interfaces
        std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        //CAN stuff 
        int can_sock_ = -1;
        std::string can_interface_;
        //std::mutex can_mutex_; //added this to try preventing corrupted CAN frames, not sure if necessary 
        std::mutex spark_mutex;

        bool open_can();
        void close_can();
        void send_C620_frame();
        void zero_all_outputs();
        
        std::thread feedback_thread_; //receives CAN messages for drive
        std::atomic<bool> feedback_running_{false};
        void feedback_loop();
        std::thread send_thread_; //sends CAN messages to the drive, providing a heartbeat
        std::atomic<bool> send_running_{false};
        void send_loop();
        
        //C620 data (drive motors)
        std::array<int,    NUM_WHEELS> wheel_can_ids_{};
        std::array<bool,   NUM_WHEELS> wheel_reverse_flags_{};
        std::array<double, NUM_WHEELS> wheel_cmd_vel_{};   //commands received from LunaController in rad/s
        std::array<double, NUM_WHEELS> wheel_state_vel_{}; //feedback sent to LunaController in rad/s
        std::array<C620_Feedback, NUM_WHEELS> c620_fb_{};   //data collected from drive motors 
        
        //SparkMax data (swerve motors) 
        std::array<std::unique_ptr<SparkMax>, NUM_PODS> spark_;
        std::array<int,    NUM_PODS> pod_can_ids_{}; 
        std::array<bool,   NUM_PODS> pod_reverse_flags_{};
        std::array<double, NUM_PODS> pod_cmd_pos_{}; //commands received from LunaController in rad
        std::array<double, NUM_PODS> pod_state_pos_{}; //feedback sent to LunaController in rad (offset + wrapped to [-pi, pi])
        std::array<double, NUM_PODS> pod_duty_cycle_{}; //current duty cycle of each pod
        std::array<double, NUM_PODS> pod_offset_{};      //captured at on_activate; subtracted from raw encoder
        std::array<double, NUM_PODS> pod_integral_{};    //PID integrator state per pod
        std::array<double, NUM_PODS> pod_last_error_{};  //PID derivative state per pod
        
        //ramp down on shutdown for safety 
        std::array<int, NUM_WHEELS> current_ramp_{};
    };

}   // namespace luna_can


