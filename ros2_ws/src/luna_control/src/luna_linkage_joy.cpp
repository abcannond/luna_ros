// Joy → linkage CAN (menu_loop cmd2/cmd3/cmd4). NOT_TESTED until hardware check.
// LT = up, LB = down, neither = stop, BOTH LT+LB = stop (neutral).

#include "luna_control/teleop_can.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace {

class LunaLinkageJoyNode : public rclcpp::Node {
public:
    LunaLinkageJoyNode() : Node("luna_linkage_joy") {
        declare_parameter<std::string>("can_interface", "can0");
        declare_parameter<int>("trigger_axis", 2);
        declare_parameter<double>("trigger_threshold", 0.25);
        declare_parameter<int>("bumper_button", 4);
        declare_parameter<double>("timer_period_sec", 0.05);

        can_iface_ = get_parameter("can_interface").as_string();
        trigger_axis_ =
            static_cast<int>(get_parameter("trigger_axis").as_int());
        trigger_threshold_ = get_parameter("trigger_threshold").as_double();
        bumper_button_ =
            static_cast<int>(get_parameter("bumper_button").as_int());
        const double period = get_parameter("timer_period_sec").as_double();

        if (!can_.open(can_iface_.c_str())) {
            RCLCPP_FATAL(get_logger(), "Failed to open CAN interface '%s'", can_iface_.c_str());
            throw std::runtime_error("teleop_can::open failed");
        }

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            rclcpp::QoS(10),
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(joy_mutex_);
                last_joy_ = *msg;
                have_joy_ = true;
            });

        const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(std::max(0.02, period)));
        timer_ = create_wall_timer(period_ns, [this]() { on_timer(); });

        RCLCPP_INFO(
            get_logger(),
            "luna_linkage_joy: CAN=%s LT=axis[%d] (|axis|>%.2f) LB=button[%d] both→stop",
            can_iface_.c_str(),
            trigger_axis_,
            trigger_threshold_,
            bumper_button_);
    }

private:
    static bool trigger_held(const sensor_msgs::msg::Joy& joy, int axis, double threshold) {
        if (axis < 0 || static_cast<std::size_t>(axis) >= joy.axes.size()) {
            return false;
        }
        return std::abs(static_cast<double>(joy.axes[static_cast<std::size_t>(axis)])) > threshold;
    }

    static bool bumper_held(const sensor_msgs::msg::Joy& joy, int btn) {
        if (btn < 0 || static_cast<std::size_t>(btn) >= joy.buttons.size()) {
            return false;
        }
        return joy.buttons[static_cast<std::size_t>(btn)] != 0;
    }

    void on_timer() {
        sensor_msgs::msg::Joy joy_copy;
        {
            std::lock_guard<std::mutex> lock(joy_mutex_);
            if (!have_joy_) {
                can_.move_linkages_stop();
                return;
            }
            joy_copy = last_joy_;
        }

        const bool lt = trigger_held(joy_copy, trigger_axis_, trigger_threshold_);
        const bool lb = bumper_held(joy_copy, bumper_button_);

        if (lt && lb) {
            if (!can_.move_linkages_stop()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "move_linkages_stop failed");
            }
            return;
        }
        if (lt) {
            if (!can_.move_linkages_up()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "move_linkages_up failed");
            }
            return;
        }
        if (lb) {
            if (!can_.move_linkages_down()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "move_linkages_down failed");
            }
            return;
        }
        if (!can_.move_linkages_stop()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "move_linkages_stop failed");
        }
    }

    luna_control::teleop_can::TeleopCan can_;
    std::string can_iface_;
    int trigger_axis_{2};
    double trigger_threshold_{0.25};
    int bumper_button_{4};

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex joy_mutex_;
    sensor_msgs::msg::Joy last_joy_;
    bool have_joy_{false};
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<LunaLinkageJoyNode>());
    } catch (const std::exception& e) {
        std::cerr << "luna_linkage_joy: " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
