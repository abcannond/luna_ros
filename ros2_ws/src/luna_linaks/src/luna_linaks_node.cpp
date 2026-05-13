#include "luna_linaks/LinakActuator.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

// Three actuators on the robot:
//   shoulder (0x80) – main arm that raises/lowers the bucket
//   wrist    (0xC8) – bucket wrist/tilt
//   lift     (0xF6) – scissor lift used for berm dump
//
// Each gets its own CAN socket and heartbeat (via the ROS timer) so all three
// can be commanded independently at the same time.
//
// Position control:
//   Publish a target on ~/shoulder/target_mm (Float32) to drive to that position.
//   Current position is published on ~/shoulder/position_mm (Float32) at command_rate_hz.
//   Manual extend/retract/stop services cancel any active position target.

struct ActuatorChannel {
    std::unique_ptr<LinakActuator> act;

    enum class Cmd { STOP, EXTEND, RETRACT, MOVE_TO } cmd = Cmd::STOP;
    float target_mm = std::numeric_limits<float>::quiet_NaN();
};

class LinaksNode : public rclcpp::Node
{
public:
    LinaksNode() : Node("luna_linaks_node")
    {
        declare_parameter("can_iface",        "can0");
        declare_parameter("src_addr",         0x01);
        declare_parameter("test_mode",        false);
        declare_parameter("speed_percent",    50);
        declare_parameter("command_rate_hz",  10.0);
        declare_parameter("pos_tolerance_mm", 2.0);

        const std::string iface = get_parameter("can_iface").as_string();
        const uint8_t     src   = static_cast<uint8_t>(get_parameter("src_addr").as_int());
        const bool        test  = get_parameter("test_mode").as_bool();
        const uint8_t     spd   = static_cast<uint8_t>(get_parameter("speed_percent").as_int());
        pos_tolerance_mm_ = static_cast<float>(get_parameter("pos_tolerance_mm").as_double());

        // shoulder=0x80, wrist=0xC8, lift=0xF6 (from menu_loop.sh CAN IDs)
        const uint8_t addrs[3] = {0x80, 0xC8, 0xF6};
        const char*   names[3] = {"shoulder", "wrist", "lift"};

        for (int i = 0; i < 3; ++i) {
            try {
                channels_[i].act = std::make_unique<LinakActuator>(iface, addrs[i], src, test);
                channels_[i].act->set_speed(spd);
                RCLCPP_INFO(get_logger(), "%s (0x%02X) opened on %s",
                            names[i], addrs[i], iface.c_str());
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "Failed to open %s: %s", names[i], e.what());
            }

            // per-actuator: extend / retract / stop (cancel any position target)
            make_service(std::string(names[i]) + "/extend",  i, ActuatorChannel::Cmd::EXTEND);
            make_service(std::string(names[i]) + "/retract", i, ActuatorChannel::Cmd::RETRACT);
            make_service(std::string(names[i]) + "/stop",    i, ActuatorChannel::Cmd::STOP);

            // per-actuator: move to absolute position (mm)
            const int cap_i = i;
            auto sub = create_subscription<std_msgs::msg::Float32>(
                std::string("~/") + names[i] + "/target_mm", 10,
                [this, cap_i](const std_msgs::msg::Float32::SharedPtr msg) {
                    auto & ch = channels_[cap_i];
                    ch.target_mm = msg->data;
                    ch.cmd = ActuatorChannel::Cmd::MOVE_TO;
                    if (ch.act) ch.act->run_to_position(msg->data);
                });
            target_subs_.push_back(sub);

            // per-actuator position publisher
            pos_pubs_[i] = create_publisher<std_msgs::msg::Float32>(
                std::string("~/") + names[i] + "/position_mm", 10);
        }

        stop_all_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/stop_all",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
                for (auto & ch : channels_) {
                    ch.cmd = ActuatorChannel::Cmd::STOP;
                    ch.target_mm = std::numeric_limits<float>::quiet_NaN();
                    if (ch.act) { ch.act->stop(); ch.act->send_command(); }
                }
                res->success = true;
                res->message = "all actuators stopped";
            });

        state_pub_ = create_publisher<std_msgs::msg::String>("~/state", 10);

        const double period_s = 1.0 / get_parameter("command_rate_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period_s),
            [this]() { tick(); });
    }

private:
    void make_service(const std::string & name, int idx, ActuatorChannel::Cmd cmd)
    {
        auto srv = create_service<std_srvs::srv::Trigger>(
            "~/" + name,
            [this, idx, cmd, name](
                const std_srvs::srv::Trigger::Request::SharedPtr,
                      std_srvs::srv::Trigger::Response::SharedPtr res)
            {
                auto & ch = channels_[idx];
                ch.cmd = cmd;
                ch.target_mm = std::numeric_limits<float>::quiet_NaN();  // cancel any move_to
                if (ch.act) {
                    switch (cmd) {
                        case ActuatorChannel::Cmd::EXTEND:  ch.act->run_out(); break;
                        case ActuatorChannel::Cmd::RETRACT: ch.act->run_in();  break;
                        case ActuatorChannel::Cmd::STOP:    ch.act->stop();    break;
                        default: break;
                    }
                    ch.act->send_command();
                }
                res->success = true;
                res->message = name + " started";
                //RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });
        services_.push_back(srv);
    }

    void tick()
    {
        static const char* cmd_names[] = {"STOP", "EXTEND", "RETRACT", "MOVE_TO"};
        std::string state_str;

        for (int i = 0; i < 3; ++i) {
            auto & ch = channels_[i];
            if (!ch.act) continue;

            ch.act->poll_feedback();

            if (ch.cmd == ActuatorChannel::Cmd::MOVE_TO) {
                float cur = ch.act->get_position_mm();
                // Only check position if we have a real reading (>0 from first feedback)
                if (cur > 0.0f && std::abs(cur - ch.target_mm) <= pos_tolerance_mm_) {
                    ch.act->stop();
                    ch.cmd = ActuatorChannel::Cmd::STOP;
                    ch.target_mm = std::numeric_limits<float>::quiet_NaN();
                    //RCLCPP_INFO(get_logger(), "actuator[%d] reached position %.1f mm", i, cur);
                } else {
                    ch.act->run_to_position(ch.target_mm);
                }
            } else {
                switch (ch.cmd) {
                    case ActuatorChannel::Cmd::EXTEND:  ch.act->run_out(); break;
                    case ActuatorChannel::Cmd::RETRACT: ch.act->run_in();  break;
                    case ActuatorChannel::Cmd::STOP:    ch.act->stop();    break;
                    default: break;
                }
            }
            ch.act->send_command();

            // publish current position
            std_msgs::msg::Float32 pos_msg;
            pos_msg.data = ch.act->get_position_mm();
            pos_pubs_[i]->publish(pos_msg);

            if (!state_str.empty()) state_str += ' ';
            state_str += cmd_names[static_cast<int>(ch.cmd)];
        }

        std_msgs::msg::String msg;
        msg.data = state_str;
        state_pub_->publish(msg);
    }

    ActuatorChannel channels_[3];  // [0]=shoulder, [1]=wrist, [2]=lift
    float pos_tolerance_mm_ = 2.0f;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pubs_[3];
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   stop_all_srv_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr>        services_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr>   target_subs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinaksNode>());
    rclcpp::shutdown();
    return 0;
}
