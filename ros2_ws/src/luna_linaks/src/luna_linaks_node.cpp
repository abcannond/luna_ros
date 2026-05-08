#include "luna_linaks/LinakActuator.hpp"

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

struct ActuatorChannel {
    std::unique_ptr<LinakActuator> act;

    // what the keep-alive loop should be sending right now
    enum class Cmd { STOP, EXTEND, RETRACT } cmd = Cmd::STOP;
};

class LinaksNode : public rclcpp::Node
{
public:
    LinaksNode() : Node("luna_linaks_node")
    {
        declare_parameter("can_iface",   "can0");
        declare_parameter("src_addr",    0x01);
        declare_parameter("test_mode",   false);
        declare_parameter("speed_percent", 50);
        declare_parameter("command_rate_hz", 10.0);

        const std::string iface = get_parameter("can_iface").as_string();
        const uint8_t     src   = static_cast<uint8_t>(get_parameter("src_addr").as_int());
        const bool        test  = get_parameter("test_mode").as_bool();
        const uint8_t     spd   = static_cast<uint8_t>(get_parameter("speed_percent").as_int());

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

            // per-actuator: extend / retract / stop
            make_service(std::string(names[i]) + "/extend",  i, ActuatorChannel::Cmd::EXTEND);
            make_service(std::string(names[i]) + "/retract", i, ActuatorChannel::Cmd::RETRACT);
            make_service(std::string(names[i]) + "/stop",    i, ActuatorChannel::Cmd::STOP);
        }

        stop_all_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/stop_all",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
                for (auto & ch : channels_) {
                    ch.cmd = ActuatorChannel::Cmd::STOP;
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
                if (ch.act) {
                    switch (cmd) {
                        case ActuatorChannel::Cmd::EXTEND:  ch.act->run_out(); break;
                        case ActuatorChannel::Cmd::RETRACT: ch.act->run_in();  break;
                        case ActuatorChannel::Cmd::STOP:    ch.act->stop();    break;
                    }
                    ch.act->send_command();
                }
                res->success = true;
                res->message = name + " started";
                RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
            });
        services_.push_back(srv);
    }

    void tick()
    {
        static const char* cmd_names[] = {"STOP", "EXTEND", "RETRACT"};
        std::string state_str;

        for (auto & ch : channels_) {
            if (!ch.act) continue;
            // keep-alive: resend current command so actuator doesn't timeout (>200ms = stop)
            switch (ch.cmd) {
                case ActuatorChannel::Cmd::EXTEND:  ch.act->run_out(); break;
                case ActuatorChannel::Cmd::RETRACT: ch.act->run_in();  break;
                case ActuatorChannel::Cmd::STOP:    ch.act->stop();    break;
            }
            ch.act->send_command();
            ch.act->poll_feedback();

            if (!state_str.empty()) state_str += ' ';
            state_str += cmd_names[static_cast<int>(ch.cmd)];
        }

        std_msgs::msg::String msg;
        msg.data = state_str;
        state_pub_->publish(msg);
    }

    ActuatorChannel channels_[3];  // [0]=shoulder, [1]=wrist, [2]=lift

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  stop_all_srv_;
    std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> services_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinaksNode>());
    rclcpp::shutdown();
    return 0;
}
