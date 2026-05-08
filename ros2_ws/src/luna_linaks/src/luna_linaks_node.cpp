#include "luna_linaks/LinakActuator.hpp"

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class LinaksNode : public rclcpp::Node
{
public:
    LinaksNode() : Node("luna_linaks_node")
    {
        declare_parameter("can_iface",          "can0");
        declare_parameter("actuator_addr",      0xC8);
        declare_parameter("src_addr",           0x01);
        declare_parameter("test_mode",          false);
        declare_parameter("speed_percent",      50);
        declare_parameter("excavation_hold_s",  5.0);
        declare_parameter("dump_hold_s",        5.0);
        declare_parameter("command_rate_hz",    10.0);

        const std::string iface = get_parameter("can_iface").as_string();
        const uint8_t     act   = static_cast<uint8_t>(get_parameter("actuator_addr").as_int());
        const uint8_t     src   = static_cast<uint8_t>(get_parameter("src_addr").as_int());
        const bool        test  = get_parameter("test_mode").as_bool();
        speed_         = static_cast<uint8_t>(get_parameter("speed_percent").as_int());
        excavate_hold_ = get_parameter("excavation_hold_s").as_double();
        dump_hold_     = get_parameter("dump_hold_s").as_double();

        try {
            actuator_ = std::make_unique<LinakActuator>(iface, act, src, test);
            actuator_->set_speed(speed_);
            RCLCPP_INFO(get_logger(), "LinakActuator opened on %s addr=0x%02X test=%s",
                        iface.c_str(), act, test ? "true" : "false");
        } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Failed to open LinakActuator: %s", e.what());
        }

        pos_pub_   = create_publisher<std_msgs::msg::Float32>("~/position_mm",  10);
        err_pub_   = create_publisher<std_msgs::msg::UInt8>("~/error_code",     10);
        state_pub_ = create_publisher<std_msgs::msg::String>("~/state",         10);

        excavate_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/excavate",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
                if (!actuator_) { res->success = false; res->message = "actuator not open"; return; }
                start_sequence(true);
                res->success = true;
                res->message = "excavation sequence started";
            });

        dump_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/dump",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
                if (!actuator_) { res->success = false; res->message = "actuator not open"; return; }
                start_sequence(false);
                res->success = true;
                res->message = "dump sequence started";
            });

        stop_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/stop",
            [this](const std_srvs::srv::Trigger::Request::SharedPtr,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
                go_idle();
                res->success = true;
                res->message = "stopped";
            });

        const double period_s = 1.0 / get_parameter("command_rate_hz").as_double();
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period_s),
            [this]() { tick(); });
    }

private:
    enum class State { IDLE, EXTENDING, HOLDING, RETRACTING };

    void start_sequence(bool is_excavate)
    {
        is_excavate_ = is_excavate;
        state_ = State::EXTENDING;
        actuator_->set_speed(speed_);
        actuator_->run_out();
        actuator_->send_command();
        RCLCPP_INFO(get_logger(), "%s: EXTENDING", is_excavate_ ? "excavate" : "dump");
    }

    void go_idle()
    {
        if (actuator_) {
            actuator_->stop();
            actuator_->send_command();
        }
        state_ = State::IDLE;
        RCLCPP_INFO(get_logger(), "IDLE (stopped)");
    }

    void tick()
    {
        if (!actuator_) return;

        actuator_->poll_feedback();

        switch (state_) {
            case State::IDLE:
                // keep-alive: re-send stop so actuator doesn't time out
                actuator_->stop();
                actuator_->send_command();
                break;

            case State::EXTENDING:
                actuator_->run_out();
                actuator_->send_command();
                // Transition to HOLDING when actuator reports it has stopped moving.
                // Use position stabilization: if position hasn't changed for one tick,
                // assume end-of-travel (or when poll_feedback reports an endstop).
                // For simplicity, transition on error code 5 (endstop) or after checking
                // that position is stable. We use error_code as the primary signal;
                // the dwell timer in the FSM is the outer safety timeout.
                if (actuator_->get_error_code() == 5 /* endstop */ ||
                    actuator_->get_error_code() == 12 /* position not changing */) {
                    RCLCPP_INFO(get_logger(), "%s: reached end, HOLDING for %.1fs",
                                is_excavate_ ? "excavate" : "dump",
                                is_excavate_ ? excavate_hold_ : dump_hold_);
                    actuator_->clear_errors();
                    hold_start_ = now();
                    state_ = State::HOLDING;
                }
                break;

            case State::HOLDING: {
                actuator_->run_out();
                actuator_->send_command();
                double elapsed = (now() - hold_start_).seconds();
                double hold = is_excavate_ ? excavate_hold_ : dump_hold_;
                if (elapsed >= hold) {
                    actuator_->run_in();
                    actuator_->send_command();
                    state_ = State::RETRACTING;
                    RCLCPP_INFO(get_logger(), "%s: RETRACTING", is_excavate_ ? "excavate" : "dump");
                }
                break;
            }

            case State::RETRACTING:
                actuator_->run_in();
                actuator_->send_command();
                if (actuator_->get_error_code() == 5 /* endstop */ ||
                    actuator_->get_error_code() == 12 /* position not changing */) {
                    RCLCPP_INFO(get_logger(), "%s: sequence complete",
                                is_excavate_ ? "excavate" : "dump");
                    actuator_->clear_errors();
                    go_idle();
                }
                break;
        }

        // Publish telemetry
        std_msgs::msg::Float32 pos;
        pos.data = actuator_->get_position_mm();
        pos_pub_->publish(pos);

        std_msgs::msg::UInt8 err;
        err.data = actuator_->get_error_code();
        err_pub_->publish(err);

        std_msgs::msg::String st;
        st.data = state_name();
        state_pub_->publish(st);
    }

    const char * state_name() const
    {
        switch (state_) {
            case State::IDLE:       return "IDLE";
            case State::EXTENDING:  return "EXTENDING";
            case State::HOLDING:    return "HOLDING";
            case State::RETRACTING: return "RETRACTING";
        }
        return "UNKNOWN";
    }

    std::unique_ptr<LinakActuator> actuator_;

    State   state_     = State::IDLE;
    bool    is_excavate_ = false;
    rclcpp::Time hold_start_;

    uint8_t speed_         = 50;
    double  excavate_hold_ = 5.0;
    double  dump_hold_     = 5.0;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pos_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr   err_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  state_pub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr excavate_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dump_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinaksNode>());
    rclcpp::shutdown();
    return 0;
}
