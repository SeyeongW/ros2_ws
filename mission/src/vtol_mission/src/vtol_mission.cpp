#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MotorTestNode : public rclcpp::Node
{
public:
    MotorTestNode() : Node("vtol_mission_node")
    {
        mission_state_ = MissionState::WAIT_FOR_COMMAND;

        // --- Subscribers ---
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, std::bind(&MotorTestNode::stateCallback, this, _1));

        start_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/mission/start", 10, std::bind(&MotorTestNode::startCallback, this, _1));

        // Status text (best effort QoS 맞춰주기)
        status_sub_ = this->create_subscription<mavros_msgs::msg::StatusText>(
            "/mavros/statustext/recv",
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            std::bind(&MotorTestNode::statusCallback, this, _1));

        // --- Publisher ---
        rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
            "/mavros/rc/override", 10);

        // --- Service Clients ---
        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/mavros/arming");         

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode");       

        // --- Timer ---
        timer_ = this->create_wall_timer(
            50ms, std::bind(&MotorTestNode::controlLoop, this));

        RCLCPP_INFO(get_logger(),
                    ">>> MP-STYLE Motor Test Ready. Publish std_msgs/Empty on /mission/start.");
    }

private:
    enum class MissionState {
        WAIT_FOR_COMMAND,
        REQ_ARMING,
        SPIN_MOTORS,
        STOPPING,
        DONE
    };

    MissionState mission_state_;
    mavros_msgs::msg::State current_state_;
    rclcpp::Time start_spin_time_;
    rclcpp::Time last_req_time_;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_sub_;
    rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr status_sub_;

    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    // --- Callbacks ---

    void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
    {
        current_state_ = *msg;
    }

    void statusCallback(const mavros_msgs::msg::StatusText::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "[FCU][sev=%d] %s",
                    msg->severity, msg->text.c_str());
    }

    void startCallback(const std_msgs::msg::Empty::SharedPtr)
    {
        if (!current_state_.connected) {
            RCLCPP_WARN(get_logger(),
                        ">>> Start requested but FCU not connected yet.");
            return;
        }

        if (mission_state_ == MissionState::WAIT_FOR_COMMAND) {
            RCLCPP_INFO(get_logger(),
                        ">>> Start signal received. Mode change & arming sequence start.");
            mission_state_ = MissionState::REQ_ARMING;
            last_req_time_ = this->now();
        }
    }

    void sendThrottle(uint16_t pwm)
    {
        mavros_msgs::msg::OverrideRCIn msg;
        for (int i = 0; i < 8; i++) {
            msg.channels[i] = 65535; // "IGNORE" 값
        }

        // 여기선 일단 스로틀만 제어 (CH3)
        msg.channels[2] = pwm;
        rc_pub_->publish(msg);
    }

    // --- Main FSM loop ---

    void controlLoop()
    {
        auto now = this->now();

        switch (mission_state_) {

        case MissionState::WAIT_FOR_COMMAND:
            // 아무것도 안 함
            break;

        case MissionState::REQ_ARMING:
        {
            // 현재 상태 로깅 (디버깅용)
            //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            //                     "STATE: connected=%d, armed=%d, mode=%s",
            //                     current_state_.connected,
            //                     current_state_.armed,
            //                     current_state_.mode.c_str());

            // 1초에 한 번만 시도
            if ((now - last_req_time_).seconds() > 1.0) {
                last_req_time_ = now;

                // 1) 모드 QSTABILIZE 아니면 먼저 모드 변경
                if (current_state_.mode != "QSTABILIZE") {
                    if (!set_mode_client_->service_is_ready()) {
                        RCLCPP_WARN(get_logger(),
                                    "[MODE] set_mode service not ready yet.");
                        return;
                    }

                    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
                    req->custom_mode = "QSTABILIZE";

                    auto future = set_mode_client_->async_send_request(req);
                    // 간단히 비동기만 쏘고 로그만 남김
                    RCLCPP_INFO(get_logger(),
                                ">>> Requesting QSTABILIZE mode...");
                    return;
                }

                // 2) ARM 준비
                if (!current_state_.armed) {
                    if (!arming_client_->service_is_ready()) {
                        RCLCPP_WARN(get_logger(),
                                    "[ARM] arming service not ready yet.");
                        return;
                    }

                    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                    req->value = true;

                    RCLCPP_INFO(get_logger(),
                                ">>> Requesting ARM (command only)...");

                    auto cb =
                        [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture fut)
                        {
                            auto res = fut.get();
                            if (res->success) {
                                RCLCPP_INFO(this->get_logger(),
                                            "[ARM] success, result=%d",
                                            res->result);
                            } else {
                                RCLCPP_WARN(this->get_logger(),
                                            "[ARM] FAILED, result=%d",
                                            res->result);
                            }
                        };

                    arming_client_->async_send_request(req, cb);
                }
            }

            // 3) 실제 state 토픽에서 armed 플래그가 바뀌었는지 확인
            if (current_state_.armed) {
                RCLCPP_INFO(get_logger(),
                            ">>> ARMED detected in state. Go to SPIN_MOTORS.");
                mission_state_ = MissionState::SPIN_MOTORS;
                start_spin_time_ = now;
            }
            break;
        }

        case MissionState::SPIN_MOTORS:
            // QSTABILIZE + ARMED 상태에서 스로틀 올리기
            sendThrottle(1250);

            if ((now - start_spin_time_).seconds() > 5.0) {
                RCLCPP_INFO(get_logger(),
                            ">>> 5 seconds up. Stopping throttle...");
                mission_state_ = MissionState::STOPPING;
                last_req_time_ = now;
            }
            break;

        case MissionState::STOPPING:
            sendThrottle(1000); // 최소 스로틀 값

            if ((now - last_req_time_).seconds() > 1.0) {
                last_req_time_ = now;

                if (current_state_.armed) {
                    if (arming_client_->service_is_ready()) {
                        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
                        req->value = false;
                        arming_client_->async_send_request(req);
                        RCLCPP_INFO(get_logger(), ">>> Requesting DISARM...");
                    }
                } else {
                    RCLCPP_INFO(get_logger(), ">>> Disarmed. Test Done.");
                    mission_state_ = MissionState::WAIT_FOR_COMMAND;
                }
            }
            break;

        case MissionState::DONE:
            // 필요하면 나중에 사용
            break;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorTestNode>());
    rclcpp::shutdown();
    return 0;
}