#include "rclcpp/rclcpp.hpp"
#include "mecha_control/msg/mecha_state.hpp"
#include "mecha_control/msg/sensor_states.hpp"
#include "mecha_control/msg/actuator_commands.hpp"

class SequenceController : public rclcpp::Node {
public:
    SequenceController() : Node("sequence_controller") {
        // メッセージのサブスクライバーを設定
        mecha_state_subscription_ = this->create_subscription<mecha_control::msg::MechaState>(
            "/mecha_state", 10, std::bind(&SequenceController::mecha_state_callback, this, std::placeholders::_1));
        daiza_state_subscription_ = this->create_subscription<mecha_control::msg::SensorStates>(
            "/daiza_state", 10, std::bind(&SequenceController::daiza_state_callback, this, std::placeholders::_1));
        hina_state_subscription_ = this->create_subscription<mecha_control::msg::SensorStates>(
            "/hina_state", 10, std::bind(&SequenceController::hina_state_callback, this, std::placeholders::_1));

        // メッセージのパブリッシャーを設定
        daiza_commands_publisher_ = this->create_publisher<mecha_control::msg::ActuatorCommands>("/daiza_clamp", 10);
        hina_commands_publisher_ = this->create_publisher<mecha_control::msg::ActuatorCommands>("/hina_dastpan", 10);
        
        // 台座と人形機構の現在の状態を保存するメンバ変数の初期化
        current_daiza_state_ = mecha_control::msg::SensorStates();
        current_hina_state_ = mecha_control::msg::SensorStates();
    }

private:
    void mecha_state_callback(const mecha_control::msg::MechaState::SharedPtr msg) {
        // 台座機構の処理
        if (msg->daiza_state) {
            // 台座機構の目標状態に応じた処理をここに実装
        } 
        // 人形機構の処理
        else if (msg->hina_state) {
            // 人形機構の目標状態に応じた処理をここに実装
        }
        // ぼんぼり点灯の処理
        else if (msg->bonbori_state) {
            // ぼんぼり点灯の処理をここに実装
        }
    }

    void daiza_state_callback(const mecha_control::msg::SensorStates::SharedPtr msg) {
        // 台座機構の状態を更新
        current_daiza_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "received daiza state");
    }

    void hina_state_callback(const mecha_control::msg::SensorStates::SharedPtr msg) {
        // 人形機構の状態を更新
        current_hina_state_ = *msg;
        RCLCPP_INFO(this->get_logger(), "received hina state");
    }

    rclcpp::Subscription<mecha_control::msg::MechaState>::SharedPtr mecha_state_subscription_;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr daiza_state_subscription_;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr hina_state_subscription_;

    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr daiza_commands_publisher_;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr hina_commands_publisher_;

    // 台座と人形機構の現在の状態を保持するメンバ変数
    mecha_control::msg::SensorStates current_daiza_state_;
    mecha_control::msg::SensorStates current_hina_state_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SequenceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}