#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "mecha_control/msg/mecha_state.hpp"
#include "mecha_control/msg/sensor_states.hpp"
#include "mecha_control/msg/actuator_commands.hpp"

using namespace std::chrono_literals;

class SequenceController : public rclcpp::Node {
public:
    SequenceController()
    : Node("sequence_controller"),
    delay_rate(1000ms)  // 1秒のディレイ
    {
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
        
        daiza_commands = mecha_control::msg::ActuatorCommands();
        hina_commands = mecha_control::msg::ActuatorCommands();

        // 台座機構の目標状態を初期化
        daiza_commands.cylinder_states.resize(4, false);  // シリンダの状態をすべて false（縮んでいる）に初期化

        // 人形機構の目標状態を初期化
        hina_commands.motor_positions.resize(1, 0.0);    // モータの位置を 0.0（初期位置）に初期化
        hina_commands.motor_expand.resize(1, false);     // モータの展開状態をすべて false（縮んでいる）に初期化
    }

private:
    rclcpp::WallRate delay_rate;    // ディレイ用のレート

    rclcpp::Subscription<mecha_control::msg::MechaState>::SharedPtr mecha_state_subscription_;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr daiza_state_subscription_;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr hina_state_subscription_;

    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr daiza_commands_publisher_;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr hina_commands_publisher_;

    // 台座と人形機構の現在の状態を保持するメンバ変数
    mecha_control::msg::SensorStates current_daiza_state_;
    mecha_control::msg::SensorStates current_hina_state_;

    rclcpp::TimerBase::SharedPtr timer_; // タイマーのメンバ変数

    mecha_control::msg::ActuatorCommands daiza_commands;
    mecha_control::msg::ActuatorCommands hina_commands;

    void mecha_state_callback(const mecha_control::msg::MechaState::SharedPtr msg) {
        // 台座機構の処理
        if (!(msg->daiza_state == 0)) {
            // 台座機構の目標状態に応じた処理をここに実装
            switch (msg->daiza_state) {
            case 1: // 展開
                std::cout << "台座機構の展開" << std::endl;
                // 展開用シリンダを動かす指令を送信
                daiza_commands.cylinder_states[3] = true;
                daiza_commands_publisher_->publish(daiza_commands);
                delay_with_log();   // 1秒待つ（シリンダが展開するのを待つ）
                // シリンダ1, 2, 3を展開する指令を送信
                for (int i = 0; i < 3; i++) { daiza_commands.cylinder_states[i] = true; }
                daiza_commands_publisher_->publish(daiza_commands);
                break;
            case 2: // 回収
                std::cout << "台座機構の回収" << std::endl;
                if (!current_daiza_state_.limit_switch_states.empty()) {
                    if (current_daiza_state_.limit_switch_states[2]) {
                        // シリンダ(小)を縮める指令を送信（小台座を倒す）
                        daiza_commands.cylinder_states[2] = false;
                        daiza_commands_publisher_->publish(daiza_commands);
                        // this->delay_rate.sleep();   // 1秒待つ
                        delay_with_log();
                        // シリンダ(右), (左)を縮める: 台座を挟む
                        daiza_commands.cylinder_states[0] = false;
                        daiza_commands.cylinder_states[1] = false;
                        daiza_commands_publisher_->publish(daiza_commands);
                    }
                }
                else { std::cout << "limit_switch_states is empty" << std::endl; }
                delay_with_log();   // 1秒待つ
                // 展開用シリンダを動かす指令を送信(縮める)
                daiza_commands.cylinder_states[3] = false;
                daiza_commands_publisher_->publish(daiza_commands);
                break;
            case 3: // 設置
                std::cout << "台座機構の設置" << std::endl;
                // 展開用シリンダを動かす指令を送信
                daiza_commands.cylinder_states[3] = true;
                daiza_commands_publisher_->publish(daiza_commands);
                delay_with_log();   // 1秒待つ（シリンダが展開するのを待つ）
                // シリンダ1, 2, 3を展開する指令を送信
                for (int i = 0; i < 3; i++) { daiza_commands.cylinder_states[i] = true; }
                daiza_commands_publisher_->publish(daiza_commands);    
                break;
            default:
                break;
            }
        } 
        // 人形機構の処理
        else if (!(msg->hina_state == 0)) {
            switch (msg->hina_state) {
            case 4: // 準備
                std::cout << "人形機構の準備" << std::endl;
                // モータ2: ポテンショメータが90°になるまで展開
                if (!current_hina_state_.potentiometer_angles.empty()) { 
                    if (current_hina_state_.potentiometer_angles[1] < 90.0) {
                        hina_commands.motor_positions[1] = 90.0;
                        hina_commands_publisher_->publish(hina_commands);
                    }
                }
                else { std::cout << "potentiometer_angles is empty" << std::endl; }
                // モータ1: リミットスイッチ(下)が押されるまで縮める
                hina_commands.motor_expand[0] = false;
                hina_commands_publisher_->publish(hina_commands);
                break;
            case 1: // 展開
                std::cout << "人形機構の展開" << std::endl;
                // モータ2: ポテンショメータが-10° (調整できるようにする)になるまで動かす
                if (!current_hina_state_.potentiometer_angles.empty()) {
                    if (current_hina_state_.potentiometer_angles[1] > -10.0) {
                        hina_commands.motor_positions[1] = -10.0;
                        hina_commands_publisher_->publish(hina_commands);
                    }
                }
                else { std::cout << "potentiometer_angles is empty" << std::endl; }
                break;
            case 2: // 回収
                std::cout << "人形機構の回収" << std::endl;
                // if (リミットスイッチ(壁)1, 2が両方押されたら)
                if (!current_hina_state_.limit_switch_states.empty()) {
                    if (current_hina_state_.limit_switch_states[2] && current_hina_state_.limit_switch_states[3]) {
                        // モータ2: テンショメータ1,2が90°になるまで縮める
                        if (current_hina_state_.potentiometer_angles[1] < 90.0) {
                            hina_commands.motor_positions[1] = 90.0;
                            hina_commands_publisher_->publish(hina_commands);
                        }
                        // モータ1: リミットスイッチ(下)が押されるまで縮める
                        hina_commands.motor_expand[0] = false;
                        hina_commands_publisher_->publish(hina_commands);
                    }
                }
                else { std::cout << "limit_switch_states is empty" << std::endl; }
                break;
            case 3: // 設置
                std::cout << "人形機構の設置" << std::endl;
                // if (リミットスイッチ(上)が押されいる)
                if (!current_hina_state_.limit_switch_states.empty()) {
                    if (current_hina_state_.limit_switch_states[0]) {
                        // モータ2: ポテンショメータが-10° (調整できるようにする)になるまで動かす
                        if (current_hina_state_.potentiometer_angles[1] > -10.0) {
                            hina_commands.motor_positions[1] = -10.0;
                            hina_commands_publisher_->publish(hina_commands);
                        }
                        delay_with_log();   // 1秒待つ
                        // モータ2: テンショメータが90°になるまで縮める
                        if (current_hina_state_.potentiometer_angles[1] < 90.0) {
                            hina_commands.motor_positions[1] = 90.0;
                            hina_commands_publisher_->publish(hina_commands);
                        }
                        // ぼんぼり点灯をする
                    }
                }
                else { std::cout << "limit_switch_states is empty" << std::endl; }
                break;
            default:
                break;
            }
        }
        else if (msg->bonbori_state) {
            // ぼんぼり点灯をする
            std::cout << "ぼんぼり点灯" << std::endl;
        }
    }
    void delay_with_log() {
        RCLCPP_INFO(this->get_logger(), "delay");
        this->delay_rate.sleep();
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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SequenceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}