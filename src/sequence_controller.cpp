"""
# mecha_control/msg/MechaState.msg
# 0: 準備
# 1: 展開
# 2: 回収
# 3: 設置
# false: ぼんぼりオフ
# true: ぼんぼり点灯

byte daiza_state
byte hina_state
bool bonbori_state
"""

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
        // 台座機構のアクチュエータ指令の初期化
        mecha_control::msg::ActuatorCommands daiza_commands;
        // daiza_commands.cylinder_states.resize(3, false);  // シリンダの状態をすべて false（縮んでいる）に初期化
        // daiza_commands.motor_positions.resize(1, 0.0);    // モータの位置を 0.0（初期位置）に初期化

        // 人形機構のアクチュエータ指令の初期化
        mecha_control::msg::ActuatorCommands hina_commands;
        // hina_commands.cylinder_states.resize(2, false);  // シリンダの状態をすべて false（縮んでいる）に初期化
        // hina_commands.motor_positions.resize(2, 0.0);    // モータの位置を 0.0（初期位置）に初期化

        // // タイマーの初期化（ここではまだスタートしない）
        // timer_ = this->create_wall_timer(
        //     std::chrono::seconds(1),  // 1秒後に実行
        //     std::bind(&SequenceController::timer_callback, this));
        daiza_set_flag = false; // リミットスイッチ(下)が押されたらシリンダ1, 2, 3を展開する指令を送信
    }

private:
    // void timer_callback() {
    //     // タイマーのコールバック関数
    //     // ここに1秒後に実行する処理を記述
    //     // 例：シリンダ1, 2を縮める指令を送信
    //     daiza_commands.cylinder_states[0] = false;
    //     daiza_commands.cylinder_states[1] = false;
    //     daiza_commands_publisher_->publish(daiza_commands);
    // }
    void mecha_state_callback(const mecha_control::msg::MechaState::SharedPtr msg) {
        // 台座機構の処理
        if (msg->daiza_state) {
            // // 台座機構の目標状態に応じた処理をここに実装
            // switch (msg->daiza_state)
            // {
            // case 1: // 展開
            //     // // シリンダ1, 2, 3を展開する指令を送信
            //     // for (int i = 0; i < 3; i++) { daiza_commands.cylinder_states[i] = true; }
            //     // // 角度調整モータを動かす指令を送信（リミットスイッチ(下)が押されるまで）
            //     // daiza_commands.motor_positions[0] = 1.0;
            //     // daiza_commands_publisher_->publish(daiza_commands);
            //     std::cout << "daiza_state = 1" << std::endl;
            //     break;
            
            // case 2: // 回収
            //     // if (current_daiza_state_->limit_switch_states[2]) {
            //     //     // シリンダ3を縮める指令を送信（小台座を倒す）
            //     //     daiza_commands.cylinder_states[2] = false;
            //     //     daiza_commands_publisher_->publish(daiza_commands);
            //     //     // タイマーを開始（1秒後にコールバックが実行される）
            //     //     timer_->reset();
            //     // }
            //     // // 角度調整モータを動かす指令を送信（リミットスイッチ(上)が押されるまで）
            //     // daiza_commands.motor_positions[0] = -1.0;
            //     std::cout << "daiza_state = 2" << std::endl;
            //     break;

            // case 3: // 設置
            //     // // 角度調整モータを動かす指令を送信（リミットスイッチ(下)が押されるまで）
            //     // daiza_commands.motor_positions[0] = -1.0;
            //     // daiza_set_flag = true; // リミットスイッチ(下)が押されたらシリンダ1, 2, 3を展開する指令を送信
            //     // daiza_commands_publisher_->publish(daiza_commands);
            //     std::cout << "daiza_state = 3" << std::endl;
            //     break;

            // default:
            //     break;
            // }
        } 
        // 人形機構の処理
        else if (msg->hina_state) {
            // 人形機構の目標状態に応じた処理をここに実装
            // hina_commands_publisher_->publish(hina_commands);
        }
        // ぼんぼり点灯の処理
        else if (msg->bonbori_state) {
            // ぼんぼり点灯の処理をここに実装
        }
    }

    void daiza_state_callback(const mecha_control::msg::SensorStates::SharedPtr msg) {
        // // 台座機構の状態を更新
        // current_daiza_state_ = *msg;
        // // 対応するリミットスイッチが押されたら司令を停止
        // if (daiza_commands->motor_positions[0] > 0.0 && current_daiza_state_->limit_switch_states[1]) {
        //     // リミットスイッチ(下)が押されたら角度調整モータの展開を停止
        //     daiza_commands->motor_positions[0] = 0.0;
        //     if (daiza_set_flag) {
        //         // シリンダ1, 2, 3を展開する指令を送信
        //         for (int i = 0; i < 3; i++) { daiza_commands.cylinder_states[i] = true; }
        //     }
        //     daiza_commands_publisher_->publish(daiza_commands);
        // }
        // else if (daiza_commands->motor_positions[0] < 0.0 && current_daiza_state_->limit_switch_states[0]) {
        //     // リミットスイッチ(上)が押されたら角度調整モータの縮小を停止
        //     daiza_commands->motor_positions[0] = 0.0;
        //     daiza_commands_publisher_->publish(daiza_commands);
        // }
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
    bool daiza_set_flag;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SequenceController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}