#include "rclcpp/rclcpp.hpp"
#include "mecha_control/msg/sensor_states.hpp"
#include <iostream>
#include <thread>
#include <mutex>
#include <unistd.h>

template<typename _Node>
class Mech
{
private:
    std::thread mech_observer;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr daiza_pub;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr hina_pub;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr daiza_sensor_sub_;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr hina_sensor_sub_;
    std::mutex daiza_mutex;
    DaizaState daiza;
    std::mutex hina_mutex;
    HinaState  hina;
    void daiza_sensor_callback(const mecha_control::msg::SensorStates message) const {
        this->daiza_mutex.lock();
        this->daiza.is_clamp        = message.cylinder_states[0];
        this->daiza.is_guide_close  = message.cylinder_states[2];
        this->daiza.is_extract      = message.cylinder_states[3];
        this->daiza_mutex.unlock();
    }
    void hina_sensor_callback(const mecha_control::msg::SensorStates message) const {
        this->hina_mutex.lock();
        this->hina.angle         = message.potentiometer_angles[0];
        this->hina.is_up         = message.limit_switch_states[0];
        this->hina.is_down       = message.limit_switch_states[1];
        this->hina.hina_launch_1 = message.cylinder_states[0];
        this->hina.hina_launch_2 = message.cylinder_states[1];
        this->hina_mutex.unlock()
    }
public:
    Mech(_Node *node){
        this->daiza_sensor_sub_ = create_subscription<mecha_control::msg::SensorStates>(
            "daiza_state", 10, std::bind(&MechSeqNode::daiza_sensor_callback, node, std::placeholders::_1));
        this->hina_sensor_sub_  = create_subscription<mecha_control::msg::SensorStates>(
            "hina_state", 10, std::bind(&MechSeqNode::hina_sensor_callback, node, std::placeholders::_1));
    }
    DaizaState get_daiza(){
        this->daiza_mutex.lock();
        DaizaState ret = this->daiza;
        this->daiza_mutex.unlock();
        return ret;
    }
    HinaState get_hina(){
        this->hina_mutex.lock();
        DaizaState ret = this->hina;
        this->hina_mutex.unlock();
        return ret;
    }
    struct DaizaState{
        bool is_extract;
        bool is_clamp;
        bool is_guide_close;
    }
    struct HinaState{
        float angle;
        bool is_up;
        bool is_down;
        bool hina_launch_1;
        bool hina_launch_2;
    }
};
