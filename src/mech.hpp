#pragma once
#include "rclcpp/rclcpp.hpp"
#include "nucleo_agent/msg/actuator_commands.hpp"
#include "nucleo_agent/msg/sensor_states.hpp"
#include <iostream>
#include <thread>
#include <mutex>
#include <unistd.h>


class Mech
{
public:
    struct DaizaState{
        bool is_expand = 0;
        bool is_clamp = 0;
        bool is_guide_close = 0;
    };
    struct HinaState{
        float angle = 0;
        bool is_pos_carry = 0;
        bool is_pos_take = 0;
        bool is_pos_place = 0;
        bool is_up = 0;
        bool is_down = 0;
        bool launched_hina_1 = 0;
        bool launched_hina_2 = 0;
    };
    struct DaizaActuator{
        bool expand = 0;
        bool clamp = 0;
        bool guide_close = 0;
    };
    struct HinaActuator{
        float angle = 0;
        bool up = 0;
        bool guide_expand  = 0;
        bool launch_hina_1 = 0;
        bool launch_hina_2 = 0;
    };

    template<typename NodeT>
    Mech(NodeT && node) : daiza_mutex(), hina_mutex(){
        this->daiza_pub_ = rclcpp::create_publisher<nucleo_agent::msg::ActuatorCommands>(node, "daiza_clamp", 10);
        this->hina_pub_ = rclcpp::create_publisher<nucleo_agent::msg::ActuatorCommands>(node, "hina_dastpan", 10);
        this->daiza_sensor_sub_ = rclcpp::create_subscription<nucleo_agent::msg::SensorStates>(
            node, "daiza_state", 10, std::bind(&Mech::daiza_sensor_callback, this, std::placeholders::_1));
        this->hina_sensor_sub_  = rclcpp::create_subscription<nucleo_agent::msg::SensorStates>(
            node, "hina_state", 10, std::bind(&Mech::hina_sensor_callback, this, std::placeholders::_1));
    }
    struct DaizaState get_daiza(){
        this->daiza_mutex.lock();
        struct DaizaState ret = this->daiza;
        this->daiza_mutex.unlock();
        return ret;
    }
    struct HinaState get_hina(){
        this->hina_mutex.lock();
        struct HinaState ret = this->hina;
        this->hina_mutex.unlock();
        return ret;
    }
    void set_daiza(struct DaizaActuator act){
        auto message = nucleo_agent::msg::ActuatorCommands();
        message.cylinder_states.resize(4, false);
        message.cylinder_states[0] = act.clamp;
        message.cylinder_states[1] = act.clamp;
        message.cylinder_states[2] = act.guide_close;
        message.cylinder_states[3] = act.expand;
        daiza_pub_->publish(message);
    }
    void set_hina(struct HinaActuator act){
        auto message = nucleo_agent::msg::ActuatorCommands();
        message.cylinder_states.resize(2, false);
        message.motor_expand.resize(1, false);
        message.motor_positions.resize(3, false);
        message.cylinder_states[0] = act.launch_hina_1;
        message.cylinder_states[1] = act.launch_hina_2;
        message.motor_expand[0]    = act.up;
        message.motor_positions[0] = act.angle;
        message.motor_positions[1] = act.guide_expand ? 0.0 : 0.78539816339745;
        message.motor_positions[2] = act.guide_expand ? 0.0 : -0.78539816339745;
        daiza_pub_->publish(message);
    }
private:
    std::thread mech_observer;
    rclcpp::Publisher<nucleo_agent::msg::ActuatorCommands>::SharedPtr daiza_pub_;      // cylinder_states.size()==4 && motor_expand.size()==0 && motor_positions.size()==0
    rclcpp::Publisher<nucleo_agent::msg::ActuatorCommands>::SharedPtr hina_pub_;       // cylinder_states.size()==2 && motor_expand.size()==1 && motor_positions.size()==3
    rclcpp::Subscription<nucleo_agent::msg::SensorStates>::SharedPtr daiza_sensor_sub_;
    rclcpp::Subscription<nucleo_agent::msg::SensorStates>::SharedPtr hina_sensor_sub_;
    std::mutex daiza_mutex;
    struct DaizaState daiza;
    std::mutex hina_mutex;
    struct HinaState  hina;
    void daiza_sensor_callback(const nucleo_agent::msg::SensorStates message) {
        this->daiza_mutex.lock();
        this->daiza.is_clamp        = message.cylinder_states[0];
        this->daiza.is_guide_close  = message.cylinder_states[2];
        this->daiza.is_expand      = message.cylinder_states[3];
        this->daiza_mutex.unlock();
    }
    void hina_sensor_callback(const nucleo_agent::msg::SensorStates message) {
        this->hina_mutex.lock();
        this->hina.angle         = message.potentiometer_angles[0];
        this->hina.is_up         = message.limit_switch_states[0];
        this->hina.is_down       = message.limit_switch_states[1];
        this->hina.launched_hina_1 = message.cylinder_states[0];
        this->hina.launched_hina_2 = message.cylinder_states[1];
        this->hina_mutex.unlock();
    }
};
