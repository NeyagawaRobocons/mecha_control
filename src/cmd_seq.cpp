#include "rclcpp/rclcpp.hpp"
#include "mecha_control/srv/mech_cmd.hpp"
#include "mecha_control/action/mech_cmd.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "mecha_control/msg/actuator_commands.hpp"
#include "mecha_control/msg/sensor_states.hpp"
#include <thread>
#include <mutex>
#include <unistd.h>

class MechSeqNode : public rclcpp::Node
{
public:
    MechSeqNode()
    : Node("mech_cmd_seq_node")
    {
        daiza_pub = create_publisher<mecha_control::msg::ActuatorCommands>("daiza_clamp", 10);
        hina_pub = create_publisher<mecha_control::msg::ActuatorCommands>("hina_dastpan", 10);
        daiza_controller = std::thread(&MechSeqNode::daiza_controll, this);
        // hina_controller = std::thread(&MechSeqNode::hina_controll, this);
        // bonbori_controller = std::thread(&MechSeqNode::bonbori_controll, this);
        daiza_sensor_sub = create_subscription<mecha_control::msg::SensorStates>(
            "daiza_state", 10, std::bind(&MechSeqNode::daiza_sensor_callback, this, std::placeholders::_1));
        hina_sensor_sub = create_subscription<mecha_control::msg::SensorStates>(
            "hina_state", 10, std::bind(&MechSeqNode::hina_sensor_callback, this, std::placeholders::_1));
        daiza_service = this->create_service<mecha_control::srv::MechCmd>(
            "daiza_cmd", std::bind(&MechSeqNode::daiza_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
        hina_service = this->create_service<mecha_control::srv::MechCmd>(
            "hina_cmd", std::bind(&MechSeqNode::hina_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
        bonbori_service = this->create_service<std_srvs::srv::SetBool>(
            "set_bonbori", std::bind(&MechSeqNode::set_bonbori_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "node started");
    }

private:
    rclcpp::Service<mecha_control::srv::MechCmd>::SharedPtr daiza_service;
    rclcpp::Service<mecha_control::srv::MechCmd>::SharedPtr hina_service;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bonbori_service;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr daiza_pub;
    rclcpp::Publisher<mecha_control::msg::ActuatorCommands>::SharedPtr hina_pub;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr daiza_sensor_sub;
    rclcpp::Subscription<mecha_control::msg::SensorStates>::SharedPtr hina_sensor_sub;
    std::thread daiza_controller;
    std::thread hina_controller;
    std::thread bonbori_controller;
    
    void daiza_cmd_callback(const std::shared_ptr<mecha_control::srv::MechCmd::Request> request,
        std::shared_ptr<mecha_control::srv::MechCmd::Response> response) const
    {
        response->result = mecha_control::srv::MechCmd::Response::OK;
        RCLCPP_INFO(this->get_logger(), "Service daiza_cmd called with: %d", request->command);
    }
    void hina_cmd_callback(const std::shared_ptr<mecha_control::srv::MechCmd::Request> request,
        std::shared_ptr<mecha_control::srv::MechCmd::Response> response) const
    {
        response->result = mecha_control::srv::MechCmd::Response::OK;
        RCLCPP_INFO(this->get_logger(), "Service hina_cmd called with: %d", request->command);
    }
    void set_bonbori_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) const
    {
        response->success = true;
        response->message = "";
        RCLCPP_INFO(this->get_logger(), "bonbori: %d", request->data);
    }
    void daiza_sensor_callback(const mecha_control::msg::SensorStates) const {}
    void hina_sensor_callback(const mecha_control::msg::SensorStates) const {}
    void daiza_controll(){
        while(1){
            
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MechSeqNode>());
    rclcpp::shutdown();
    return 0;
}
