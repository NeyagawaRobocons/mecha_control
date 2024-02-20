#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "mecha_control/srv/mech_cmd.hpp"
#include "mecha_control/action/mech_cmd.hpp"
#include "mecha_control/action/daiza_cmd.hpp"
#include "mecha_control/action/hina_cmd.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "mecha_control/msg/actuator_commands.hpp"
#include "mecha_control/msg/sensor_states.hpp"
#include <thread>
#include <mutex>
#include <unistd.h>

class MechSeqNode : public rclcpp::Node
{
public:
    using DaizaCmd = mecha_control::action::DaizaCmd;
    using GoalHandleDaizaCmd = rclcpp_action::ServerGoalHandle<DaizaCmd>;
    using HinaCmd = mecha_control::action::HinaCmd;
    using GoalHandleHinaCmd = rclcpp_action::ServerGoalHandle<HinaCmd>;
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
        // daiza_service = this->create_service<mecha_control::srv::MechCmd>(
        //     "daiza_cmd", std::bind(&MechSeqNode::daiza_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
        // hina_service = this->create_service<mecha_control::srv::MechCmd>(
        //     "hina_cmd", std::bind(&MechSeqNode::hina_cmd_callback, this, std::placeholders::_1, std::placeholders::_2));
        using namespace std::placeholders;
        this->daiza_action_server = rclcpp_action::create_server<DaizaCmd>(
            this,
            "daiza_cmd",
            std::bind(&MechSeqNode::daiza_handle_goal, this, _1, _2),
            std::bind(&MechSeqNode::daiza_handle_cancel, this, _1),
            std::bind(&MechSeqNode::daiza_handle_accepted, this, _1));
        bonbori_service = this->create_service<std_srvs::srv::SetBool>(
            "set_bonbori", std::bind(&MechSeqNode::set_bonbori_callback, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "node started");
    }

private:
    rclcpp_action::Server<DaizaCmd>::SharedPtr daiza_action_server;
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
    
    // void daiza_cmd_callback(const std::shared_ptr<mecha_control::srv::MechCmd::Request> request,
    //     std::shared_ptr<mecha_control::srv::MechCmd::Response> response) const
    // {
    //     response->result = mecha_control::srv::MechCmd::Response::OK;
    //     RCLCPP_INFO(this->get_logger(), "Service daiza_cmd called with: %d", request->command);
    // }
    // void hina_cmd_callback(const std::shared_ptr<mecha_control::srv::MechCmd::Request> request,
    //     std::shared_ptr<mecha_control::srv::MechCmd::Response> response) const
    // {
    //     response->result = mecha_control::srv::MechCmd::Response::OK;
    //     RCLCPP_INFO(this->get_logger(), "Service hina_cmd called with: %d", request->command);
    // }
    rclcpp_action::GoalResponse daiza_handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const DaizaCmd::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->command);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse daiza_handle_cancel(
        const std::shared_ptr<GoalHandleDaizaCmd> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void daiza_handle_accepted(const std::shared_ptr<GoalHandleDaizaCmd> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&MechSeqNode::daiza_cmd_execute, this, _1), goal_handle}.detach();
    }
    void daiza_cmd_execute(const std::shared_ptr<GoalHandleDaizaCmd> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<DaizaCmd::Feedback>();
        auto result = std::make_shared<DaizaCmd::Result>();

        for (int i = 0; i < 100 && rclcpp::ok(); i++) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->result = 0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            feedback->feedback = 0;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");
        }

        // Check if goal is done
        if (rclcpp::ok()) {
            result->result = 0;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
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
