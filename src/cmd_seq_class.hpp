#pragma once
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

#include "mech.hpp"

class MechSeqNode : public rclcpp::Node
{
public:
    using DaizaCmd = mecha_control::action::DaizaCmd;
    using GoalHandleDaizaCmd = rclcpp_action::ServerGoalHandle<DaizaCmd>;
    using HinaCmd = mecha_control::action::HinaCmd;
    using GoalHandleHinaCmd = rclcpp_action::ServerGoalHandle<HinaCmd>;
    MechSeqNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : rclcpp::Node("mech_cmd_seq_node", options), mech(*this)
    {
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
    Mech mech;
    rclcpp_action::Server<DaizaCmd>::SharedPtr daiza_action_server;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr bonbori_service;

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

        rclcpp::WallRate loop(0.1);
        u_int8_t step = 0;
        for (int i = 0; i < 100 && rclcpp::ok(); i++) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
                result->result = DaizaCmd::Result::ABORTED;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            switch (goal->command)
            {
            case DaizaCmd::Goal::STOP:
                result->result = DaizaCmd::Result::OK;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::STOP OK");
                return;
                break;
            case DaizaCmd::Goal::READY:{
                Mech::DaizaActuator act;
                act.expand = 0;
                act.clamp  = 1;
                act.guide_close = 1;
                mech.set_daiza(act);

                auto state = this->mech.get_daiza();
                if(state.is_expand == 0 && state.is_clamp == 1 && state.is_guide_close == 1){
                    result->result = DaizaCmd::Result::OK;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::READY OK");
                    return;
                }
                break;}
            case DaizaCmd::Goal::EXPAND:{
                Mech::DaizaState state = this->mech.get_daiza();
                Mech::DaizaActuator act;
                act.expand = 1;
                act.clamp  = state.is_clamp;
                act.guide_close = state.is_guide_close;
                mech.set_daiza(act);

                if(state.is_expand == 1){
                    result->result = DaizaCmd::Result::OK;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::EXPAND OK");
                    return;
                }
                break;}
            case DaizaCmd::Goal::CONTRACT:{
                Mech::DaizaState state = this->mech.get_daiza();
                Mech::DaizaActuator act;
                act.expand = 0;
                act.clamp  = state.is_clamp;
                act.guide_close = state.is_guide_close;
                mech.set_daiza(act);

                if(state.is_expand == 0){
                    result->result = DaizaCmd::Result::OK;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::CONTRACT OK");
                    return;
                }
                break;}
            case DaizaCmd::Goal::CLAMP:{
                Mech::DaizaState state = this->mech.get_daiza();
                Mech::DaizaActuator act;
                act.expand = state.is_expand;
                act.clamp  = 1;
                act.guide_close = state.is_guide_close;
                mech.set_daiza(act);

                if(state.is_clamp == 1){
                    result->result = DaizaCmd::Result::OK;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::CLAMP OK");
                    return;
                }
                break;}
            case DaizaCmd::Goal::UNCLAMP:{
                Mech::DaizaState state = this->mech.get_daiza();
                Mech::DaizaActuator act;
                act.expand = state.is_expand;
                act.clamp  = 0;
                act.guide_close = state.is_guide_close;
                mech.set_daiza(act);

                if(state.is_clamp == 0){
                    result->result = DaizaCmd::Result::OK;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::UNCLAMP OK");
                    return;
                }
                break;}
            case DaizaCmd::Goal::EXPAND_AND_UNCLAMP:{
                if(step == 0){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 1;
                    act.clamp  = state.is_clamp;
                    act.guide_close = state.is_guide_close;
                    mech.set_daiza(act);

                    if(state.is_expand == 1){
                        feedback->feedback = DaizaCmd::Feedback::EXPANDED_AT_EXPAND_AND_UNCLAMP;
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "EXPANDED_AT_EXPAND_AND_UNCLAMP");
                        step++;
                    }
                }else if(step == 1){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 1;
                    act.clamp  = 0;
                    act.guide_close = 0;
                    mech.set_daiza(act);

                    if(state.is_clamp == 0 && state.is_guide_close == 0){
                        result->result = DaizaCmd::Result::OK;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::EXPAND_AND_UNCLAMP OK");
                        return;
                    }
                }
                break;}
            case DaizaCmd::Goal::CLAMP_AND_CONTRACT:{
                if(step == 0){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = state.is_expand;
                    act.clamp  = state.is_clamp;
                    act.guide_close = 1;
                    mech.set_daiza(act);

                    if(state.is_guide_close == 1){
                        // feedback->feedback = DaizaCmd::Feedback::CLAMPED_AT_CLAMP_AND_CONTRACT;
                        // goal_handle->publish_feedback(feedback);
                        // RCLCPP_INFO(this->get_logger(), "CLAMPED_AT_CLAMP_AND_CONTRACT");
                        step++;
                    }
                }else if(step == 1){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = state.is_expand;
                    act.clamp  = 1;
                    act.guide_close = 1;
                    mech.set_daiza(act);

                    if(state.is_clamp == 1){
                        feedback->feedback = DaizaCmd::Feedback::CLAMPED_AT_CLAMP_AND_CONTRACT;
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "CLAMPED_AT_CLAMP_AND_CONTRACT");
                        step++;
                    }
                }else if(step == 2){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 0;
                    act.clamp  = 1;
                    act.guide_close = 1;
                    mech.set_daiza(act);

                    if(state.is_expand == 0){
                        result->result = DaizaCmd::Result::OK;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::CLAMP_AND_CONTRACT OK");
                        return;
                    }
                }
                break;}
            case DaizaCmd::Goal::EXPAND_AND_PLACE:{
                if(step == 0){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 1;
                    act.clamp  = state.is_clamp;
                    act.guide_close = state.is_guide_close;
                    mech.set_daiza(act);

                    if(state.is_expand == 1){
                        feedback->feedback = DaizaCmd::Feedback::EXPANDED_AT_EXPAND_AND_PLACE;
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "EXPANDED_AT_EXPAND_AND_PLACE");
                        step++;
                    }
                }else if(step == 1){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 1;
                    act.clamp  = 0;
                    act.guide_close = state.is_guide_close;
                    mech.set_daiza(act);

                    if(state.is_clamp == 0){
                        result->result = DaizaCmd::Result::OK;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::EXPAND_AND_PLACE OK");
                        return;
                    }
                }
                break;}
            case DaizaCmd::Goal::EXPAND_AND_PLACE_AND_CONTRACT:{
                if(step == 0){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 1;
                    act.clamp  = state.is_clamp;
                    act.guide_close = state.is_guide_close;
                    mech.set_daiza(act);

                    if(state.is_expand == 1){
                        feedback->feedback = DaizaCmd::Feedback::EXPANDED_AT_EXPAND_AND_PLACE_AND_CONTRACT;
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "EXPANDED_AT_EXPAND_AND_PLACE_AND_CONTRACT");
                        step++;
                    }
                }else if(step == 1){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 1;
                    act.clamp  = 0;
                    act.guide_close = state.is_guide_close;
                    mech.set_daiza(act);

                    if(state.is_clamp == 0){
                        feedback->feedback = DaizaCmd::Feedback::PLACE_AT_EXPAND_AND_PLACE_AND_CONTRACT;
                        goal_handle->publish_feedback(feedback);
                        RCLCPP_INFO(this->get_logger(), "PLACE_AT_EXPAND_AND_PLACE_AND_CONTRACT");
                        step++;
                    }
                }else if(step == 2){
                    Mech::DaizaState state = this->mech.get_daiza();
                    Mech::DaizaActuator act;
                    act.expand = 0;
                    act.clamp  = 0;
                    act.guide_close = state.is_guide_close;
                    mech.set_daiza(act);

                    if(state.is_expand == 0){
                        result->result = DaizaCmd::Result::OK;
                        goal_handle->succeed(result);
                        RCLCPP_INFO(this->get_logger(), "DaizaCmd::Goal::EXPAND_AND_PLACE_AND_CONTRACT OK");
                        return;
                    }
                }
                break;}
            default:{
                result->result = DaizaCmd::Result::ERR_UNEXPECTED_ARG;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "ERR_UNEXPECTED_ARG");
                return;}
            }
            RCLCPP_INFO(this->get_logger(), "loop: %d", i);
            loop.sleep();
        }

        
        result->result = DaizaCmd::Result::ERR_TIMEOUT;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "ERR_TIMEOUT");
    }

    void set_bonbori_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) const
    {
        response->success = true;
        response->message = "";
        RCLCPP_INFO(this->get_logger(), "bonbori: %d", request->data);
    }
};