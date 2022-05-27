#include "ros/ros.h"
#include <iostream>
#include <functional>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "pan_tilt_pi_ctrl/cmd_actionAction.h"
/*Shorten long and dirty type definition*/
using GOAL_STATE = actionlib::SimpleClientGoalState::StateEnum;
/*do you wanna have a bad time?*/
#define USE_LAMBDA 1
#define VIEW_FEEDBACK 0

class MyActionClient{
    protected:
    /*Actual action client*/
    actionlib::SimpleActionClient<pan_tilt_pi_ctrl::cmd_actionAction> ac_;
    /*Goal input*/
    pan_tilt_pi_ctrl::cmd_actionGoal goal_;

    #if (!USE_LAMBDA)
    void DoneCB(const actionlib::SimpleClientGoalState &state, const pan_tilt_pi_ctrl::cmd_actionResultConstPtr& _result){
        /*Report the status when it finished in such cases.*/
        auto state_now = ac_.getState();
        if(state_now == GOAL_STATE::SUCCEEDED){
            ROS_INFO("The action has finished successfully.: %d", _result->final_count);
        }else if(state_now == GOAL_STATE::PREEMPTED){
            ROS_INFO("The action has Preempted.");
        }else{
            ROS_ERROR("The action has terminated unexpectly.");
        }
    }
    #endif
    void ActiveCB(){
        ROS_INFO("Now the action has activated");
    }
    void FeedbackCB(const pan_tilt_pi_ctrl::cmd_actionFeedbackConstPtr& _feedback){
        /*Report when the server published feedback.*/
        #if VIEW_FEEDBACK
        ROS_INFO("Feedback: Now the state is %lf", _feedback->snake_posi_fb);
        #endif
    }
    #if USE_LAMBDA
    /*Non-static member cannot be declared using "auto" but argu*/
    std::function<void(const actionlib::SimpleClientGoalState&,const pan_tilt_pi_ctrl::cmd_actionResultConstPtr&)> DoneCBThis 
    = [this](const auto& state, const auto& _result)->void{
        /*Report the status when it finished in such cases.*/
        auto state_now = ac_.getState();
        if(state_now == GOAL_STATE::SUCCEEDED){
            ROS_INFO("The action has finished successfully.: %lf", _result->snake_posi);
        }else if(state_now == GOAL_STATE::PREEMPTED){
            ROS_INFO("The action has Preempted.");
        }else{
            ROS_ERROR("The action has terminated unexpectly.");
        };
    };
    #endif

    public:
    MyActionClient(ros::NodeHandle& _nh, const std::string& _to_subscribe = ""):
    ac_(_nh, _to_subscribe,true)    {
        ac_.waitForServer();
    };
    void SendGoal(const double& _goal){
        goal_.snake_posi_dest = _goal;
        ROS_INFO("Sending Goal [%lf]",goal_.snake_posi_dest);
        ac_.sendGoal(goal_,
        #if USE_LAMBDA
        DoneCBThis
        #endif
        ,boost::bind(&MyActionClient::ActiveCB, this)
        ,boost::bind(&MyActionClient::FeedbackCB, this, _1)    );
    }
    void CancelGoal(){
        ac_.cancelGoal();
    }
    const actionlib::SimpleClientGoalState GetState() const {
        auto state_now = ac_.getState();
        return state_now;
    }
};

int main (int argc, char **argv) {

    /*run this code with argument to control "snake" "snake controller"*/
    ros::init(argc, argv, "cmd_action_client");
    ros::NodeHandle nh_;


    /*Set a goal from argument 1*/
    pan_tilt_pi_ctrl::cmd_actionGoal goal;
    goal.snake_posi_dest = atof(argv[1]);


    ROS_INFO("Waiting for action server to start.");
    auto myActClient = new MyActionClient(nh_, "cmd_action");
    ROS_INFO("Action server started, sending goal.");

    myActClient->SendGoal(atoi(argv[1]));


    ros::Rate sleep_rate(1);
    while(ros::ok()){
        auto GOAL_STATE = myActClient->GetState();
        ROS_INFO("Goal status: %s", GOAL_STATE.toString().c_str());
        if((GOAL_STATE != GOAL_STATE::ACTIVE)&&(GOAL_STATE != GOAL_STATE::PENDING))
        {
            delete myActClient; /*Goal is deadvertised by delete*/
            break;/*E.N.D*/
        }
        sleep_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}