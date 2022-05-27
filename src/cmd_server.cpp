#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <actionlib/server/simple_action_server.h>
#include <functional>
#include "pan_tilt_pi_ctrl/cmd_actionAction.h"
#include "jnt_sts_subscriber.h"

#define SEND_FEEDBACK_MSG 1

using JntMsgCallback = boost::function< void(const sensor_msgs::JointState &)>;

const size_t len = 2;
double e_sum[len];

class cmd_actionAction
{
    protected:
    /*What is name of the action?*/
    std::string action_name;
    ros::Rate cycle_freq;
    /*Declare the action server*/
    actionlib::SimpleActionServer<pan_tilt_pi_ctrl::cmd_actionAction> as_;
    ros::Publisher snake_publisher;
    ros::Publisher scorpion_publisher;
    pan_tilt_pi_ctrl::cmd_actionFeedback fb_;
    pan_tilt_pi_ctrl::cmd_actionResult result_;
    /*Obtain sensor info.*/
    std::shared_ptr<JntStsSubscriber> jnt_info_handle;
    double posi[len];
    /*PI control parameters*/
    const double upper_sat = 25.0;
    const double lower_sat = 25.0;
    const double Kp = 1.0;

    /*Callback fncs. for exec, preempt*/
    void ExecCallback(const pan_tilt_pi_ctrl::cmd_actionGoalConstPtr& _goal){
        if( (as_.isActive() == false) && (as_.isPreemptRequested() == true) ) return; /*Reject when it is not activated or preempted*/
        /*ROS_INFO("This action trying to reach goal %d", goal_tmp); */
        /*Iterates until the job is complete*/
        while(!isJobCplt(_goal)){
            /*When the user input Ctrl + C to shutdown this node*/
            if(!ros::ok()){
                /*Report progress to final count and set this action as aborted.*/
                result_.snake_posi = posi[0];
                result_.snake_posi = posi[1];
                as_.setAborted(result_, "I failed!");
                ROS_INFO("%s is shutted down.", action_name.c_str());
                break;
            }
            /*Reject when it is not activated or preempted*/
            if( (as_.isActive() == false) && (as_.isPreemptRequested() == true) ) return;
            /*Do job something*/
            job(_goal);
            /*Is the job complete?*/
            if(isJobCplt(_goal)){
                /*Report progress to final count and set this action as succeeded.*/
                result_.snake_posi = posi[0];
                result_.scorpion_posi = posi[1];
                as_.setSucceeded(result_, "I succeeded!");
                /*ROS_INFO("%s is reached the goal %d.", action_name.c_str(), goal_tmp);*/
            }
            else{
                #if SEND_FEEDBACK_MSG
                /*Set the feedback and publish a feedback.*/
                fb_.snake_posi_fb = posi[0];
                fb_.scorpion_posi_fb = posi[1];
                as_.publishFeedback(fb_);
                /*ROS_INFO("%s is %d of the goal %d.", action_name.c_str(), progress,  goal_tmp);*/
                #endif
            }
            cycle_freq.sleep();
        }
    }
    void PreemptCallback(){
        /*Report progress to final count and set this action as preempted.*/
	    result_.snake_posi = posi[0];
        result_.scorpion_posi = posi[1];
	    as_.setPreempted(result_,"I got Preempted"); 
	    ROS_WARN("%s got preempted!", action_name.c_str());
    }


    inline void job(const pan_tilt_pi_ctrl::cmd_actionGoalConstPtr& _goal){
        /*Do feedback ctrl*/
        /*Get sensor info*/
        for(size_t i=0; i<len;++i){
            posi[i] = jnt_info_handle->pos[i];
        }
        /*Get error*/
        double e[len];
        for(size_t i=0; i<len; ++i){
            if((e_sum[i] < 20) ||(e_sum[i] > -20) ){
                e_sum[i] += e[i];
            }
        }
        e[0] = _goal->snake_posi_dest - posi[0];
        /*Calculate output*/

        /*Define torque of "snake_controller"*/
        double u[2];
        u[0] = 0.5*e[0] + 0.01 * e_sum[0];
        std_msgs::Float64 snake_msg;
        snake_msg.data = u[0];
        snake_publisher.publish(snake_msg);

    }
    inline bool isJobCplt(const pan_tilt_pi_ctrl::cmd_actionGoalConstPtr& _goal){
        bool retval = false;
        if(_goal->snake_posi_dest == posi[0]){
            retval = true;
        }
        return retval;
    }

    public:
    cmd_actionAction(ros::NodeHandle& _nh, const std::string& _action_name, const std::shared_ptr<JntStsSubscriber>& _jnt_info_handle, const unsigned int& _cycle_freq = 5)
    /*Register execution callback fnc. with initialization list.*/
    :as_(_nh, _action_name, boost::bind(&cmd_actionAction::ExecCallback, this, _1), false)
    /*Set name and cycle frequency of this action.*/
    ,action_name(_action_name), jnt_info_handle(_jnt_info_handle),cycle_freq(_cycle_freq)
    {
        /*Register Preempt callback fnc..*/
        as_.registerPreemptCallback(boost::bind(&cmd_actionAction::PreemptCallback, this));
        snake_publisher = _nh.advertise<std_msgs::Float64>("/pan_tilt/effort_controller_snake/command",1);
    }

    
    void StartAction(){as_.start();}
};

int main(int argc, char** argv){
    ros::init(argc, argv, "cmd_action_server");
    ros::NodeHandle nh_;
    ros::NodeHandle nh2_;
    
    /*Allocate same names in same order with the hardware_interface*/
    /*I will change it using service of hardware node*/
    std::vector<std::string> jnt_names;
    jnt_names.push_back("snake_joint");
    jnt_names.push_back("scorpion_joint");
    auto jnt_info_handle = std::make_shared<JntStsSubscriber>(jnt_names);

    ros::Subscriber jnt_status_subscriber = nh_.subscribe("/pan_tilt/joint_states", 1,&JntStsSubscriber::GetMsgCB,jnt_info_handle.get());

    e_sum[0] = 0;
    e_sum[1] = 0;
    ROS_INFO("Starting Feedback control Action Server");
    auto myaction = new cmd_actionAction(nh_, "cmd_action", jnt_info_handle, 10);
    myaction->StartAction();
    while(ros::ok()){
        ros::spinOnce();
    }
    delete myaction;
    return 0;
}