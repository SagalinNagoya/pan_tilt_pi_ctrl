#ifndef ___JNT___STS___SUB___H___
#define ___JNT___STS___SUB___H___

#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include <vector>
#include <algorithm>
#include <functional>


class JntStsSubscriber{
private:
    ros::Subscriber jnt_status_subscriber;
    const size_t num_jnts;   
public: 
    std::vector<size_t> id;
    std::vector<double> pos;    
    std::vector<double> vel;    
    std::vector<double> eff;    
    JntStsSubscriber(const std::vector<std::string>& _jnt_names);
    void GetMsgCB(const boost::shared_ptr<sensor_msgs::JointState>& _jnt_state);
};

#endif