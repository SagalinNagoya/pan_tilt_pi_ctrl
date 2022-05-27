#include "jnt_sts_subscriber.h"

JntStsSubscriber::JntStsSubscriber(const std::vector<std::string>& _jnt_names)
:num_jnts(_jnt_names.size()){
    id.reserve(num_jnts);
    pos.reserve(num_jnts);
    vel.reserve(num_jnts);
    eff.reserve(num_jnts);
    struct JointNameID{
        std::string Joint_name;
        size_t input_order;
    };
    std::vector<JointNameID> joint_id_container;
    joint_id_container.reserve(num_jnts);
    for(size_t i=0; i<num_jnts;++i){
        JointNameID element;
        element.Joint_name = _jnt_names[i];
        element.input_order = i;
        joint_id_container.push_back(std::move(element));
    }
    std::sort(joint_id_container.begin(), joint_id_container.end(), []
        (const JointNameID& _left, const JointNameID& _right)->bool{
            return _left.Joint_name < _right.Joint_name; /*Order by joint names to get message order*/
        });
    for(size_t i=0; i<num_jnts;++i){
        ROS_INFO("%s is was %d th and now %d th",joint_id_container[i].Joint_name.c_str(), static_cast<unsigned int>(joint_id_container[i].input_order), static_cast<unsigned int>(i));
        /*Store the sort information*/
        id.push_back(joint_id_container[i].input_order);
    }
}

void JntStsSubscriber::GetMsgCB(const boost::shared_ptr<sensor_msgs::JointState>& _jnt_state){
    for(size_t i=0; i<num_jnts;++i){
        /*Get input with inverse-sorting*/
        pos[i] = _jnt_state->position[id[i]];
        vel[i] = _jnt_state->velocity[id[i]];
        eff[i] = _jnt_state->effort[id[i]];
        //ROS_INFO("Get msg: %lf", pos[i]);
    }
}
