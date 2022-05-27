#ifndef ___SAGALBOT_H___
#define ___SAGALBOT_H___

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <vector>
#include <algorithm>
#include <sstream>
#include <fstream>

namespace sagalbot{
class MyRobot : public hardware_interface::RobotHW{
private:
    hardware_interface::JointStateHandle* jnt_state_handles;
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::JointHandle* jnt_pos_handles;
    hardware_interface::EffortJointInterface jnt_pos_interface;
    double* cmd;
    double* pos;
    double* vel;
    double* eff;
    std::vector<std::string> joint_list;
    void MyRobot_init();
public:
    MyRobot(const size_t& _num_joints=0);
    MyRobot(const std::string& _joint_list_file);
    MyRobot(const std::vector<std::string>& _joint_list_vector);
    ~MyRobot();
    void read(const unsigned int _dt_inv);
    void write();
};

}



#endif /*___SAGALBOT_H___*/