# pan_tilt_pi_ctrl
urdf model of pan/tilt robot and control joints with feedback control action to hardware interface

execute these instructions in order

1. Launch pan_tilt_pi_ctrl view_sagal_pan_tilt.launch
2. rosrun pan_tilt_pi_ctrl pan_tilt_pi_ctrl_cmd_server
3. rosrun pan_tilt_pi_ctrl pan_tilt_pi_ctrl_cmd_client (position to reach)


1: Spawn urdf model on robot_description and register hardware interface and execute joint_state_publisher/controller manager
 (includes torque-velocity-position dynamics) and invoke rviz
2: Action server executes PI control with sending torque instruction to joint/command
3: Action client to send a reference position to reach
