// ROS CONTROL HARDWARE INTERFACE
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// FRI standford 
#include <FastResearchInterface.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>


class MyLWR : public hardware_interface::RobotHW
{
public:
  MyLWR() 
 { 
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_6("arm_6_joint", &pos[6], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_0);

    hardware_interface::JointStateHandle state_handle_1("arm_1_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("arm_2_joint", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_2);

    hardware_interface::JointStateHandle state_handle_3("arm_3_joint", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_3);

    hardware_interface::JointStateHandle state_handle_5("arm_5_joint", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_5);

    hardware_interface::JointStateHandle state_handle_5("arm_5_joint", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_5);

    hardware_interface::JointStateHandle state_handle_6("arm_6_joint", &pos[6], &vel[6], &eff[6]);
    jnt_state_interface.registerHandle(state_handle_6);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_0(jnt_state_interface.getHandle("arm_0_joint"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_0);

    hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("arm_1_joint"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_1);

    hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("arm_2_joint"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_2);

    hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("arm_3_joint"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_3);

    hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("arm_4_joint"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_4);

    hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("arm_5_joint"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_5);

    hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("arm_6_joint"), &cmd[6]);
    jnt_pos_interface.registerHandle(pos_handle_6);

    registerInterface(&jnt_pos_interface);
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

main()
{
  MyLWR robot;
  controller_manager::ControllerManager cm(&robot);

  while (true)
  {
     robot.read();
     cm.update(robot.get_time(), robot.get_period());
     robot.write();
     sleep();
  }
}