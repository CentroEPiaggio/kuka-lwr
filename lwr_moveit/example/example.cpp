#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_lwr_planner");
    ros::NodeHandle nh;

    // execution service
    std::string planning_service_name("/lwr_planning_srv");
    if ( !ros::service::waitForService(planning_service_name, ros::Duration().fromSec(1.0)) )
    { 
      ROS_ERROR("After one second, the service %s hasn't shown up...",  planning_service_name.c_str());
      return (-1);     
    }

    // create a test trajectory
    definitions::Grasp grasp;
    ros::Duration five_seconds(5.0);

    // resize with the number of waypoints you want to test
    grasp.grasp_trajectory.resize(1);

    // there is suppose to be solution for this pose, since it was taken from the moveit-gui
    grasp.grasp_trajectory[0].wrist_pose.pose.position.x = 0.114;
    grasp.grasp_trajectory[0].wrist_pose.pose.position.y = 0.349;
    grasp.grasp_trajectory[0].wrist_pose.pose.position.z = 0.445;
    grasp.grasp_trajectory[0].wrist_pose.pose.orientation.x = 0.368;
    grasp.grasp_trajectory[0].wrist_pose.pose.orientation.y = 0.687;
    grasp.grasp_trajectory[0].wrist_pose.pose.orientation.z = 0.600;
    grasp.grasp_trajectory[0].wrist_pose.pose.orientation.w = -0.182;

    // create the service instance
    definitions::TrajectoryPlanning trajectory_planning_srv;
    trajectory_planning_srv.request.ordered_grasp.push_back(grasp);

    // call the service with the instance
    ROS_INFO("Calling the service");
    if ( !ros::service::call( planning_service_name, trajectory_planning_srv) )
    { 
        ROS_ERROR("Call to the service %s failed.", planning_service_name.c_str());  
        return (-1);
    }   

    if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.OTHER_ERROR)
    {   
        ROS_ERROR("Unable to execute the trajectory");
        return (-1);
    }

    if (trajectory_planning_srv.response.result == trajectory_planning_srv.response.SUCCESS)
    { 
        ROS_INFO("Trajectory Execution OK...\n");
        return 0;
    }

    return 0;
}