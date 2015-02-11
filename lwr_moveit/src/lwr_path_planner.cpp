//// system headers
#include <boost/shared_ptr.hpp> 

//// ros headers 
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit/kinematic_constraints/utils.h>

//// local headers
#include <definitions/TrajectoryPlanning.h>

namespace lwr_moveit {

class LWRPlanner
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    
    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    ros::ServiceServer srv_trajectory_planning_;
    ros::ServiceServer srv_test_trajectory_planning_;

    // clients
    ros::ServiceClient clt_moveit_planning_;

	// planning related parameters
	int max_points_in_trajectory_;
	int max_planning_attempts_;
	int max_planning_time_;
	double tolerance_in_position_;
	double tolerance_in_orientation_;

    // define the names passed in the urdf files corresponding to the current move group for planning
    std::string group_name_;
    std::string base_frame_for_goal_;
	std::string plan_for_frame_;

	// the variable where the plans are stored
	std::vector<moveit_msgs::MotionPlanResponse> motion_plans_;
  
  public:

  	// the actual planning function
    bool planTrajectory(std::vector<definitions::Trajectory> &trajectories, geometry_msgs::PoseStamped &goal);

    // constructor
    LWRPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {

		// wait for moveit to load
		std::string planning_service_name = "/plan_kinematic_path";

		ROS_INFO("Waiting for MoveIt! to fully load...");
		ros::service::waitForService(planning_service_name, -1);
		clt_moveit_planning_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>(planning_service_name);

		// planning related parameters
		nh_.param<int>("max_points_in_trajectory", max_points_in_trajectory_, 50);
		nh_.param<int>("max_planning_attempts", max_planning_attempts_, 2);
		nh_.param<int>("max_planning_time", max_planning_time_, 2);
		nh_.param<double>("tolerance_in_position", tolerance_in_position_, 0.1);
		nh_.param<double>("tolerance_in_orientation", tolerance_in_orientation_, 0.1);

        // define the names passed in the urdf files corresponding to the current move group for planning
        nh_.param<std::string>("arm_name", group_name_, "lwr");
        nh_.param<std::string>("base_frame_for_goal", base_frame_for_goal_, "lwr_base_link");
		nh_.param<std::string>("plan_for_frame", plan_for_frame_, "lwr_7_link");

		srv_trajectory_planning_ = nh_.advertiseService(nh_.resolveName("/lwr_planning_srv"),&LWRPlanner::planTrajectory, this);

    }

    //! Empty stub
    ~LWRPlanner() {}

};

bool LWRPlanner::planTrajectory(std::vector<definitions::Trajectory> &trajectories, geometry_msgs::PoseStamped &goal) 
{
	// first state the planning constraint
	goal.header.frame_id = base_frame_for_goal_.c_str(); // just to ensure we plan with respect to the base_frame
	moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(plan_for_frame_.c_str(), goal, tolerance_in_position_, tolerance_in_orientation_);

	ROS_INFO("Planning for wrist (px, py, pz, qx, qy, qz, qw):\t%f\t%f\t%f\t%f\t%f\t%f\t%f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w);

	// now construct the motion plan request
	moveit_msgs::GetMotionPlan motion_plan;
	moveit_msgs::MotionPlanRequest &motion_plan_request = motion_plan.request.motion_plan_request;

	motion_plan_request.group_name = group_name_.c_str();
	motion_plan_request.goal_constraints.push_back(pose_goal);
	motion_plan_request.num_planning_attempts = max_planning_attempts_;
	motion_plan_request.allowed_planning_time = max_planning_time_;

	// and call the service with the request
	ROS_INFO("Calling plannig service...");
	bool success = clt_moveit_planning_.call(motion_plan);

	// check results 
	if(success) 
	{
		moveit_msgs::MotionPlanResponse &motion_plan_response = motion_plan.response.motion_plan_response;
		int trajSize = (int)motion_plan_response.trajectory.joint_trajectory.points.size();

		ROS_INFO("Planning completed with code %d", motion_plan_response.error_code.val);
		ROS_INFO("Planning took %.2fs", motion_plan_response.planning_time);

		if(trajSize > max_points_in_trajectory_) 
		{
			ROS_WARN("Computed trajectory contains too many points, so it will be dropped!");
			return false;
		} 
		else 
		{
			ROS_INFO("Trajectory contains %d points.", trajSize);

			// // store the motion plan for later usage
			// int index = motion_plans.size();
			// moveit_msgs::MotionPlanResponse response = motion_plan.response.motion_plan_response;
			// motion_plans_.push_back(response);


			definitions::Trajectory trajectory;
			trajectory.trajectory_id = 0;

			moveit_msgs::RobotTrajectory robot_trajectory = motion_plan.response.motion_plan_response.trajectory;

			// populate trajectory with motion plan data
			convertFromMRobotTrajectoryToTrajectory(robot_trajectory, trajectory);
			trajectories.push_back(trajectory);

			return true;
		}
	} 
	else 
	{
		ROS_WARN("No solution found");
		return false;
	}
}


} // namespace lwr_moveit


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lwr_planner_node");
    ros::NodeHandle nh;

    lwr_moveit::LWRPlanner node(nh);

    ROS_INFO("This node is ready to do trejctory planning!");

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    return 0;
}