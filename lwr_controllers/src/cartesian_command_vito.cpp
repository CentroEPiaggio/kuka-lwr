#include "ros/ros.h"
#include <lwr_controllers/PoseRPY.h>
#include <kdl/tree.hpp>
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define PI 3.141592653

ros::Subscriber sub_terminal;

ros::Publisher pub_right;
ros::Publisher pub_left;

Eigen::Matrix<double,3,1> p_global_r, p_global_l, p_right, p_left;

tf::StampedTransform transform_right;
tf::StampedTransform transform_left;

double L = 0.1;

Eigen::Matrix<double,3,3> rot_right, rot_left;
Eigen::Matrix<double,3,3> rot_des_r, rot_des_l;
Eigen::Matrix<double,3,3> rot_fin_r, rot_fin_l;
Eigen::Matrix<double,3,1> pos_right, pos_left;

Eigen::Matrix<double,3,3> quat_rot(double x, double y, double z, double w) {
	Eigen::Matrix<double,3,3> temp;
	temp << 1-2*pow(y,2)-2*pow(z,2),             2*x*y-2*w*z,             2*x*z+2*w*y,
			            2*x*y+2*w*z, 1-2*pow(x,2)-2*pow(z,2),             2*z*y-2*w*x,
			            2*x*z-2*w*y,             2*z*y+2*w*x, 1-2*pow(x,2)-2*pow(y,2);
	return temp;
}

int sign(double n) {
	if (n > 0)
		return 1;
	else if (n < 0)
		return -1;
	else 
		return 0;
}

Eigen::Matrix<double,4,1> rot_quat(Eigen::Matrix<double,3,3> r) {
	Eigen::Matrix<double,4,1> temp;
	temp << 0.5*sign(r(2,1)-r(1,2))*sqrt(r(0,0)-r(1,1)-r(2,2)+1),
			0.5*sign(r(0,2)-r(2,0))*sqrt(r(1,1)-r(2,2)-r(0,0)+1),
			0.5*sign(r(1,0)-r(0,1))*sqrt(r(2,2)-r(0,0)-r(1,1)+1),
			0.5*sqrt(r(0,0)+r(1,1)+r(2,2)+1);
	return temp;
}

void gposeCallback(const lwr_controllers::PoseRPY::ConstPtr& msg) {
	KDL::Rotation rot_msg = KDL::Rotation::EulerZYX(msg->orientation.yaw, msg->orientation.pitch, msg->orientation.roll);
	double t1,t2,t3,t4;
	rot_msg.GetQuaternion(t1,t2,t3,t4);
	Eigen::Matrix<double,3,3> rot_msg_mat;
	rot_msg_mat = quat_rot(t1,t2,t3,t4);
	
	Eigen::Matrix<double,3,1> vec_r, vec_l;
	vec_r << 0, L/2, 0;
	vec_l << 0, -L/2, 0;
	p_global_r = rot_msg_mat*vec_r;
	p_global_l = rot_msg_mat*vec_l;
	p_global_r << msg->position.x + p_global_r(0), msg->position.y + p_global_r(1), msg->position.z + p_global_r(2);
	p_global_l << msg->position.x + p_global_l(0), msg->position.y + p_global_l(1), msg->position.z + p_global_l(2);
	
	p_right = rot_right*p_global_r + pos_right;
	p_left = rot_left*p_global_l + pos_left;
	
	KDL::Rotation rot_glob = KDL::Rotation::EulerZYX(msg->orientation.yaw,msg->orientation.pitch,msg->orientation.roll);
	KDL::Rotation rot_des_r_kdl = rot_glob;
	rot_des_r_kdl.DoRotX(PI/2);
	KDL::Rotation rot_des_l_kdl = rot_glob;
	rot_des_l_kdl.DoRotX(-PI/2);
	
	Eigen::Matrix<double,3,3> rot_des_r, rot_des_l;
	
	rot_des_r_kdl.GetQuaternion(t1,t2,t3,t4);
	rot_des_r = quat_rot(t1,t2,t3,t4);
	rot_des_l_kdl.GetQuaternion(t1,t2,t3,t4);
	rot_des_l = quat_rot(t1,t2,t3,t4);
	
	rot_fin_r = rot_right*rot_des_r;
	rot_fin_l = rot_left*rot_des_l;
	
	Eigen::Matrix<double,4,1> r, l;
	r = rot_quat(rot_fin_r);
	l = rot_quat(rot_fin_l);
	KDL::Rotation q_r = KDL::Rotation::Quaternion(r(0),r(1),r(2),r(3));
	KDL::Rotation q_l = KDL::Rotation::Quaternion(l(0),l(1),l(2),l(3));
	
	lwr_controllers::PoseRPY msg_right;
	lwr_controllers::PoseRPY msg_left;
	
	q_r.GetEulerZYX(msg_right.orientation.yaw, msg_right.orientation.pitch, msg_right.orientation.roll);
	q_l.GetEulerZYX(msg_left.orientation.yaw, msg_left.orientation.pitch, msg_left.orientation.roll);
	
	msg_left.id = 0;
	msg_left.position.x = p_left(0);
	msg_left.position.y = p_left(1);
	msg_left.position.z = p_left(2);
	
	msg_right.id = 0;
	msg_right.position.x = p_right(0);
	msg_right.position.y = p_right(1);
	msg_right.position.z = p_right(2);
	
	pub_right.publish(msg_right);
	pub_left.publish(msg_left);
}

int main(int argc, char **argv) {
	// Initialize the node
	ros::init(argc, argv, "command_vito");
	ros::NodeHandle node;

// 	rot_des_l <<   0, -1, 0,
// 				   0,  0, 1,
// 				  -1,  0, 0;
// 	rot_des_r <<  1, 0,  0,
// 				  0, 0, -1,
// 				  0, 1,  0;
	
	tf::TransformListener listener_right;
	tf::TransformListener listener_left;
	transform_right.setIdentity();
	transform_left.setIdentity();
	sleep(5.0);
	while (transform_right.getOrigin().getX() == 0 && transform_left.getOrigin().getX() == 0) {
		try{
			listener_right.lookupTransform("world", "right_arm_base_link", ros::Time(0), transform_right);
			listener_left.lookupTransform("world", "left_arm_base_link", ros::Time(0), transform_left);
		} catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
	}
	
	rot_right = quat_rot(transform_right.inverse().getRotation().getX(),
						 transform_right.inverse().getRotation().getY(),
						 transform_right.inverse().getRotation().getZ(),
						 transform_right.inverse().getRotation().getW());
	rot_left = quat_rot(transform_left.inverse().getRotation().getX(),
						transform_left.inverse().getRotation().getY(),
						transform_left.inverse().getRotation().getZ(),
						transform_left.inverse().getRotation().getW());
	pos_right << transform_right.inverse().getOrigin().getX(),
				 transform_right.inverse().getOrigin().getY(),
				 transform_right.inverse().getOrigin().getZ();
	pos_left << transform_left.inverse().getOrigin().getX(), 
				transform_left.inverse().getOrigin().getY(),
				transform_left.inverse().getOrigin().getZ();
	
	sub_terminal = node.subscribe("/global_pose", 30, &gposeCallback);

    pub_right = node.advertise<lwr_controllers::PoseRPY>("/right_arm/joint_impedance_controller/command", 30);
	pub_left = node.advertise<lwr_controllers::PoseRPY>("/left_arm/joint_impedance_controller/command", 30);
	
	sleep(0.1);
	// Loop at a specified frequency, publishing movement commands until we shut down
	ros::Rate rate(30);

	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
}