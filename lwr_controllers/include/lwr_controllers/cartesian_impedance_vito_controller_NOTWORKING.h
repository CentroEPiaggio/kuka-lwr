#ifndef LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_VITO_CONTROLLER_H
#define LWR_CONTROLLERS__CARTESIAN_IMPEDANCE_VITO_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <lwr_controllers/PoseRPY.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <lwr_controllers/Stiffness.h>
#include <tf/transform_listener.h>
#include <realtime_tools/realtime_publisher.h>

#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>
#include <kdl_conversions/kdl_msg.h>

#include "lwr_controllers/KinematicChainControllerBase.h"
#include "lwr_controllers/SetCartesianImpedanceCommand.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <urdf/model.h>
#include <utils/skew_symmetric.h>
#include <fstream>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include <math.h>
#include <tf/transform_datatypes.h>



namespace lwr_controllers {
	class CartesianImpedanceVitoController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface> {
	public:
		CartesianImpedanceVitoController();
		~CartesianImpedanceVitoController();

		bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		//void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void command(const lwr_controllers::PoseRPY::ConstPtr &msg);
		void setParam(const std_msgs::Float64MultiArray::ConstPtr &msg, KDL::JntArray* array, std::string s);
		
		Eigen::Matrix<double,3,3> quat_rot(double x, double y, double z, double w);
		Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1> v);
        Eigen::Matrix<double,6,6> K_var_(Eigen::Matrix<double,6,1> e);
        Eigen::Matrix<double,6,6> D_var_(Eigen::Matrix<double,6,1> e);
		int sign(double n);
        double norm(Eigen::VectorXd v);
	    void load_gains();

	private:
        bool firsttime = true;

            KDL::JntArray q_des__;
        KDL::JntArray K_des, D_des;

		bool com_torque = false;

        Eigen::Matrix<double,6,6> K_, D_, M_mms_;
		
		Eigen::Matrix<double,7,7> Kp_ct_, Kv_ct_;
		Eigen::Matrix<double,7,1> q_des_ct_, q_des_ct_step_, q0_;

		Eigen::Matrix<double,7,1> tau_; 
		
        Eigen::Matrix<double,6,1> xVEC_; 
        Eigen::Matrix<double,6,1> xDES_;
        Eigen::Matrix<double,3,1> rpy;

        Eigen::Matrix<double,3,1> k_p;
        Eigen::Matrix<double,3,1> k_o;
        Eigen::Matrix<double,3,1> d_p;
        Eigen::Matrix<double,3,1> d_o;

        Eigen::Matrix<double,3,1> quat_des_vec_;
        Eigen::Matrix<double,3,1> quat_vec_;
        Eigen::Matrix<double,3,1> quat_temp_;
        Eigen::Matrix<double,3,1> quat_old_;
        double quat_des_scal_, quat_scal_;

        Eigen::MatrixXd J_an_pinv_;
        Eigen::MatrixXd J_ge_pinv_;

        double N;
        bool primo = true;
        tf::Quaternion quat_0;
        tf::Quaternion quat_f;
        tf::Quaternion quat_t = tf::Quaternion(0, 0, 0, 1); 
        Eigen::Matrix<double,6,1> x0;
        Eigen::Matrix<double,6,1> xDES_step_;
        int counter;

        std::string robot_namespace_;
        
		KDL::JntArray q_des_;

        ros::Subscriber sub_command_;
        
        KDL::Frame x_;
		KDL::Frame x_des_;	//desired pose
		KDL::Twist x_des_dot_;
		KDL::Twist x_des_dotdot_;

		KDL::JntSpaceInertiaMatrix M_;	// intertia matrix
		KDL::JntArray C_;	// coriolis
		KDL::JntArray G_;	// gravity

        KDL::Jacobian J_;	//Jacobian J(q)
		KDL::Jacobian J_ge_;	//Jacobian J(q)
		KDL::Jacobian J_an_;	//Jacobian J(q)
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;	//d/dt(J(q))

		Eigen::Matrix<double,6,1> e_ref_;
        Eigen::Matrix<double,6,1> e_ref_dot_;
        Eigen::Matrix<double,6,6> T_;
        
		int msg_id_;
		int cmd_flag_;
		
		lwr_controllers::PoseRPY msg_pose_;

        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

    protected:
        std::vector<std::string> joint_names_, cart_12_names_, cart_6_names_;

        std::vector<hardware_interface::JointHandle> joint_handles_left_;
        std::vector<hardware_interface::JointHandle> joint_handles_right_;
        bool publish_cartesian_pose_;
		
        // ROS API (topic, service and dynamic reconfigure)
        ros::ServiceServer srv_command_;
        ros::Subscriber sub_ft_measures_;
        ros::Publisher pub_pose_;
        boost::shared_ptr< realtime_tools::RealtimePublisher< geometry_msgs::PoseStamped > > realtime_pose_pub_;
        
        // Transformation from robot base to controller base
        KDL::Frame robotBase_controllerBase_;
        
        // FRI vars
        std::vector<double> cur_T_FRI_;

        // Measured external force
        KDL::Wrench f_cur_;
        KDL::Wrench f_des_;
        // KDL::Wrench f_error_;

		// That is, desired kx, ky, kz, krx, kry, krz, they need to be expressed in the same ref as x, and J
		KDL::Stiffness k_des_;
        KDL::Stiffness d_des_;

        // Because of the lack of the Jacobian transpose in KDL
        void multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest);
        
        // Utility function to get the current pose
        void getCurrentPose(KDL::Frame& f);
        
        // Utility function to publish the current pose
        void publishCurrentPose(const KDL::Frame& f);
        
        // Utility function to forward commands to FRI
        void forwardCmdFRI(const KDL::Frame& f);

        // FRI<->KDL conversion
        void fromKDLtoFRI(const KDL::Frame& in, std::vector<double>& out);
        void fromKDLtoFRI(const KDL::Stiffness& in, std::vector<double>& out);
        void fromKDLtoFRI(const KDL::Wrench& in, std::vector<double>& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out);
        void fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out);
	};
} // namespace

#endif
