#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_conversions/kdl_msg.h>

#include "lwr_controllers/cartesian_impedance_controller.h"

namespace lwr_controllers 
{
    CartesianImpedanceController::CartesianImpedanceController() {}
    CartesianImpedanceController::~CartesianImpedanceController() {}

    bool CartesianImpedanceController::init(hardware_interface::PositionCartesianInterface *robot, ros::NodeHandle &n)
    {
        ROS_INFO("THIS CONTROLLER USES THE TCP INFORMATION SET ON THE FRI SIDE... SO BE SURE YOU KNOW WHAT YOU ARE DOING!");

        if (!ros::param::search(n.getNamespace(),"robot_name", robot_namespace_))
        {
            ROS_WARN_STREAM("CartesianImpedanceController: No robot name found on parameter server ("<<n.getNamespace()<<"/robot_name), using the namespace...");
            robot_namespace_ = n.getNamespace();
            //return false;
        }
        if (!n.getParam("robot_name", robot_namespace_))
        {
            ROS_WARN_STREAM("CartesianImpedanceController: Could not read robot name from parameter server ("<<n.getNamespace()<<"/robot_name), using the namespace...");
            robot_namespace_ = n.getNamespace();
            //return false;
        }

        joint_names_.push_back( robot_namespace_ + std::string("_0_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_1_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_2_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_3_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_4_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_5_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_6_joint") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xx") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yx") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zx") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_pos_x") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xy") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yy") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zy") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_pos_y") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_xz") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_yz") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_rot_zz") );
        cart_12_names_.push_back( robot_namespace_ + std::string("_pos_z") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_X") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_Y") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_Z") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_A") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_B") );
        cart_6_names_.push_back( robot_namespace_ + std::string("_C") );

        // now get all handles, 12 for cart pos, 6 for stiff, 6 for damp, 6 for wrench, 6 for joints
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.push_back(robot->getHandle(cart_12_names_.at(c)));
            if(c > 11 && c < 18)
                cart_handles_.push_back(robot->getHandle(cart_6_names_.at(c-12) + std::string("_stiffness")));
            if(c > 17 && c < 24)
                cart_handles_.push_back(robot->getHandle(cart_6_names_.at(c-18) + std::string("_damping")));
            if(c > 23 && c < 30)
                cart_handles_.push_back(robot->getHandle(cart_6_names_.at(c-24) + std::string("_wrench")));
        }
        for(int j = 0; j < 6; ++j)
        {
            joint_handles_.push_back(robot->getHandle(joint_names_.at(j)));
        }

        sub_command_ = n.subscribe(n.resolveName("command"), 1, &CartesianImpedanceController::command, this);
        srv_command_ = n.advertiseService("set_command", &CartesianImpedanceController::command_cb, this);
        sub_ft_measures_ = n.subscribe(n.resolveName("ft_measures"), 1, &CartesianImpedanceController::updateFT, this);
        pub_goal_ = n.advertise<geometry_msgs::PoseStamped>(n.resolveName("goal"),0);

        return true;
    }

    void CartesianImpedanceController::starting(const ros::Time& time)
    {
        KDL::Rotation cur_R(cart_handles_.at(0).getPosition(),
                            cart_handles_.at(1).getPosition(),
                            cart_handles_.at(2).getPosition(),
                            cart_handles_.at(4).getPosition(),
                            cart_handles_.at(5).getPosition(),
                            cart_handles_.at(6).getPosition(),
                            cart_handles_.at(8).getPosition(),
                            cart_handles_.at(9).getPosition(),
                            cart_handles_.at(10).getPosition());
        KDL::Vector cur_p(cart_handles_.at(3).getPosition(),
                            cart_handles_.at(7).getPosition(),
                            cart_handles_.at(11).getPosition());
        KDL::Frame cur_T( cur_R, cur_p );
        x_ref_ = cur_T;
        x_des_ = cur_T;

        // Initial Cartesian stiffness
        KDL::Stiffness k( 800.0, 800.0, 800.0, 50.0, 50.0, 50.0 );
        k_des_ = k;

        // Initial force/torque measure
        KDL::Wrench w(KDL::Vector(0.0, 0.0, 0.0), KDL::Vector(0.0, 0.0, 0.0));
        f_des_ = w;

    }

    void CartesianImpedanceController::command(const lwr_controllers::CartesianImpedancePoint::ConstPtr &msg)
    {
        // Validate command, for now, only check non-zero of stiffness, damping, and orientation


        // Compute a KDL frame out of the message
        tf::poseMsgToKDL( msg->x_FRI, x_des_ );

        // Convert Wrench msg to KDL wrench
        tf::wrenchMsgToKDL( msg->f_FRI, f_des_ );

        // Convert from Stiffness msg array to KDL stiffness
        //if(!(msg->k_FRI.x + msg->k_FRI.y + msg->k_FRI.z + msg->k_FRI.rx + msg->k_FRI.ry + msg->k_FRI.rz == 0.0))
        //{
            ROS_INFO("Updating Stiffness command");
            KDL::Stiffness k( msg->k_FRI.x, msg->k_FRI.y, msg->k_FRI.z, msg->k_FRI.rx, msg->k_FRI.ry, msg->k_FRI.rz );
            k_des_ = k;
        //}

        // publishe goal
        geometry_msgs::PoseStamped goal;
        goal.header = msg->header;
        goal.pose = msg->x_FRI;
        pub_goal_.publish(goal);

    }

    bool CartesianImpedanceController::command_cb(SetCartesianImpedanceCommand::Request &req, SetCartesianImpedanceCommand::Response &res)
    {
        return true;
    }

    void CartesianImpedanceController::updateFT(const geometry_msgs::WrenchStamped::ConstPtr &msg)
    {
        // Convert Wrench msg to KDL wrench
        geometry_msgs::Wrench f_meas = msg->wrench;
        tf::wrenchMsgToKDL( f_meas, f_cur_);
    }

    void CartesianImpedanceController::update(const ros::Time& time, const ros::Duration& period)
    {
        // get current values
        std::cout << "Update current values" << std::endl;
        KDL::Rotation cur_R(cart_handles_.at(0).getPosition(),
                            cart_handles_.at(1).getPosition(),
                            cart_handles_.at(2).getPosition(),
                            cart_handles_.at(4).getPosition(),
                            cart_handles_.at(5).getPosition(),
                            cart_handles_.at(6).getPosition(),
                            cart_handles_.at(8).getPosition(),
                            cart_handles_.at(9).getPosition(),
                            cart_handles_.at(10).getPosition());
        KDL::Vector cur_p(cart_handles_.at(3).getPosition(),
                            cart_handles_.at(7).getPosition(),
                            cart_handles_.at(11).getPosition());
        KDL::Frame cur_T( cur_R, cur_p );
        x_cur_ = cur_T;


        std::vector<double> cur_T_FRI;

        fromKDLtoFRI(x_des_, cur_T_FRI);
        // forward commands to hwi
        std::cout << "Before forwarding the command" << std::endl;
        for(int c = 0; c < 30; ++c)
        {
            if(c < 12)
                cart_handles_.at(c).setCommand(cur_T_FRI.at(c));
            if(c > 11 && c < 18)
                cart_handles_.at(c).setCommand(k_des_[c-12]);
            if(c > 17 && c < 24)
                cart_handles_.at(c).setCommand(d_des_[c-18]);
            if(c > 23 && c < 30)
                cart_handles_.at(c).setCommand(f_des_[c-24]);
        }
    }

    void CartesianImpedanceController::stopping(const ros::Time& time)
    {

    }

    void CartesianImpedanceController::multiplyJacobian(const KDL::Jacobian& jac, const KDL::Wrench& src, KDL::JntArray& dest)
    {
        Eigen::Matrix<double,6,1> w;
        w(0) = src.force(0);
        w(1) = src.force(1);
        w(2) = src.force(2);
        w(3) = src.torque(0);
        w(4) = src.torque(1);
        w(5) = src.torque(2);

        Eigen::MatrixXd j(jac.rows(), jac.columns());
        j = jac.data;
        j.transposeInPlace();

        Eigen::VectorXd t(jac.columns());
        t = j*w;

        dest.resize(jac.columns());
        for (unsigned i=0; i<jac.columns(); i++)
        dest(i) = t(i);
    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Frame& in, std::vector<double>& out)
    {
        out.resize(12);
        out.at(0) = in.M.UnitX().x();
        out.at(1) = in.M.UnitY().x();
        out.at(2) = in.M.UnitZ().x();
        out.at(3) = in.p.x();
        out.at(4) = in.M.UnitX().y();
        out.at(5) = in.M.UnitY().y();
        out.at(6) = in.M.UnitZ().y();
        out.at(7) = in.p.y();
        out.at(8) = in.M.UnitX().z();
        out.at(9) = in.M.UnitY().z();
        out.at(10) = in.M.UnitZ().y();
        out.at(11) = in.p.z();

    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Stiffness& in, std::vector<double>& out)
    {
        out.resize(6);
        for( int d = 0; d < 6; ++d)
        {
            out.at(d) = in[d];
        }
    }

    void CartesianImpedanceController::fromKDLtoFRI(const KDL::Wrench& in, std::vector<double>& out)
    {
        out.resize(6);
        out.at(0) = in.force.x();
        out.at(1) = in.force.y();
        out.at(2) = in.force.z();
        out.at(3) = in.torque.x();
        out.at(4) = in.torque.y();
        out.at(5) = in.torque.z();
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Frame& out)
    {
        KDL::Rotation R(in.at(0), in.at(1), in.at(2), in.at(4), in.at(5), in.at(6), in.at(8), in.at(9), in.at(10));
        KDL::Vector p(in.at(3), in.at(7), in.at(11));
        KDL::Frame T(R, p);
        out = T;
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Stiffness& out)
    {
        KDL::Stiffness s(in.at(0), in.at(1), in.at(2), in.at(3), in.at(4), in.at(5));
        out = s;
    }

    void CartesianImpedanceController::fromFRItoKDL(const std::vector<double>& in, KDL::Wrench& out)
    {
        KDL::Wrench w(KDL::Vector(in.at(0), in.at(1), in.at(2)), KDL::Vector(in.at(3), in.at(4), in.at(5)));
        out = w;
    }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianImpedanceController, controller_interface::ControllerBase)
