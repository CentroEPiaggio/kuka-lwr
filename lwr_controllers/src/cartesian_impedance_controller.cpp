#include <math.h>
#include <pluginlib/class_list_macros.h>
#include <kdl_conversions/kdl_msg.h>

#include "lwr_controllers/cartesian_impedance_controller.h"

namespace lwr_controllers 
{
    CartesianImpedanceController::CartesianImpedanceController() {}
    CartesianImpedanceController::~CartesianImpedanceController() {}

    bool CartesianImpedanceController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
        if( !(KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize CartesianImpedanceController controller.");
            return false;
        }

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

        // Resize according to parsed chain
        J_.resize(kdl_chain_.getNrOfJoints());

        // Update the initial values, note the mirror of the msr and des states
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // Computing the initial FK, and mirror desired as well
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_cur_);
        x_des_ = x_cur_;

        // Initial Cartesian stiffness
        KDL::Stiffness k( 1000, 1000, 1000, 200, 200, 200 );
        k_des_ = k;

        sub_command_ = nh_.subscribe("command", 1, &CartesianImpedanceController::command, this);
        sub_command_ = nh_.subscribe(nh_.resolveName("ft_measures"), 1, &CartesianImpedanceController::updateFT, this);

        return true;
    }

    void CartesianImpedanceController::starting(const ros::Time& time)
    {

    }

    void CartesianImpedanceController::command(const lwr_controllers::CartesianImpedanceReference::ConstPtr &msg)
    {
        // Compute a KDL frame out of the message
        tf::poseMsgToKDL( msg->x_FRI, x_des_ );

        // Convert Wrench msg to KDL wrench
        tf::wrenchMsgToKDL( msg->f_FRI, f_des_ );

        // Convert from Stiffness msg array to KDL stiffness
        KDL::Stiffness k( msg->k_FRI.x, msg->k_FRI.y, msg->k_FRI.z, msg->k_FRI.rx, msg->k_FRI.ry, msg->k_FRI.rz );
        k_des_ = k;
    }

    void CartesianImpedanceController::updateFT(const geometry_msgs::Wrench::ConstPtr &msg)
    {
        // Convert Wrench msg to KDL wrench
        tf::wrenchMsgToKDL( *msg, f_cur_);
    }

    void CartesianImpedanceController::update(const ros::Time& time, const ros::Duration& period)
    {
        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }

        // computing Jacobian at msr states
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

        // computing current frame kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_cur_);

        // end-effector position error
        x_err_ = KDL::diff(x_des_, x_cur_);

        // compute the wrench error according to k_des
        KDL::Wrench x_wrench;
        x_wrench = k_des_*x_err_;

        // compute the error in the force
        f_error_ = f_des_ - f_cur_;

        // sum up the wrench error contribution in x space
        x_wrench += f_error_;

        // compute the torque commands
        multiplyJacobian( J_, x_wrench, tau_cmd_ );

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand( tau_cmd_(i) );
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

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CartesianImpedanceController, controller_interface::ControllerBase)
