
#ifndef VITO_CONTROLLERS__CARTESIAN_INPEDANCE_CONTROLLER_H
#define VITO_CONTROLLERS__CARTESIAN_INPEDANCE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

#include <fstream>

/*
    tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

namespace lwr_controllers
{

    class CartesianImpedanceVitoController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
    {
    public:

        CartesianImpedanceVitoController();
        ~CartesianImpedanceVitoController();

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);
        void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setParam(const std_msgs::Float64MultiArray::ConstPtr &msg, KDL::JntArray* array, std::string s);
        
    private:

        ros::Subscriber sub_stiffness_, sub_damping_, sub_add_torque_;
        ros::Subscriber sub_posture_;

        KDL::JntArray q_des_;
        KDL::JntArray tau_des_;
        KDL::JntArray K_, D_;
        KDL::JntArray tau_;
        KDL::JntArray q_start_;

        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

        KDL::Jacobian J_ge_;    //Jacobian J(q)
        KDL::JntSpaceInertiaMatrix M_;  // intertia matrix
        KDL::JntArray C_;   // coriolis
        KDL::JntArray G_;   // gravity

        KDL::JntArray CartesianForces;

        KDL::Frame x_des_;  //desired pose
        KDL::Frame x_meas_;  //measured pose (from Direct Kinematics)
        KDL::Frame x_meas_old_;  //measured pose (from Direct Kinematics)

        Eigen::Matrix<double,7,1> q0_;

        std::ofstream log_tau_meas, log_tau_des;
        std::ofstream log_qerr, log_qdot;
        long int timer;

        KDL::JntArray K_joint, D_joint;
        KDL::JntArray K_cart, D_cart;



    };

} // namespace

#endif
