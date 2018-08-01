#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <lwr_controllers/cartesian_impedance_vito_controller.h>

namespace lwr_controllers {

CartesianImpedanceVitoController::CartesianImpedanceVitoController() {
        log_tau_meas.open("tau_meas.csv");
        log_tau_des.open("tau_des.csv");
        log_qerr.open("qerr.csv");
        log_qdot.open("qdot.csv");
        timer = 0;
}

CartesianImpedanceVitoController::~CartesianImpedanceVitoController() {
        log_tau_meas.close();
        log_tau_des.close();
        log_qerr.close();
        log_qdot.close();

}

bool CartesianImpedanceVitoController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());   
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());
    q_start_.resize(kdl_chain_.getNrOfJoints());

    tau_.resize(kdl_chain_.getNrOfJoints());

    K_joint.resize(kdl_chain_.getNrOfJoints());
    D_joint.resize(kdl_chain_.getNrOfJoints());

    K_joint(0) = 50.0;
    K_joint(1) = 50.0;
    K_joint(2) = 10.0;
    K_joint(3) = 5.0;
    K_joint(4) = 5.0;
    K_joint(5) = 5.0;
    K_joint(6) = 5.0;

    D_joint(0) = 10;
    D_joint(1) = 10;
    D_joint(2) = 7;
    D_joint(3) = 7;
    D_joint(4) = 1;
    D_joint(5) = 1;
    D_joint(6) = 1;

    //x_des_.resize(6);
    //x_meas_.resize(6);

    K_cart.resize(6);
    K_cart(0) = 100.0;
    K_cart(1) = 1000.0;
    K_cart(2) = 1000.0;
    K_cart(3) = 0.0;
    K_cart(4) = 0.0;
    K_cart(5) = 0.0;

    D_cart.resize(6);
    D_cart(0) = 10.0;
    D_cart(1) = 100.0;
    D_cart(2) = 100.0;
    D_cart(3) = 0.0;
    D_cart(4) = 0.0;
    D_cart(5) = 0.0;


    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_des_(i) = joint_handles_[i].getPosition();
        K_(i) = joint_stiffness_handles_[i].getPosition();
        D_(i) = joint_damping_handles_[i].getPosition();
        q_des_(i) = joint_set_point_handles_[i].getPosition();

        tau_(i) = joint_handles_[i].getEffort();
        q_start_(i) = joint_handles_[i].getPosition();
    }

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("stiffness_gains", K_(i) ) ) {
            ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
        }
    }
    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("damping_gains", D_(i)) ) {
            ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
        }
    }

    typedef  const std_msgs::Float64MultiArray::ConstPtr& msg_type;
    sub_stiffness_ = nh_.subscribe<CartesianImpedanceVitoController, msg_type>("stiffness", 1, boost::bind(&CartesianImpedanceVitoController::setParam, this, _1, &K_, "K"));
    sub_damping_ = nh_.subscribe<CartesianImpedanceVitoController, msg_type>("damping", 1, boost::bind(&CartesianImpedanceVitoController::setParam, this, _1, &D_, "D"));
    sub_add_torque_ = nh_.subscribe<CartesianImpedanceVitoController, msg_type>("additional_torque", 1, boost::bind(&CartesianImpedanceVitoController::setParam, this, _1, &tau_des_, "AddTorque"));
    sub_posture_ = nh_.subscribe("command", 1, &CartesianImpedanceVitoController::command, this);


    // kinematics and dynamics stuff
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    KDL::Rotation world_to_base_right_arm = KDL::Rotation::RPY(3.1415926535897932384626, -3.1415926535897932384626/4.0, 0.0);
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,world_to_base_right_arm.Inverse()*gravity_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    J_ge_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());

    CartesianForces.resize(6);

    return true;


}

void CartesianImpedanceVitoController::starting(const ros::Time& time)
{
    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        tau_des_(i) = 0.0;
        tau_(i) = joint_handles_[i].getEffort();
        ROS_INFO_STREAM("starting Effort [" << i << "] " << joint_handles_[i].getEffort());
        q_des_(i) = joint_handles_[i].getPosition();
    }

    for (size_t i = 0; i < joint_handles_.size(); i++) {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        ROS_INFO_STREAM("Effort [" << i << "] " << joint_handles_[i].getEffort());
    }
    
    // posizione dei giunti del robot all'inizio della simulazione
    q0_ = q_start_.data;

    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);
    id_solver_->JntToGravity(joint_msr_states_.q, G_);

    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_);
    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_old_);


}

void CartesianImpedanceVitoController::update(const ros::Time& time, const ros::Duration& period)
{

    // get joints positions
    for (int i = 0; i < joint_handles_.size(); i++) {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }
    // compute gravity term
    id_solver_->JntToGravity(joint_msr_states_.q, G_);
    id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
 
    // compute Jacobian
    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_ge_);

    ROS_INFO_STREAM(J_ge_.data);
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_(i) = G_(i) + C_(i);
    }

    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_);
    for(int i = 0; i < 6; i++) {
        if(i<3)
        {
            CartesianForces(i) = K_cart(i)*(x_des_.p(i) - x_meas_.p(i)) - D_cart(i)*(x_meas_.p(i) - x_meas_old_.p(i))/period.toSec();
            //ROS_INFO_STREAM("Update Certesian error: " << (x_des_.p(i) - x_meas_.p(i)) << " x_des_.p(i) " << x_des_.p(i) << " x_meas_.p(i) " <<  x_meas_.p(i));
        }
        else
        {
            CartesianForces(i) = 0.0;
        }
        
    }

    for(size_t i = 0; i < joint_handles_.size(); i++) {
        tau_(i) = 0;
        for(size_t j = 0; j < 6; j++) {
            tau_(i) += J_ge_(j,i)*CartesianForces(j);
        }
    }
    //ROS_INFO_STREAM("CartesianForces\n\n" << CartesianForces.data << "\n\n");
    log_tau_meas << timer << ",";
    log_tau_des << timer << ",";
    timer++;
    //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        //joint_handles_[i].setCommand(tau_des_(i));
        float q_error = q_start_(i) - joint_handles_[i].getPosition();
        //float q_vel = joint_handles_[i].getVelocity();
        //tau_(i) = K_(i)*(q_start_(i) - joint_handles_[i].getPosition()) + G_(i) + C_(i);
        //tau_(i) = G_(i) + C_(i);
        
        // working
        //tau_(i) = K_joint(i)*(q_start_(i) - joint_handles_[i].getPosition()) - D_joint(i) * joint_handles_[i].getVelocity(); // Working
        // working

        //tau_(i) -= joint_handles_[i].getEffort();

        //ROS_INFO_STREAM("Update Joint error: " << q_error << " q_start_(i) " << q_start_(i) << " joint_handles_[i].getPosition() " <<  joint_handles_[i].getPosition() );
        //tau_(i) = joint_handles_[i].getEffort();
        joint_handles_[i].setCommand(tau_(i));
        // ROS_INFO_STREAM("Effort [" << i << "] " << joint_handles_[i].getEffort() << " tau_(i) = " << tau_(i));
        ROS_INFO_STREAM(" tau_(i) = " << tau_(i));
        // ROS_INFO_STREAM("K [" << i << "] " << K_(i));

        joint_stiffness_handles_[i].setCommand(0.0*K_(i));
        joint_damping_handles_[i].setCommand(0.0*D_(i));
        //joint_set_point_handles_[i].setCommand(0.0*q_des_(i));
        joint_set_point_handles_[i].setCommand(q_start_(i));
        //joint_set_point_handles_[i].setCommand(joint_handles_[i].getPosition());

        log_tau_meas << joint_handles_[i].getEffort() << ",";
        log_tau_des << tau_(i) << ",";
        log_qerr << q_start_(i) - joint_handles_[i].getPosition() << ",";
        log_qdot << -joint_handles_[i].getVelocity() << ",";
    }
    log_tau_meas << "\n";
    log_tau_des << "\n";
    log_qerr << "\n";
    log_qdot << "\n";

    x_meas_old_ = x_meas_;
    ROS_INFO_STREAM(" ");
}


void CartesianImpedanceVitoController::command(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if ((int)msg->data.size() != joint_handles_.size()) {
        ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        for (unsigned int j = 0; j < joint_handles_.size(); ++j)
            q_des_(j) = msg->data[j];
    }

}

void CartesianImpedanceVitoController::setParam(const std_msgs::Float64MultiArray_< std::allocator< void > >::ConstPtr& msg, KDL::JntArray* array, std::string s)
{
    if (msg->data.size() == joint_handles_.size())
    {
        for (unsigned int i = 0; i < joint_handles_.size(); ++i)
        {
            (*array)(i) = msg->data[i];
        }
    }
    else
    {
        ROS_INFO("Num of Joint handles = %lu", joint_handles_.size());
    }

    ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

    ROS_INFO("New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
             (*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
}

} // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::CartesianImpedanceVitoController, controller_interface::ControllerBase)
