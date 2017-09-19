// authors: Piero Guardati, Federico Nesti

#include <lwr_controllers/CLIK_controller.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <kdl/frames_io.hpp>
#include <tf_conversions/tf_kdl.h>

#include <yaml-cpp/yaml.h>
 
#include <math.h>


namespace lwr_controllers  
{
    CLIKController::CLIKController() {}
    CLIKController::~CLIKController() {}

    bool CLIKController::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize CLIKController controller.");
            return false;
        }

        // add end-effector in kdl chain -> extended_chain_
        extend_chain(n);

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(extended_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(extended_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(extended_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(extended_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
        id_solver_.reset( new KDL::ChainDynParam( extended_chain_, gravity_) ); //ricorda EXTENDED


        q_cmd_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());
        M_.resize(kdl_chain_.getNrOfJoints());

        // Get the initial joints from the rosparam server
        std::vector<double> home;
        n.getParam("joint_home", home);

        // Get joint positions
        ///for(int i=0; i < joint_handles_.size(); i++)
        //{
         //   joint_msr_states_.q(i) = joint_handles_[i].getPosition();
         //   joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
         //   joint_des_states_.q(i) = home.at(i);
      //  }

        // Initial Pos
        // Set controls for joints
        //for (unsigned int i = 0; i < joint_handles_.size(); i++)
       // {
            //joint_handles_[i].setCommand(joint_des_states_.q(i));
        //}



        // computing forward kinematics
       // fk_pos_solver_->JntToCart(joint_des_states_.q, x_);

        //Desired posture is the current one
        //x_des_ = x_;
     

        cmd_flag_ = 0;
        control_type = 0;
std::cout << "Default Control Strategy: Stack of Tasks, second task is to stay far from joints limits"<<std::endl;

        sub_command_ = nh_.subscribe("command", 1, &CLIKController::command, this);
        sub_control_type = nh_.subscribe("controller_type", 1, &CLIKController::controller_type, this);
            sub_command2 = nh_.subscribe("command2", 1, &CLIKController::command2, this);




        std::stringstream ManTopicStream;
        ManTopicStream << nh_.getNamespace() + "/initial_position_topic";

        ROS_INFO_STREAM(ManTopicStream.str());
        pub_initial_position = nh_.advertise<std_msgs::String>(ManTopicStream.str(), 1000);   //PEPIPA




            W_gain = MatrixXd::Zero(7,7);
            Eigen::VectorXd j_t(7);
            double j_norm;

            std::vector<double> weight_gains_;
            n.getParam("weight_gains",weight_gains_);

            for (int i =0;  i < joint_handles_.size(); i++)
            {
                    W_gain(i,i) = weight_gains_[i];
                    //W(i,i) = 1 / ( joint_limits_.max(i) - abs(joint_msr_states_.q(i)) );
                    //j_t(i) = 1 / ( joint_limits_.max(i) - abs(joint_msr_states_.q(i)) );
                    //j_t(i) = 7 - i;

            }



        return true;





    }

    void CLIKController::starting(const ros::Time& time)
    {
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_des_states_.q(i) = joint_handles_[i].getPosition();
        }
         fk_pos_solver_->JntToCart(joint_des_states_.q, x_des_);
    }

    void CLIKController::update(const ros::Time& time1, const ros::Duration& period)
    {
        double k_click = 1;
        nh_.param<double>("k_click",k_click, 1);

        std_msgs::String initial_position_msg; 
       //std::cout <<std::endl<< "control type: "<< std::to_string(control_type)<<std::endl;
        std::stringstream ss2;
        
       for (unsigned int i = 0; i < 4; i++) {
            
            for (unsigned int j = 0; j < 3; j++) {
                if (i < 3) 
                    ss2 << std::to_string(x_des_(i,j))<<" ";
                if (i == 3) 
                    ss2 << std::to_string(x_des_.p(j))<< " ";
                
               // std::cout << std::endl << std::to_string(x_initial.p(0))<< " " <<std::to_string(x_initial(3,1)) << std::to_string(x_initial(3,2));//<< std::to_string(x_initial(0,3));
            }

       }
        

        initial_position_msg.data = ss2.str();

        //Publish
        pub_initial_position.publish(initial_position_msg);
        
        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        }

        if (cmd_flag_)//
        {
            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);


            //pseudo_inverse(J_.data* W_gain *J_.data.transpose(), J_pinv_);

            //J_pinv_ = W_gain * J_.data.transpose() *J_pinv_;

            switch(control_type) //scelta pseudo inv
            {
/*
                case 0: // multi task (lontano da fine corsa)
                pseudo_inverse(J_.data, J_pinv_);
                break;

                case 1: //multi task (gomito basso)
                pseudo_inverse(J_.data, J_pinv_);
                break;

                case 2: //one task normale
                pseudo_inverse(J_.data, J_pinv_);
                break;
*/
                case 3: //one task pseudo 
                {
                id_solver_->JntToMass(joint_msr_states_.q, M_);
                W_gain = M_.data;
                pseudo_inverse(J_.data* W_gain.inverse() *J_.data.transpose(), J_pinv_);
                //std::cout << "Following command with strategy 3" << std::endl;
                J_pinv_ = W_gain.inverse() * J_.data.transpose() *J_pinv_;
                }
                break;
                
                default:
                
                pseudo_inverse(J_.data, J_pinv_); 
                break;
                
            }



            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

            // end-effector position error
            x_err_.vel = x_des_.p - x_.p;

            // getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);
            x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));
            }

            // end-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += k_click*J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
          
            }

            //JOINTS LIMIT AVOIDANCE
            if (control_type == 0)
            {
                    //std::cout << "Following command with strategy 0" << std::endl;
                    Eigen::Matrix<double, 7, 7> P;
                    P =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_ * J_.data;

                    Eigen::Matrix<double, 7, 1> q_null;
                    Eigen::Matrix<double, 7, 1> q_zero;
                    
                    double k0 = 1;
                    for (unsigned int i = 0; i < 7; i++) 
                    {
                        // funzione peso per stale lontano dai fine corsa
                        q_zero[i] = -k0/14 * joint_msr_states_.q(i)/joint_limits_.max(i);

                    }

                    q_null = P * q_zero; //removed scaling factor of .7

                    for (int i = 0; i < 7; i++)
                    {
                        joint_des_states_.qdot(i) += q_null[i]; //removed scaling factor of .7
                    }
                
            }

            //MACRO MICRO ON JACOBIAN NULL
            if (control_type == 1)
            {
                   // std::cout << "Following command with strategy 1" << std::endl;

                    KDL::Twist x_err_2;

                KDL::Frame x_2;
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_2, 6);

            KDL::Jacobian J_2;
            J_2.resize(kdl_chain_.getNrOfJoints());
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_2, 6);

                Eigen::Matrix<double, 7, 7> P;
                P =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_ * J_.data;

                Eigen::Matrix<double, 7, 1> q_null;
                Eigen::MatrixXd J_pinv_2;

                Eigen::Matrix<double, 3, 7> J_2_short = Eigen::Matrix<double, 3, 7>::Zero();
                J_2_short = J_2.data.block<3, 7>(0, 0);
                pseudo_inverse(J_2_short, J_pinv_2);
                Eigen::Matrix<double, 7, 3> NullSpace = Eigen::Matrix<double, 7, 3>::Zero();
                NullSpace = P * J_pinv_2;

                x_err_2.vel = x_des_2.p - x_2.p;
               // std::cout << std::endl << "TASK 2 current frame = " << std::endl << x_2 <<std::endl;
         
                //std::cout << std::endl << "TASK 2 current frame = " << std::endl << x_des_2 <<std::endl;
                //std::cout << std::endl << "TASK 2 Jacobian= " << std::endl << J_2_short <<std::endl;

                Eigen::MatrixXd x_err_2_eigen = Eigen::MatrixXd::Zero(3, 1);
                x_err_2_eigen << x_err_2.vel(0), x_err_2.vel(1), x_err_2.vel(2);

                q_null = alpha2 * NullSpace * x_err_2_eigen; //removed scaling factor of .7

                for (int i = 0; i < J_pinv_2.rows(); i++)
                {
                    joint_des_states_.qdot(i) += alpha2 * q_null[i]; //removed scaling factor of .7
                }
            }
            








            // integrating q_dot -> getting q (Euler method)
            for (unsigned int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }



            //PEPIPA


            if (Equal(x_, x_des_, 0.005))
            {
                ROS_INFO("On target");
                cmd_flag_ = 0;

                

              
            }


        }

        
        // set controls for joints
        for (unsigned int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }

    }

    void CLIKController::command(const lwr_controllers::PoseRPY::ConstPtr &msg)
    {
        KDL::Frame frame_des_;

        switch(msg->id)
        {
            case 0:
            frame_des_ = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                      msg->orientation.pitch,
                                      msg->orientation.yaw),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
            break;

            case 1: // position only
            frame_des_ = KDL::Frame(
                KDL::Vector(msg->position.x,
                            msg->position.y,
                            msg->position.z));
            break;

            case 2: // orientation only
            frame_des_ = KDL::Frame(
                KDL::Rotation::RPY(msg->orientation.roll,
                                   msg->orientation.pitch,
                                   msg->orientation.yaw));
            break;

            default:
            ROS_INFO("Wrong message ID");
            return;
        }

        x_des_ = frame_des_;
        cmd_flag_ = 1;

    }


void CLIKController::command2(const geometry_msgs::Pose::ConstPtr &msg)
{
    KDL::Frame frame_des_;

    frame_des_ = KDL::Frame(
                     KDL::Rotation::Quaternion(msg->orientation.x,
                             msg->orientation.y,
                             msg->orientation.z,
                             msg->orientation.w),
                     KDL::Vector(msg->position.x,
                                 msg->position.y,
                                 msg->position.z));

    x_des_2 = frame_des_;
  
    //std::cout << std::endl << "TASK 2 desired frame = " << std::endl << x_des_2 <<std::endl;

    tf::Transform CollisionTransform2;
    tf::transformKDLToTF( frame_des_, CollisionTransform2);
    elbow_reference.sendTransform( tf::StampedTransform( CollisionTransform2, ros::Time::now(), "world", "elbow_reference") );
}
     void CLIKController::controller_type(const std_msgs::Int16::ConstPtr &msg)
     {
            std::stringstream ss;
            control_type = msg->data;
            //std::cout << "message arrived " << std::to_string(msg->data)<<" while control type is "<< std::to_string(control_type)<< std::endl;       
            //control_type = 0 (default) -> multi task (lontano da fine corsa)
            //control_type = 1 -> multi task (gomito basso)
            //control_type = 2 -> one task normale
            //control_type = 3 -> pseudo-inv pesata

            switch(control_type)
            {

                case 0: 
                ss << "Multi Task; second task is to stay far from joints limits";
                break;

                case 1:
                ss << "Multi Task; second task is to stay with elbow in a position. Please set Elbow Position with command2 topic";
                break;

                case 2:
                ss << "CLIKController";
                break;

                case 3: 
                ss << "CLIKController with weighted Pseudo-Inverse";
                break;
                default: 
                ss << "Strategy not consented. Please insert a strategy between 0 and 3.";
                break;

                
            }
            std::cout << ss.str()<< std::endl;
            return;
 

     }

      void CLIKController::extend_chain(ros::NodeHandle &n)
  {
    // get the p_wrist_ee arm from the rosparam server
    std::vector<double> p_wrist_ee;

    n.getParam("p_wrist_ee", p_wrist_ee);

    p_wrist_ee_ = KDL::Vector(p_wrist_ee.at(0), p_wrist_ee.at(1), p_wrist_ee.at(2));

    // Extend the default chain with a fake segment in order to evaluate
    // Jacobians and forward kinematics with respect to a given reference point
    // (typicallly the tool tip)
    // also takes into account end effector mass and inertia
    KDL::Joint fake_joint = KDL::Joint();
    KDL::Frame frame(KDL::Rotation::Identity(), p_wrist_ee_);


    //TODO
    double total_mass = 0.1;
    double fake_cylinder_radius  = 0.0475;
    double fake_cylinder_height = 0.31;



    //Zingarata
    KDL::Vector p_sensor_tool_com ={0,0,0} ;
    //load_calib_data(total_mass, p_sensor_tool_com);

        
    double fake_cyl_i_xx = 1.0 / 12.0 * total_mass * (3 * pow(fake_cylinder_radius, 2) + \
                                 pow(fake_cylinder_height, 2));
    double fake_cyl_i_zz = 1.0 / 2.0 * total_mass * pow(fake_cylinder_radius, 2);

    KDL::RotationalInertia rot_inertia(fake_cyl_i_xx, fake_cyl_i_xx, fake_cyl_i_zz);
    KDL::RigidBodyInertia inertia(total_mass, p_sensor_tool_com, rot_inertia);

    KDL::Segment fake_segment(fake_joint, frame, inertia);


    extended_chain_ = kdl_chain_;
    extended_chain_.addSegment(fake_segment);
  }

}

PLUGINLIB_EXPORT_CLASS(lwr_controllers::CLIKController, controller_interface::ControllerBase)
