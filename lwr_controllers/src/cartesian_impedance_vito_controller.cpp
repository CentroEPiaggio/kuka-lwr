#include <lwr_controllers/cartesian_impedance_vito_controller.h>

namespace lwr_controllers {
    CartesianImpedanceVitoController::CartesianImpedanceVitoController() {}
    CartesianImpedanceVitoController::~CartesianImpedanceVitoController() {}
    
    void CartesianImpedanceVitoController::load_gains() { // carica dal file mat.txt i valori delle matrici per il sistema massa molla smorzatore a ciclo chiuso
		// il file mat.txt contiene (n ordine) i valori della matrice K, di D, di M, le posizioni dei giunti del braccio sinistro e destro di Vito
        std::ifstream in("/home/centropiaggio/lorale_ws/src/vito-robot/kuka-lwr/lwr_controllers/src/cartesian_gains_mat.txt");

        if (!in) {
          std::cout << "Cannot open file\n";
          return;
        }
      
        K_.setZero();
        D_.setZero();
		M_mms_.setZero();
		q_des_ct_.setZero();
		
		in >> M_mms_(0,0);
		in >> M_mms_(1,1);
		in >> M_mms_(2,2);
		in >> M_mms_(3,3);
		in >> M_mms_(3,4);
		in >> M_mms_(3,5);
		in >> M_mms_(4,3);
		in >> M_mms_(4,4);
		in >> M_mms_(4,5);
		in >> M_mms_(5,3);
		in >> M_mms_(5,4);
		in >> M_mms_(5,5);
        
        for (int x = 0; x < 6; x++)
          in >> K_(x,x);
        for (int x = 0; x < 6; x++)
          in >> D_(x,x);
		for (int x = 0; x < 7; x++)
          in >> q_des_ct_(x);
		if (!robot_namespace_.compare("right_arm"))
			for (int x = 0; x < 7; x++)
				in >> q_des_ct_(x);
        in.close();
    }
    
    Eigen::Matrix<double,3,3> CartesianImpedanceVitoController::quat_rot(double x, double y, double z, double w) {
		Eigen::Matrix<double,3,3> temp;
		temp << 1-2*pow(y,2)-2*pow(z,2),             2*x*y-2*w*z,             2*x*z+2*w*y,
							2*x*y+2*w*z, 1-2*pow(x,2)-2*pow(z,2),             2*z*y-2*w*x,
							2*x*z-2*w*y,             2*z*y+2*w*x, 1-2*pow(x,2)-2*pow(y,2);
		return temp;
	}
    
    Eigen::Matrix<double,3,3> CartesianImpedanceVitoController::skew(Eigen::Matrix<double,3,1> v) {
        Eigen::Matrix<double,3,3> temp;
        temp <<     0, -v(2),  v(1),
                 v(2),     0, -v(0),
                -v(1),  v(0),     0;
        return temp;
    }
    
    double CartesianImpedanceVitoController::norm(Eigen::VectorXd v) {
        return sqrt( v.transpose()*v );
    }
    
    bool CartesianImpedanceVitoController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
        if ( !( KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n))) {
            ROS_ERROR("Couldn't initialize controller.");
            return false;
        }

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        KDL::Rotation world_to_base_right_arm = KDL::Rotation::RPY(3.1415926535897932384626, -3.1415926535897932384626/4.0, 0.0);
		id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,world_to_base_right_arm.Inverse()*gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

		J_ge_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
		
        /*robot_namespace_ = n.getNamespace();
        n.getParam("robot_name", robot_namespace_);

        joint_names_.push_back( robot_namespace_ + std::string("_a1_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a2_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_e1_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a3_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a4_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a5_joint") );
        joint_names_.push_back( robot_namespace_ + std::string("_a6_joint") );

        for(int j = 0; j < 7; j++)
            joint_handles_.push_back(robot->getHandle(joint_names_[j]));*/

        sub_command_ = nh_.subscribe("command", 1, &CartesianImpedanceVitoController::command, this);
        pub_pose_ = nh_.advertise<lwr_controllers::PoseRPY>("pose", 100);

        return true;
    }

    void CartesianImpedanceVitoController::starting(const ros::Time& time) {

        for (size_t i = 0; i < joint_handles_.size(); i++) {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            ROS_INFO_STREAM("Effort [" << i << "] " << joint_handles_[i].getEffort());
        }
        
        // posizione dei giunti del robot all'inizio della simulazione
        q0_ = joint_msr_states_.q.data;

        fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);
        id_solver_->JntToGravity(joint_msr_states_.q, G_);
    	cmd_flag_ = 0;	// non attiva il controllo di impedenza

        load_gains();
		
		int n = 1000;
		Kp_ct_ << n, 0, 0, 0, 0, 0, 0,
				  0, n, 0, 0, 0, 0, 0,
				  0, 0, n, 0, 0, 0, 0,
				  0, 0, 0, n, 0, 0, 0,
				  0, 0, 0, 0, n, 0, 0,
				  0, 0, 0, 0, 0, n, 0,
				  0, 0, 0, 0, 0, 0, n;

		n = 10;
		Kv_ct_ << n, 0, 0, 0, 0, 0, 0,
				  0, n, 0, 0, 0, 0, 0,
				  0, 0, n, 0, 0, 0, 0,
				  0, 0, 0, n, 0, 0, 0,
				  0, 0, 0, 0, n, 0, 0,
				  0, 0, 0, 0, 0, n, 0,
				  0, 0, 0, 0, 0, 0, n;
		counter = 0;	// per l'interpolazione
		N = 10000;		//

        ROS_INFO_STREAM("M:" << std::endl << M_mms_ << std::endl << "K:" << std::endl << K_ << std::endl << "D:" << std::endl << D_ << std::endl);
        
        /////////// homing
    }

    void CartesianImpedanceVitoController::update(const ros::Time& time, const ros::Duration& period) {
        
        ROS_INFO_STREAM("aaaaaaaaaa");
        ROS_INFO_STREAM("joint_handles_.size() " << joint_handles_.size());
        // get joints positions
  		for (int i = 0; i < joint_handles_.size(); i++) {
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            tau_[i] = joint_handles_[i].getEffort();
            ROS_INFO_STREAM("Effort [" << i << "] " << joint_handles_[i].getEffort());
    	}

        ROS_INFO_STREAM("bbbbbbbbbb");
    	
    	//id_solver_->JntToGravity(joint_msr_states_.q, G_);
    	//id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
		
    	//tau_ = G_.data + C_.data;
        //tau_ = G_.data;

		if (com_torque) {	// computed torque per portare il robot alla configurazione iniziale presa dal file mat.txt
			// interpolazione
            if (counter < N) {
                q_des_ct_step_ = q_des_ct_step_ + (q_des_ct_- q0_)/N;
                counter++;
            } else {
                q_des_ct_step_ = q_des_ct_;
            }
			tau_ = G_.data + C_.data + Kp_ct_*(q_des_ct_step_ - joint_msr_states_.q.data) - Kv_ct_*joint_msr_states_.qdot.data;
		}

        ROS_INFO_STREAM("cccccccccccc");
        id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
        id_solver_->JntToGravity(joint_msr_states_.q, G_);

        for(int i=1;i<joint_handles_.size();i++)
        {
            //tau_[i] = C_(i)*joint_msr_states_.qdot(i) + G_(i);
        }
        //tau_ = G_.data + C_.data;

    	if (cmd_flag_) {
			// computing Inertia, Coriolis and Gravity matrices
			id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
			id_solver_->JntToGravity(joint_msr_states_.q, G_);
		    id_solver_->JntToMass(joint_msr_states_.q, M_);

	    	// computing Jacobian J(q)
	    	jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_ge_);
			pseudo_inverse(J_ge_.data,J_ge_pinv_,false);

	    	// computing forward kinematics
	    	fk_pos_solver_->JntToCart(joint_msr_states_.q,x_);

			double t1,t2,t3,t4;
			x_.M.GetQuaternion(t1,t2,t3,t4);
			
	    	// publish pose to be plotted in rqt_plot
	    	msg_pose_.position.x = x_.p(0);
	    	msg_pose_.position.y = x_.p(1);
	    	msg_pose_.position.z = x_.p(2);
	    	x_.M.GetEulerZYX(msg_pose_.orientation.yaw, msg_pose_.orientation.pitch, msg_pose_.orientation.roll);
	    	pub_pose_.publish(msg_pose_);

			// xVec usato solo per il calcolo dell'errore sul posizionamento
            xVEC_(0) = x_.p(0);
            xVEC_(1) = x_.p(1);
            xVEC_(2) = x_.p(2);
			ROS_INFO_STREAM("pos " << xVEC_);
            //x_.M.GetEulerZYX(xVEC_(5),xVEC_(4),xVEC_(3));

	    	// velocity error
            e_ref_dot_ = J_ge_.data*joint_msr_states_.qdot.data;	// scrivo direttamente così perché la velocità desiderata è 0
            e_ref_dot_(3) = 0;	// errori di velocità sull'orientamento calcolati dopo
            e_ref_dot_(4) = 0;
            e_ref_dot_(5) = 0;

			// interpolazione
            if (counter < N) {
                // interpolazione per le posizioni
                xDES_step_ = xDES_step_ + (xDES_ - x0)/N;
                //interpolazione per l'orientamento
                quat_t = quat_0.slerp(quat_f,counter/N);
                counter++;
            } else {
                if (!primo)
                    ROS_INFO("End positioning.");
                xDES_step_ = xDES_;
                quat_t = quat_f;
                primo = true;
            }    
            
	    	// position error
            e_ref_ = xDES_step_ - xVEC_;
            
            // Quaternion
            quat_des_vec_(0) = quat_t.x();
            quat_des_vec_(1) = quat_t.y();
            quat_des_vec_(2) = quat_t.z();
            quat_des_scal_ = quat_t.w();
	    	x_.M.GetQuaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);
	    	quat_temp_ = quat_scal_*quat_des_vec_ - quat_des_scal_*quat_vec_ - skew(quat_des_vec_)*quat_vec_;
	    	e_ref_(3) = quat_temp_(0);
	    	e_ref_(4) = quat_temp_(1);
	    	e_ref_(5) = quat_temp_(2);
	    	e_ref_dot_(3) = (quat_temp_(0) - quat_old_(0))/period.toSec();	// è giusto imporre così gli errori di velocità sull'orientamento?
            e_ref_dot_(4) = (quat_temp_(1) - quat_old_(1))/period.toSec();	//
            e_ref_dot_(5) = (quat_temp_(2) - quat_old_(2))/period.toSec();	//
            
            // controlla se il robot è nella posa desiderata
            if (norm(xDES_ - xVEC_) < 0.001)
	    		ROS_INFO("On target");
            
            quat_old_ = quat_temp_;

            // jacobian derivative
            J_dot_.data = (J_ge_.data - J_last_.data)/period.toSec();

	    	// computing the torque "tau"
	    	tau_ = G_.data + C_.data + 
				   M_.data*J_ge_pinv_*M_mms_.inverse()*(-D_*e_ref_dot_ + K_*e_ref_ - M_mms_*J_dot_.data*joint_msr_states_.qdot.data);
    	}

        ROS_INFO_STREAM("dddddddddd");
        
    	// set controls for joints
    	for (int i = 0; i < 6; i++) 
    	    joint_handles_[i].setCommand(tau_(i));
		if (com_torque)
			joint_handles_[6].setCommand(tau_(6));
		else
			joint_handles_[6].setCommand(tau_(6));	// con gli attriti più alti nel robot ci va 10*G_.data(6)
		
//Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        joint_stiffness_handles_[i].setCommand(joint_stiffness_handles_[i].getPosition());
        joint_damping_handles_[i].setCommand(joint_damping_handles_[i].getPosition());
        joint_set_point_handles_[i].setCommand(joint_msr_states_.q.data[i]);
    }

        ROS_INFO_STREAM("Torques:");
        ROS_INFO_STREAM(tau_);

        J_last_ = J_ge_;
	    ros::spinOnce();
    }

    void CartesianImpedanceVitoController::command(const lwr_controllers::PoseRPY::ConstPtr &msg) {	// callback per gestire messaggio della posa desiderata da terminale
        ROS_INFO("Received message");
        KDL::Frame frame_des_;
		
		com_torque = false;
        
        for (int i = 0; i < joint_handles_.size(); i++) {
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    	}
    	fk_pos_solver_->JntToCart(joint_msr_states_.q,x_);

        switch(msg->id) {
			case 0:		// position and orientation
			    frame_des_ = KDL::Frame(
					KDL::Rotation::EulerZYX(msg->orientation.yaw,
						 			        msg->orientation.pitch,
								 	        msg->orientation.roll),
					KDL::Vector(msg->position.x,
								msg->position.y,
								msg->position.z));
			    xDES_(0) = msg->position.x;
                xDES_(1) = msg->position.y;
                xDES_(2) = msg->position.z;
                xDES_(3) = msg->orientation.roll;
                xDES_(4) = msg->orientation.pitch;
                xDES_(5) = msg->orientation.yaw;
			    break;
	
			case 1: // position only (holds the orientation)
			    xDES_(0) = msg->position.x;
                xDES_(1) = msg->position.y;
                xDES_(2) = msg->position.z;
                x_.M.GetEulerZYX(xDES_(5),xDES_(4),xDES_(3));
                frame_des_ = KDL::Frame(
					KDL::Rotation::EulerZYX(xDES_(5),
						 			        xDES_(4),
								 	        xDES_(3)),
					KDL::Vector(msg->position.x,
								msg->position.y,
								msg->position.z));
			    break;
		
			case 2: // orientation only (holds the position)
			    frame_des_ = KDL::Frame(
					KDL::Rotation::EulerZYX(msg->orientation.yaw,
						 			        msg->orientation.pitch,
								 	        msg->orientation.roll),
					KDL::Vector(x_.p(0),
								x_.p(1),
								x_.p(2)));
			    xDES_(0) = x_.p(0);
                xDES_(1) = x_.p(1);
                xDES_(2) = x_.p(2);
                xDES_(3) = msg->orientation.roll;
                xDES_(4) = msg->orientation.pitch;
                xDES_(5) = msg->orientation.yaw;
			    break;

			default:
			    ROS_INFO("Wrong message ID");
			    return;
		}

    	x_des_ = frame_des_;
		
        // computing Jacobian J(q) as initial condition to J_last_
	    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_last_);   
		cmd_flag_ = 1;	// inizia ad usare il controllo di impedenza
		
		if (primo) {	// entra in questo if solo all'invio del primo messaggio, quando c'è ancora il computerd torque in funzione
            primo = false;
            N = 10000;       // da cambiare in base alla norma dell'errore
            x_.M.GetQuaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);          // quaternione terna attuale
            quat_0 = tf::Quaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);   
            x_des_.M.GetQuaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);      // quaternione terna desiderata
            quat_f = tf::Quaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);
            x0 << x_.p(0), x_.p(1), x_.p(2), 0, 0, 0;           // posizione attuale
            xDES_step_ = x0;
            counter = 0;
        }
    }
} // namespace

PLUGINLIB_EXPORT_CLASS( lwr_controllers::CartesianImpedanceVitoController, controller_interface::ControllerBase)
