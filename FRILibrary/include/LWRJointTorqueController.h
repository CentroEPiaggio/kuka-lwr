//  ---------------------- Doxygen info ----------------------
//! \file LWRJointTorqueController.h
//!
//! \brief
//! Header file for the class LWRJointTorqueController
//!
//! \details
//! The class LWRJointTorqueController constitutes an access
//! easy-to-use joint torque control interface through the KUKA
//! Fast Research Interface of the Light-Weight Robot IV.
//!
//! \n
//! \n
//! \n
//! Stanford University\n
//! Department of Computer Science\n
//! Artificial Intelligence Laboratory\n
//! Gates Computer Science Building 1A\n
//! 353 Serra Mall\n
//! Stanford, CA 94305-9010\n
//! USA\n
//! \n
//! http://cs.stanford.edu/groups/manips\n
//!
//! \date November 2011
//!
//! \version 1.0
//!
//!	\author Torsten Kroeger, tkr@stanford.edu
//!
//!
//! \note Copyright (C) 2010 Stanford University.
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __LWRJointTorqueController__
#define __LWRJointTorqueController__


#include <FastResearchInterface.h>
#include <LWRBaseControllerInterface.h>


//  ---------------------- Doxygen info ----------------------
//! \class LWRJointTorqueController
//!
//! \brief
//! Joint torque controller interface for the KUKA Light-Weight
//! Robot IV
//  ----------------------------------------------------------

class LWRJointTorqueController : public LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRJointTorqueController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
//!
//! \brief
//! Constructor
//!
//! \copydetails LWRBaseControllerInterface::LWRBaseControllerInterface()
//  ----------------------------------------------------------
	LWRJointTorqueController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRJointTorqueController(void)
//!
//! \brief
//! Destructor
//!
//! \copydetails LWRBaseControllerInterface::~LWRBaseControllerInterface()
//  ----------------------------------------------------------
	~LWRJointTorqueController(void)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
//!
//! \brief
//! \copybrief FastResearchInterface::StartRobot()
//!
//! \details
//! \copydetails FastResearchInterface::StartRobot()
//  ----------------------------------------------------------
	inline int StartRobot(const float &TimeOutValueInSeconds	=	120.0)
	{
		this->printf("Please start up the robot now by using KUKA Control Panel.\n");

		// start the controller and switch to command mode
		return(this->FRI->StartRobot(		FastResearchInterface::JOINT_TORQUE_CONTROL
		                             	,	TimeOutValueInSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointTorques(const float *CommandedJointTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointTorques()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointTorques()
//  ----------------------------------------------------------
	inline void SetCommandedJointTorques(const float *CommandedJointTorques)
	{
		return(this->FRI->SetCommandedJointTorques(CommandedJointTorques));
	}

};	// class LWRJointTorqueController

#endif
