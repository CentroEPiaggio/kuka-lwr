//  ---------------------- Doxygen info ----------------------
//! \file LWRJointImpedanceController.h
//!
//! \brief
//! Header file for the class LWRJointImpedanceController
//!
//! \details
//! The class LWRJointImpedanceController constitutes an access
//! easy-to-use joint impedance control interface through the KUKA
//! Fast Research Interface of the Light-Weight Robot IV.
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! This file is part of the Fast Research Interface Library.
//! \n\n
//! The Fast Research Interface Library is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Fast Research Interface Library is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Fast Research Interface Library. If not, see 
//! http://www.gnu.org/licenses.
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
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#ifndef __LWRJointImpedanceController__
#define __LWRJointImpedanceController__


#include <FastResearchInterface.h>
#include <LWRBaseControllerInterface.h>


//  ---------------------- Doxygen info ----------------------
//! \class LWRJointImpedanceController
//!
//! \brief
//! Joint impedance controller interface for the KUKA Light-Weight
//! Robot IV
//  ----------------------------------------------------------

class LWRJointImpedanceController : public LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRJointImpedanceController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
//!
//! \brief
//! Constructor
//!
//! \copydetails LWRBaseControllerInterface::LWRBaseControllerInterface()
//  ----------------------------------------------------------
	LWRJointImpedanceController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRJointImpedanceController(void)
//!
//! \brief
//! Destructor
//!
//! \copydetails LWRBaseControllerInterface::~LWRBaseControllerInterface()
//  ----------------------------------------------------------
	~LWRJointImpedanceController(void)
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
		return(this->FRI->StartRobot(		FastResearchInterface::JOINT_IMPEDANCE_CONTROL
		                             	,	TimeOutValueInSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointPositions(const float *CommandedJointPositions)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointPositions()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointPositions()
//  ----------------------------------------------------------
	inline void SetCommandedJointPositions(const float *CommandedJointPositions)
	{
		return(this->FRI->SetCommandedJointPositions(CommandedJointPositions));
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


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointStiffness(const float *CommandedJointStiffness)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointStiffness()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointStiffness()
//  ----------------------------------------------------------
	inline void SetCommandedJointStiffness(const float *CommandedJointStiffness)
	{
		return(this->FRI->SetCommandedJointStiffness(CommandedJointStiffness));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedJointDamping(const float *CommandedJointDamping)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedJointDamping()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedJointDamping()
//  ----------------------------------------------------------
	inline void SetCommandedJointDamping(const float *CommandedJointDamping)
	{
		return(this->FRI->SetCommandedJointDamping(CommandedJointDamping));
	}


};	// class LWRJointImpedanceController

#endif
