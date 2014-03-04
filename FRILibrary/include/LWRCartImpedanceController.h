//  ---------------------- Doxygen info ----------------------
//! \file LWRCartImpedanceController.h
//!
//! \brief
//! Header file for the class LWRCartImpedanceController
//!
//! \details
//! The class LWRCartImpedanceController constitutes an access
//! easy-to-use Cartesian impedance control interface through the KUKA
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


#ifndef __LWRCartImpedanceController__
#define __LWRCartImpedanceController__


#include <FastResearchInterface.h>
#include <LWRBaseControllerInterface.h>


//  ---------------------- Doxygen info ----------------------
//! \class LWRCartImpedanceController
//!
//! \brief
//! Cartesian impedance controller interface for the KUKA Light-Weight
//! Robot IV
//  ----------------------------------------------------------

class LWRCartImpedanceController : public LWRBaseControllerInterface
{


public:


//  ---------------------- Doxygen info ----------------------
//! \fn LWRCartImpedanceController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
//!
//! \brief
//! Constructor
//!
//! \copydetails LWRBaseControllerInterface::LWRBaseControllerInterface()
//  ----------------------------------------------------------
	LWRCartImpedanceController(const char *InitFileName):LWRBaseControllerInterface(InitFileName)
	{
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~LWRCartImpedanceController(void)
//!
//! \brief
//! Destructor
//!
//! \copydetails LWRBaseControllerInterface::~LWRBaseControllerInterface()
//  ----------------------------------------------------------
	~LWRCartImpedanceController(void)
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
		return(this->FRI->StartRobot(		FastResearchInterface::CART_IMPEDANCE_CONTROL
		                             	,	TimeOutValueInSeconds));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetMeasuredCartPose(float *MeasuredCartPose)
//!
//! \brief
//! \copybrief FastResearchInterface::GetMeasuredCartPose()
//!
//! \details
//! \copydetails FastResearchInterface::GetMeasuredCartPose()
//  ----------------------------------------------------------
	inline void GetMeasuredCartPose(float *MeasuredCartPose)
	{
		return(this->FRI->GetMeasuredCartPose(MeasuredCartPose));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartPose(const float *CommandedCartPose)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartPose()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartPose()
//  ----------------------------------------------------------
	inline void SetCommandedCartPose(const float *CommandedCartPose)
	{
		return(this->FRI->SetCommandedCartPose(CommandedCartPose));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartForcesAndTorques()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartForcesAndTorques()
//  ----------------------------------------------------------
	inline void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
	{
		return(this->FRI->SetCommandedCartForcesAndTorques(CartForcesAndTorques));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartStiffness(const float *CommandedCartStiffness)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartStiffness()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartStiffness()
//  ----------------------------------------------------------
	inline void SetCommandedCartStiffness(const float *CommandedCartStiffness)
	{
		return(this->FRI->SetCommandedCartStiffness(CommandedCartStiffness));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void SetCommandedCartDamping(const float *CommandedCartDamping)
//!
//! \brief
//! \copybrief FastResearchInterface::SetCommandedCartDamping()
//!
//! \details
//! \copydetails FastResearchInterface::SetCommandedCartDamping()
//  ----------------------------------------------------------
	inline void SetCommandedCartDamping(const float *CommandedCartDamping)
	{
		return(this->FRI->SetCommandedCartDamping(CommandedCartDamping));
	}


//  ---------------------- Doxygen info ----------------------
//! \fn inline void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
//!
//! \brief
//! \copybrief FastResearchInterface::GetEstimatedExternalCartForcesAndTorques()
//!
//! \details
//! \copydetails FastResearchInterface::GetEstimatedExternalCartForcesAndTorques()
//  ----------------------------------------------------------
	inline void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
	{
		return(this->FRI->GetEstimatedExternalCartForcesAndTorques(EstimatedExternalCartForcesAndTorques));
	}

};	// class LWRCartImpedanceController

#endif
