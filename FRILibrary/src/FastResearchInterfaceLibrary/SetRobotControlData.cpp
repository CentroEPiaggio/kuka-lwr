//  ---------------------- Doxygen info ----------------------
//! \file SetRobotControlData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! setting robot control data by the user application of this class
//!
//! \details
//! The class FastResearchInterface provides a basic low-level interface
//! to the KUKA Light-Weight Robot IV For details, please refer to the file
//! FastResearchInterface.h.
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


#include <FastResearchInterface.h>
#include <pthread.h>
#include <friComm.h>


// ****************************************************************
// SetCommandedJointPositions()
//
void FastResearchInterface::SetCommandedJointPositions(const float *CommandedJointPositions)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		this->CommandData.cmd.jntPos[i]	=	CommandedJointPositions[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedJointTorques()
//
void FastResearchInterface::SetCommandedJointTorques(const float *CommandedJointTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		this->CommandData.cmd.addJntTrq[i]	=	CommandedJointTorques[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedJointStiffness()
//
void FastResearchInterface::SetCommandedJointStiffness(const float *CommandedJointStiffness)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		this->CommandData.cmd.jntStiffness[i]	=	CommandedJointStiffness[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedJointDamping()
//
void FastResearchInterface::SetCommandedJointDamping(const float *CommandedJointDamping)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		this->CommandData.cmd.jntDamping[i]	=	CommandedJointDamping[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartPose()
//
void FastResearchInterface::SetCommandedCartPose(const float *CommandedCartPose)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_FRM_DIM; i++)
	{
		this->CommandData.cmd.cartPos[i]	=	CommandedCartPose[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartForcesAndTorques()
//
void FastResearchInterface::SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_VEC; i++)
	{
		this->CommandData.cmd.addTcpFT[i]	=	CartForcesAndTorques[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartStiffness()
//
void FastResearchInterface::SetCommandedCartStiffness(const float *CommandedCartStiffness)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_VEC; i++)
	{
		this->CommandData.cmd.cartStiffness[i]	=	CommandedCartStiffness[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetCommandedCartDamping()
//
void FastResearchInterface::SetCommandedCartDamping(const float *CommandedCartDamping)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_VEC; i++)
	{
		this->CommandData.cmd.cartDamping[i]	=	CommandedCartDamping[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}
