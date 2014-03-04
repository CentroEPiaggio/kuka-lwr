//  ---------------------- Doxygen info ----------------------
//! \file GetRobotControlData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! providing robot control data to the user application of this class
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
// GetMeasuredJointPositions()
//
void FastResearchInterface::GetMeasuredJointPositions(float *MeasuredJointPositions)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		MeasuredJointPositions[i]	=	this->ReadData.data.msrJntPos[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedJointPositions()
//
void FastResearchInterface::GetCommandedJointPositions(float *CommandedJointPositions)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		CommandedJointPositions[i]	=	this->ReadData.data.cmdJntPos[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedJointPositionOffsets()
//
void FastResearchInterface::GetCommandedJointPositionOffsets(float *CommandedJointPositionOffsets)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		CommandedJointPositionOffsets[i]	=	this->ReadData.data.cmdCartPosFriOffset[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetMeasuredJointTorques()
//
void FastResearchInterface::GetMeasuredJointTorques(float *MeasuredJointTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		MeasuredJointTorques[i]	=	this->ReadData.data.msrJntTrq[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetEstimatedExternalJointTorques()
//
void FastResearchInterface::GetEstimatedExternalJointTorques(float *EstimatedExternalJointTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		EstimatedExternalJointTorques[i]	=	this->ReadData.data.estExtJntTrq[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetMeasuredCartPose()
//
void FastResearchInterface::GetMeasuredCartPose(float *MeasuredCartPose)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_FRM_DIM; i++)
	{
		MeasuredCartPose[i]	=	this->ReadData.data.msrCartPos[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedCartPose()
//
void FastResearchInterface::GetCommandedCartPose(float *CommandedCartPose)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_FRM_DIM; i++)
	{
		CommandedCartPose[i]	=	this->ReadData.data.cmdCartPos[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCommandedCartPoseOffsets()
//
void FastResearchInterface::GetCommandedCartPoseOffsets(float *CommandedCartPoseOffsets)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_FRM_DIM; i++)
	{
		CommandedCartPoseOffsets[i]	=	this->ReadData.data.cmdCartPosFriOffset[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetEstimatedExternalCartForcesAndTorques()
//
void FastResearchInterface::GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_CART_VEC; i++)
	{
		EstimatedExternalCartForcesAndTorques[i]	=	this->ReadData.data.estExtTcpFT[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}

