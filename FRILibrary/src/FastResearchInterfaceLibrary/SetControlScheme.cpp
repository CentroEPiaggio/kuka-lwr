//  ---------------------- Doxygen info ----------------------
//! \file SetControlScheme.cpp
//!
//! \brief
//! Implementation file for the method SetControlScheme() of the class FastResearchInterface
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
#include <errno.h>
#include <string.h>
#include <friComm.h>
#include <OSAbstraction.h>


// ****************************************************************
// SetControlScheme()
//
int FastResearchInterface::SetControlScheme(const unsigned int &ControlScheme)
{
	unsigned int		i				=	0;

	int					ResultValue		=	0;

	float				FloatValues[2 * FRI_CART_FRM_DIM];

	memset(FloatValues, 0x0, 2 * LBR_MNJ * sizeof(float));

	if (this->GetFRIMode() == FRI_STATE_MON)
	{
		switch (ControlScheme)
		{
		case FastResearchInterface::JOINT_POSITION_CONTROL:
			pthread_mutex_lock(&(this->MutexForControlData));
			this->CommandData.cmd.cmdFlags	=	0;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_JNTPOS;
			pthread_mutex_unlock(&(this->MutexForControlData));

			// let the KRL program start the joint position controller
			this->SetKRLIntValue(14, 10);
			break;
		case FastResearchInterface::CART_IMPEDANCE_CONTROL:
			pthread_mutex_lock(&(this->MutexForControlData));
			this->CommandData.cmd.cmdFlags	=	0;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_CARTPOS;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_TCPFT;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_CARTSTIFF;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_CARTDAMP;
			pthread_mutex_unlock(&(this->MutexForControlData));

			// let the KRL program start the Cartesian impedance controller
			this->SetKRLIntValue(14, 20);
			break;
		case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
			pthread_mutex_lock(&(this->MutexForControlData));
			this->CommandData.cmd.cmdFlags	=	0;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_JNTPOS;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_JNTTRQ;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_JNTSTIFF;
			this->CommandData.cmd.cmdFlags	|=	FRI_CMD_JNTDAMP;
			pthread_mutex_unlock(&(this->MutexForControlData));

			// let the KRL program start the joint impedance controller
			this->SetKRLIntValue(14, 30);
			break;
		default:
			return(EINVAL);
		}

		if (ControlScheme == FastResearchInterface::CART_IMPEDANCE_CONTROL)
		{
			this->SetKRLIntValue(13, 0);
			this->SetCommandedCartForcesAndTorques(FloatValues);
			for (i = 0; i < FRI_CART_FRM_DIM; i++)
			{
				FloatValues[i]	=	(float)0.7;
			}
			this->SetCommandedCartDamping(FloatValues);
			for (i = 0; i < FRI_CART_FRM_DIM; i++)
			{
				FloatValues[i]	=	(i < 3)?(1000.0):(100.0);
			}
			this->SetCommandedCartStiffness(FloatValues);

			this->GetCommandedCartPose(&(FloatValues[0]));
			this->GetCommandedCartPoseOffsets(&(FloatValues[FRI_CART_FRM_DIM]));

			// Regarding the documentation, we should do this
			/* -------------------------------------------------------------
			for (i = 0; i < FRI_CART_FRM_DIM; i++)
			{
				FloatValues[i]	+=	FloatValues[i + FRI_CART_FRM_DIM];
			}
			//------------------------------------------------------------- */

			this->SetCommandedCartPose(FloatValues);
		}
		else
		{
			this->SetCommandedJointTorques(FloatValues);
			this->SetKRLIntValue(13, 0);
			this->GetMeasuredJointPositions(&(FloatValues[0]));
			this->GetCommandedJointPositionOffsets(&(FloatValues[LBR_MNJ]));

			// Regarding the documentation, we should do this
			/* -------------------------------------------------------------
			for (i = 0; i < LBR_MNJ; i++)
			{
				FloatValues[i] +=	FloatValues[i + LBR_MNJ];
			}
			------------------------------------------------------------- */

			this->SetCommandedJointPositions(FloatValues);
		}

		// wait for the next data telegram of the KRC unit
		pthread_mutex_lock(&(this->MutexForControlData));
		this->NewDataFromKRCReceived	=	false;
		pthread_mutex_unlock(&(this->MutexForControlData));
		ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::StopRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
			return(ENOTCONN);
		}

		return(EOK);
	}
	else
	{
		if (this->GetFRIMode() == FRI_STATE_CMD)
		{
			return(EBUSY);
		}
		else
		{
			return(ENOTCONN);
		}
	}
}
