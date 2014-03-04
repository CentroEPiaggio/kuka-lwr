//  ---------------------- Doxygen info ----------------------
//! \file GetRobotStatusData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! providing robot status data to the user application of this class
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
// GetFRIMode()
//
unsigned int FastResearchInterface::GetFRIMode(void)
{
	unsigned int	ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.intf.state;
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}

// ****************************************************************
// GetCurrentControlScheme()
//
unsigned int FastResearchInterface::GetCurrentControlScheme(void)
{
	unsigned int		ResultValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ResultValue	=	this->ReadData.robot.control;
	pthread_mutex_unlock(&(this->MutexForControlData));

	switch (ResultValue)
	{
	case FRI_CTRL_POSITION :
		return(FastResearchInterface::JOINT_POSITION_CONTROL);
		break;
	case FRI_CTRL_CART_IMP:
		return(FastResearchInterface::CART_IMPEDANCE_CONTROL);
		break;
	case FRI_CTRL_JNT_IMP:
		return(FastResearchInterface::JOINT_IMPEDANCE_CONTROL);
		break;
	}

	return(0);
}


// ****************************************************************
// IsRobotArmPowerOn()
//
bool FastResearchInterface::IsRobotArmPowerOn(void)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(this->ReadData.robot.power != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// DoesAnyDriveSignalAnError()
//
bool FastResearchInterface::DoesAnyDriveSignalAnError(void)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(this->ReadData.robot.error != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// DoesAnyDriveSignalAWarning()
//
bool FastResearchInterface::DoesAnyDriveSignalAWarning(void)
{
	bool		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(this->ReadData.robot.warning != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetDriveTemperatures()
//
void FastResearchInterface::GetDriveTemperatures(float *Temperatures)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		Temperatures[i]	=	this->ReadData.robot.temperature[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLBoolValues()
//
void FastResearchInterface::GetKRLBoolValues(bool *KRLBoolValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_USER_SIZE; i++)
	{
		KRLBoolValues[i]	=	((this->ReadData.krl.boolData & (1 << i)) != 0);
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLIntValues()
//
void FastResearchInterface::GetKRLIntValues(int *KRLIntValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_USER_SIZE; i++)
	{
		KRLIntValues[i]	=	this->ReadData.krl.intData[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLFloatValues()
//
void FastResearchInterface::GetKRLFloatValues(float *KRLFloatValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_USER_SIZE; i++)
	{
		KRLFloatValues[i]	=	this->ReadData.krl.realData[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetKRLBoolValue()
//
bool FastResearchInterface::GetKRLBoolValue(const unsigned int &Index)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	((this->ReadData.krl.boolData & (1 << Index)) != 0);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetKRLIntValue()
//
int FastResearchInterface::GetKRLIntValue(const unsigned int &Index)
{
	int		ReturnValue		=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.krl.intData[Index];
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetKRLFloatValue()
//
float FastResearchInterface::GetKRLFloatValue(const unsigned int &Index)
{
	float		ReturnValue		=	0.0;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	this->ReadData.krl.realData[Index];
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


// ****************************************************************
// GetCurrentJacobianMatrix()
//
void FastResearchInterface::GetCurrentJacobianMatrix(float **JacobianMatrix)
{
	unsigned int		i	=	0
					,	j	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
    for (i = 0; i < FRI_CART_VEC; i++)
    {
    	for (j = 0; j < LBR_MNJ; j++)
    	{
    		//JacobianMatrix[i][j]	=	this->ReadData.data.jacobian[(i==3)?(5):((i==5)?(3):(i))*LBR_MNJ+j];
			JacobianMatrix[i][j]	=	this->ReadData.data.jacobian[i*LBR_MNJ+j];
		}
    }
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCurrentMassMatrix()
//
void FastResearchInterface::GetCurrentMassMatrix(float **MassMatrix)
{
	unsigned int		i	=	0
					,	j	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
    for (i = 0; i < LBR_MNJ; i++)
    {
    	for (j = 0; j < LBR_MNJ; j++)
    	{
    		MassMatrix[i][j]	=	this->ReadData.data.massMatrix[i*LBR_MNJ+j];
		}
    }
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// GetCurrentGravityVector()
//
void FastResearchInterface::GetCurrentGravityVector(float *GravityVector)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < LBR_MNJ; i++)
	{
		GravityVector[i]	=	this->ReadData.data.gravity[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// IsMachineOK()
//
bool FastResearchInterface::IsMachineOK(void)
{
	bool		ReturnValue		=	false;

	pthread_mutex_lock(&(this->MutexForControlData));
	ReturnValue	=	(	(this->ReadData.intf.state == FRI_STATE_CMD)
					&&	(this->ReadData.robot.power != 0)
					&&	(this->ReadData.robot.error == 0)				);
	pthread_mutex_unlock(&(this->MutexForControlData));

	return(ReturnValue);
}


