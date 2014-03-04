//  ---------------------- Doxygen info ----------------------
//! \file SetRobotStatusData.cpp
//!
//! \brief
//! Implementation file for several methods of the class FastResearchInterface for
//! setting robot status data by the user application of this class
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
// SetKRLBoolValues()
//
void FastResearchInterface::SetKRLBoolValues(const bool *KRLBoolValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_USER_SIZE; i++)
	{
		if (KRLBoolValues[i])
		{
			this->CommandData.krl.boolData |= (1 << i);
		}
		else
		{
			this->CommandData.krl.boolData &= ( ~( 1 << i) );
		}
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetKRLIntValues()
//
void FastResearchInterface::SetKRLIntValues(const int *KRLIntValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_USER_SIZE; i++)
	{
		this->CommandData.krl.intData[i]	=	KRLIntValues[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetKRLFloatValues()
//
void FastResearchInterface::SetKRLFloatValues(const float *KRLFloatValues)
{
	unsigned int		i	=	0;

	pthread_mutex_lock(&(this->MutexForControlData));
	for (i = 0; i < FRI_USER_SIZE; i++)
	{
		this->CommandData.krl.realData[i]	=	KRLFloatValues[i];
	}
	pthread_mutex_unlock(&(this->MutexForControlData));

	return;
}


// ****************************************************************
// SetKRLBoolValue()
//
void FastResearchInterface::SetKRLBoolValue(	const unsigned int	&Index
                                            ,	const bool			&Value	)
{
	pthread_mutex_lock(&(this->MutexForControlData));
	if (Value)
	{
		this->CommandData.krl.boolData |= (1 << Index);
	}
	else
	{
		this->CommandData.krl.boolData &= ( ~( 1 << Index) );
	}
	pthread_mutex_unlock(&(this->MutexForControlData));
	return;
}


// ****************************************************************
// SetKRLIntValue()
//
void FastResearchInterface::SetKRLIntValue(		const unsigned int	&Index
                                           	,	const int			&Value	)
{
	pthread_mutex_lock(&(this->MutexForControlData));
	this->CommandData.krl.intData[Index]	=	Value;
	pthread_mutex_unlock(&(this->MutexForControlData));
	return;
}


// ****************************************************************
// SetKRLFloatValue()
//
void FastResearchInterface::SetKRLFloatValue(	const unsigned int	&Index
                                             ,	const float			&Value	)
{
	pthread_mutex_lock(&(this->MutexForControlData));
	this->CommandData.krl.realData[Index]	=	Value;
	pthread_mutex_unlock(&(this->MutexForControlData));
	return;
}

