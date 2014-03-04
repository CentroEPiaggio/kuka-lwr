//  ---------------------- Doxygen info ----------------------
//! \file StartRobot.cpp
//!
//! \brief
//! Implementation file for the method StartRobot() of the class FastResearchInterface
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
#include <Console.h>
#include <friComm.h>
#include <OSAbstraction.h>
#include <errno.h>
#include <string.h>



// ****************************************************************
// StartRobot()
//
int FastResearchInterface::StartRobot(		const unsigned int &ControlMode
                                      	,	const float &TimeOutValueInSeconds)
{
	int					ResultValue				=	0;

	float				CommandValues[2 * LBR_MNJ]
					,	InitialSystemTimeValue	=	GetSystemTimeInSeconds(true);

	memset(CommandValues, 0x0, 2 * LBR_MNJ * sizeof(float));

	if (this->GetFRIMode() == FRI_STATE_OFF)
	{
		// wait until the FRI is in monitor mode
		while (		((GetSystemTimeInSeconds() - InitialSystemTimeValue)	<	TimeOutValueInSeconds	)
				&&	(this->GetFRIMode()										!=	FRI_STATE_MON			))
		{
			delay(1);
		}

		if (this->GetFRIMode() == FRI_STATE_OFF)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC has not opened the FRI yet or no UDP connection can be established.\n");
			return(ENOTCONN);
		}
	}

	if (this->GetFRIMode() == FRI_STATE_CMD)
	{
		if (this->CurrentControlScheme	== ControlMode)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): The robot has already been started.\n");
			return(EALREADY);
		}
		else
		{
			this->StopRobot();
		}
	}

	if (this->GetKRLIntValue(15) == 20)
	{
		this->StopRobot();
	}


	this->CurrentControlScheme	=	ControlMode;

	this->SetControlScheme(this->CurrentControlScheme);

	// The KRL program running on the robot controller waits for a
	// integer value change at position 15 (i.e., 16 in KRL).
	// A value of 10 lets the KRL program call friStart()
	this->SetKRLIntValue(15, 10);

	// wait for the next data telegram of the KRC unit
	pthread_mutex_lock(&(this->MutexForControlData));
	this->NewDataFromKRCReceived	=	false;
	pthread_mutex_unlock(&(this->MutexForControlData));
	ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

	while (		((GetSystemTimeInSeconds() - InitialSystemTimeValue)	< TimeOutValueInSeconds	)
			&&	(this->GetFRIMode()										!= FRI_STATE_CMD		)	)
	{
		ResultValue	=	this->WaitForKRCTick((unsigned int)(this->CycleTime * 3000000.0));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
			return(ENOTCONN);
		}
	}

	if (this->GetFRIMode() == FRI_STATE_CMD)
	{
		while (		((GetSystemTimeInSeconds() - InitialSystemTimeValue) < TimeOutValueInSeconds)
				&&	(!(this->IsMachineOK()))
				&&	(this->GetKRLIntValue(15) != 20)	)
		{
			ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

			if (ResultValue != EOK)
			{
				this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
				return(ENOTCONN);
			}
		}

		if (this->IsMachineOK())
		{
			delay(12);	// KUKA interpolation cycle time
			return(EOK);
		}
		else
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, command mode reached, but the robot could be turned on correctly. Check KRC state.");
			return(ETIME);
		}
	}
	else
	{
		this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, could not switch to command mode after timeout.\n");
		return(ETIME);
	}
}

