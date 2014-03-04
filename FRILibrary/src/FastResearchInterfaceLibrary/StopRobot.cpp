//  ---------------------- Doxygen info ----------------------
//! \file StopRobot.cpp
//!
//! \brief
//! Implementation file for the method StopRobot of the class FastResearchInterface
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
#include <errno.h>
#include <OSAbstraction.h>

#define TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE	10.0


// ****************************************************************
// StartRobot()
//
int FastResearchInterface::StopRobot(void)
{
	unsigned int		DataTelegramCounter		=	0;

	int					ResultValue				=	0;

	// The KRL program running on the robot controller waits for a
	// integer value change at position 15 (i.e., 16 in KRL).
	// A value of 20 lets the KRL program call friStop()
	this->SetKRLIntValue(15, 20);

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

	// wait until we are in monitor mode again
	while (		((double)DataTelegramCounter * this->CycleTime	<	TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE)
			&&	(this->GetFRIMode()								!=	FRI_STATE_MON		)	)
	{
		ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
			return(ENOTCONN);
		}

		DataTelegramCounter++;
	}

	// wait until the KRC unit ready for the reception of the next command
	while (		((double)DataTelegramCounter * this->CycleTime	<	TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE)
			&&	(this->GetKRLIntValue(15) != 10)	)
	{
		ResultValue	=	this->WaitForKRCTick(((unsigned int)(this->CycleTime * 3000000.0)));

		if (ResultValue != EOK)
		{
			this->OutputConsole->printf("FastResearchInterface::StartRobot(): ERROR, the KRC unit does not respond anymore. Probably, the FRI was closed or there is no UDP connection anymore.");
			return(ENOTCONN);
		}

		DataTelegramCounter++;
	}

	if ((double)DataTelegramCounter * this->CycleTime	<	TIMEOUT_VALUE_IN_SECONDS_TO_REACH_MONITOR_MODE)
	{
		delay(12);	// KUKA interpolation cycle time
		return(EOK);
	}
	else
	{
		return(ETIME);
	}
}

