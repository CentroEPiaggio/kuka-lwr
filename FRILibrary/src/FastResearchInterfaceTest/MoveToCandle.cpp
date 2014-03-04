//  ---------------------- Doxygen info ----------------------
//! \file MoveToCandle.cpp
//!
//! \brief
//! Implementation file for performing a joint space motion to 
//! the candle position of the robot
//!
//! \details
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
#include <string.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <TypeIRML.h>

#ifndef PI
#define PI			3.1415926535897932384626433832795
#endif

#ifndef RAD
#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
#define DEG(A)	((A) * 180.0 / PI )
#endif

#define NUMBER_OF_JOINTS			7



// ****************************************************************
// MoveToCandle()
//
void MoveToCandle(FastResearchInterface *FRI)
{
	unsigned int				i							=	0		;

	int							ResultValue					=	0		;


	float						JointValuesInRad[NUMBER_OF_JOINTS]		;

	double						CycleTime					=	0.002	;

	TypeIRML					*RML						=	NULL	;

	TypeIRMLInputParameters		*IP							=	NULL	;

	TypeIRMLOutputParameters	*OP							=	NULL	;

	RML					=	new TypeIRML(		NUMBER_OF_JOINTS
											,	CycleTime			);

	IP					=	new TypeIRMLInputParameters(NUMBER_OF_JOINTS);

	OP					=	new TypeIRMLOutputParameters(NUMBER_OF_JOINTS);

	memset(JointValuesInRad						, 0x0, NUMBER_OF_JOINTS * sizeof(float));

	if ((FRI->GetCurrentControlScheme() != FastResearchInterface::JOINT_POSITION_CONTROL) || (!FRI->IsMachineOK()))
	{	
		printf("Program is going to stop the robot.\n");
		FRI->StopRobot();

		FRI->GetMeasuredJointPositions(JointValuesInRad);
		FRI->SetCommandedJointPositions(JointValuesInRad);
				
		printf("Restarting the joint position control scheme.\n");
		ResultValue	=	FRI->StartRobot(FastResearchInterface::JOINT_POSITION_CONTROL);
		
		if ((ResultValue != EOK) && (ResultValue != EALREADY))
		{
			printf("An error occurred during starting up the robot...\n");
			delete	RML;
			delete	IP;
			delete	OP;
			
			return;			
		}
	}
		
	printf("Moving to the candle position..\n");
	
	FRI->GetMeasuredJointPositions(JointValuesInRad);
	
	for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[i]	=	(double)0.0;
		IP->MaxVelocity->VecData			[i] =	(double)130.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)20.0;
		IP->SelectionVector->VecData		[i] =	true;
	}
	
	ResultValue	=	TypeIRML::RML_WORKING;
	
	while ((FRI->IsMachineOK()) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
	{
		FRI->WaitForKRCTick();

		ResultValue	=	RML->GetNextMotionState_Position(		*IP
															,	OP	);

		if ((ResultValue != TypeIRML::RML_WORKING) && (ResultValue != TypeIRML::RML_FINAL_STATE_REACHED))
		{
			printf("MoveToCandle(): ERROR during trajectory generation (%d).", ResultValue);
		}
											
		for ( i = 0; i < NUMBER_OF_JOINTS; i++)
		{
			JointValuesInRad[i]	=	RAD((double)(OP->NewPosition->VecData[i]));
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}

	if (!FRI->IsMachineOK())
	{
		printf("MoveToCandle(): ERROR, machine is not ready.");
		
		delete	RML;
		delete	IP;
		delete	OP;
		
		return;
	}

	printf("Stopping the robot.\n");
	FRI->StopRobot();	

	delete	RML;
	delete	IP;
	delete	OP;
	
	return;
}
