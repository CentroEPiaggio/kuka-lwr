#include <FastResearchInterface.h>
#include <Console.h>
#include <errno.h>
#include <string.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>
#include <TypeIRML.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

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
// RunTrajectorySimple()
//
void RunTrajectorySimple(FastResearchInterface *FRI)
{
	unsigned int				i							=	0		;

	int							ResultValue					=	0		;


	float						JointValuesInRad[NUMBER_OF_JOINTS]		;
	float						errorJoint[NUMBER_OF_JOINTS]		;

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
		
	printf("Moving each joint by 20 degrees...\n");
	
	FRI->GetMeasuredJointPositions(JointValuesInRad);
	
	for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[0]	=	(double)-87.08;
		IP->TargetPosition->VecData			[1]	=	(double)45.04;
		IP->TargetPosition->VecData			[2]	=	(double)57.88;
		IP->TargetPosition->VecData			[3]	=	(double)-106.76;
		IP->TargetPosition->VecData			[4]	=	(double)-11.72;
		IP->TargetPosition->VecData			[5]	=	(double)-8.27;
		IP->TargetPosition->VecData			[6]	=	(double)-17.77;
		IP->MaxVelocity->VecData			[i] =	(double)50.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)40.0;
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
			printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}
	
	for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[0]	=	(double)-81.36;
		IP->TargetPosition->VecData			[1]	=	(double)75.56;
		IP->TargetPosition->VecData			[2]	=	(double)68.17;
		IP->TargetPosition->VecData			[3]	=	(double)-101.91;
		IP->TargetPosition->VecData			[4]	=	(double)-136.91;
		IP->TargetPosition->VecData			[5]	=	(double)10.54;
		IP->TargetPosition->VecData			[6]	=	(double)116.49;
		IP->MaxVelocity->VecData			[i] =	(double)100.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)50.0;
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
			printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}

	for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[0]	=	(double)-81.93;
		IP->TargetPosition->VecData			[1]	=	(double)98.24;
		IP->TargetPosition->VecData			[2]	=	(double)78.36;
		IP->TargetPosition->VecData			[3]	=	(double)-106.93;
		IP->TargetPosition->VecData			[4]	=	(double)-143.08;
		IP->TargetPosition->VecData			[5]	=	(double)29.34;
		IP->TargetPosition->VecData			[6]	=	(double)116.49;
		IP->MaxVelocity->VecData			[i] =	(double)100.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)50.0;
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
			printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}
	
	
	for ( i = 0; i < NUMBER_OF_JOINTS; i++)  
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[0]	=	(double)-70.24;
		IP->TargetPosition->VecData			[1]	=	(double)82.11;
		IP->TargetPosition->VecData			[2]	=	(double)71.0;
		IP->TargetPosition->VecData			[3]	=	(double)-98.51;
		IP->TargetPosition->VecData			[4]	=	(double)-136.87;
		IP->TargetPosition->VecData			[5]	=	(double)-5.2;
		IP->TargetPosition->VecData			[6]	=	(double)116.49;
		IP->MaxVelocity->VecData			[i] =	(double)100.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)50.0;
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
			printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}
	
	
for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[0]	=	(double)-60.46;
		IP->TargetPosition->VecData			[1]	=	(double)98.29;
		IP->TargetPosition->VecData			[2]	=	(double)68.96;
		IP->TargetPosition->VecData			[3]	=	(double)-96.37;
		IP->TargetPosition->VecData			[4]	=	(double)-136.91;
		IP->TargetPosition->VecData			[5]	=	(double)-6.12;
		IP->TargetPosition->VecData			[6]	=	(double)116.49;
		IP->MaxVelocity->VecData			[i] =	(double)100.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)50.0;
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
			printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}

	for ( i = 0; i < NUMBER_OF_JOINTS; i++)
	{
		IP->CurrentPosition->VecData		[i] =	(double)DEG(JointValuesInRad[i]);
		IP->TargetPosition->VecData			[0]	=	(double)-71.75;
		IP->TargetPosition->VecData			[1]	=	(double)59.91;
		IP->TargetPosition->VecData			[2]	=	(double)71.52;
		IP->TargetPosition->VecData			[3]	=	(double)-102.22;
		IP->TargetPosition->VecData			[4]	=	(double)-43.62;
		IP->TargetPosition->VecData			[5]	=	(double)-0.60;
		IP->TargetPosition->VecData			[6]	=	(double)-18.67;
		IP->MaxVelocity->VecData			[i] =	(double)100.0;		
		IP->MaxAcceleration->VecData		[i] =	(double)50.0;
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
			printf("RunTrajectorySimple(): ERROR during trajectory generation (%d).", ResultValue);
		}

		FRI->SetCommandedJointPositions(JointValuesInRad);	
		
		*(IP->CurrentPosition)		=	*(OP->NewPosition);
		*(IP->CurrentVelocity)		=	*(OP->NewVelocity);
	}	
	

	if (!FRI->IsMachineOK())
	{
		printf("RunTrajectorySimple(): ERROR, machine is not ready.");
		
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


