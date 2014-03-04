
#include <FastResearchInterface.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <OSAbstraction.h>
#include <FastResearchInterfaceTest.h>



#ifndef PI
#define PI	3.1415926535897932384626433832795
#endif

#define NUMBER_OF_CYCLES_FOR_QUAULITY_CHECK		2000

#define SIZE_OF_TRANSFER_STRING					32

//*******************************************************************************************
// main()

int main(int argc, char *argv[])
{
	bool					Run							=	true
						,	StartRobotCalled			=	false;

	char					c							=	0
						,	d							=	0
						,	TransferString[SIZE_OF_TRANSFER_STRING];

	unsigned int			ControlScheme				=	FastResearchInterface::JOINT_POSITION_CONTROL
						,	i							=	0
						,	LoopValue					=	0;

	int						ResultValue					=	0;

	float					FloatValues[FRI_USER_SIZE]
	     				,	TmpFloatValues[FRI_USER_SIZE]
	     				,	DesiredTorqueValues[LBR_MNJ]
	     				,	JointStiffnessValues[LBR_MNJ]
	     				,	JointDampingValues[LBR_MNJ]
	     				,	CartStiffnessValues[FRI_CART_VEC]
	     				,	CartDampingValues[FRI_CART_VEC];

	FastResearchInterface	*FRI;



	memset(TransferString		, 0x0	, SIZE_OF_TRANSFER_STRING	* sizeof(char)	);
	memset(FloatValues			, 0x0	, FRI_USER_SIZE				* sizeof(float)	);
	memset(TmpFloatValues		, 0x0	, FRI_USER_SIZE				* sizeof(float)	);
	memset(DesiredTorqueValues	, 0x0	, LBR_MNJ					* sizeof(float)	);

#ifdef WIN32
	FRI = new FastResearchInterface("E:\\Stanford\\Research\\SourceCode\\LWR_Public\\etc\\980039-FRI-Driver.init");
#endif

#ifdef __LINUX__
	fprintf(stdout, "You may need superuser permission to run this program.\n");
	fflush(stdout);
	FRI = new FastResearchInterface("/home/crosales/Code/FRILibrary/etc/980494-FRI-Driver.init");
#endif

#ifdef __MACOS__
	FRI = new FastResearchInterface("/Users/torsten/Documents/SourceCode/LWR_Public/LWR_Public/etc/980039-FRI-Driver.init");
#endif

#ifdef _NTO_
	FRI = new FastResearchInterface("/home/lwrcontrol/etc/980039-FRI-Driver2ms.init");
#endif

	for (i = 0; i < LBR_MNJ; i++)
	{
		JointStiffnessValues	[i] =	(float)10.0;
		JointDampingValues		[i]	=	(float)0.7;
	}

	for (i = 0; i < FRI_CART_VEC; i++)
	{
		CartStiffnessValues		[i]	=	(float)10.0;
		CartDampingValues		[i]	=	(float)0.7;
	}

	FRI->SetCommandedCartDamping(CartStiffnessValues);
	FRI->SetCommandedCartStiffness(CartDampingValues);
	FRI->SetCommandedJointDamping(JointDampingValues);
	FRI->SetCommandedJointStiffness(JointStiffnessValues);

	while (Run)
	{
		printf("---------------------------------------------------------------------------------------\n");
		printf("Press     q  for exit this program\n");
		printf("          s  for starting the KUKA Fast Research Interface\n");
		printf("          x  for stopping the KUKA Fast Research Interface\n");
		printf("          p  for printing system information\n");
		printf("          d  for changing 'D' (damping term) of the joint impedance controller\n");
		printf("          k  for changing 'k' (stiffness term) of the joint impedance controller\n");
		printf("          e  for changing 'D' (damping term) of the Cartesian impedance controller\n");
		printf("          l  for changing 'k' (stiffness term) of the Cartesian impedance controller\n");
		printf("          m  for getting the current parameters of the joint impedance controller\n");
		printf("          n  for getting the current parameters of the Cartesian impedance controller\n");
		printf("          g  for getting the current joint positions\n");
		printf("          h  for getting the current joint torques\n");
		printf("          i  for starting writing to an output file\n");
		printf("          j  for stopping writing to an output file\n");
		printf("          t  for start the joint position controller and run a simple trajectory\n");
		printf("          c  for moving to the candle position\n");
		printf("          r  for record points\n");
		printf("---------------------------------------------------------------------------------------\n\n");
		printf("Please press any key...\n");

		c	=	WaitForKBCharacter(NULL);
		
		printf("\n\n\n");

		switch (c)
		{
		case 'q':
		case 'Q':
			Run	=	false;
			break;
		case 'r':
			d	=	0;
			printf("r for record poinf, p for play points recorded, t for recor trajectory, w for play trayectory recorded\n");
			while ( (d != 'r') && (d != 'p') && (d != 't') && (d != 'w') )
			{
				d	=	WaitForKBCharacter(NULL);
				printf("%c\n", c);
			}
			if (d == 'r')
				RecordPointsSimple(FRI);
			if (d == 'p')
				PlayPointsSimple(FRI);
			/*if (d == 't')
				RecordTrajectorySimple(FRI);
			if (d == 'w')
				PlayTrajectorySimple(FRI);*/
			break;
		case 's':
		case 'S':
			printf("Starting the robot through the FRI...\n");
			printf("Please select one of the following control strategies:\n\n");
			printf(" 1: Joint position control\n");
			printf(" 2: Cartesian impedance control\n");
			printf(" 3: Joint impedance control\n");
			printf(" 9: Joint torque control\n");
			printf(" a: Abort\n\n");
			d	=	0;
			while ( (d != '1') && (d != '2') && (d != '3') && (d != '9') && (d != 'a') && (d != 'A'))
			{
				d	=	WaitForKBCharacter(NULL);
				printf("%c\n", c);
			}
			if ( (d == 'a') || (d == 'A'))
			{
				printf("Control strategy remains unchanged.\n");
				break;
			}

			switch (d)
			{
			case '1':
				ControlScheme	=	FastResearchInterface::JOINT_POSITION_CONTROL;
				printf("Control strategy set to joint position control.\n");
				break;
			case '2':
				ControlScheme	=	FastResearchInterface::CART_IMPEDANCE_CONTROL;
				printf("Control strategy set to Cartesian impedance control.\n");
				break;
			case '3':
				ControlScheme	=	FastResearchInterface::JOINT_IMPEDANCE_CONTROL;
				printf("Control strategy set to joint impedance control.\n");
				break;
			}

			ResultValue	=	FRI->StartRobot(ControlScheme);

			if (ResultValue != EOK)
			{
				printf("An error occurred during starting up the robot...\n");
			}
			else
			{
				StartRobotCalled	=	true;
			}
			break;
		case 'x':
		case 'X':
			printf("Stopping the FRI...\n");
			ResultValue	=	FRI->StopRobot();
			StartRobotCalled	=	false;

			if (ResultValue != EOK)
			{
				printf("An error occurred during stopping the robot...\n");
			}
			break;
		case 'p':
		case 'P':
			printf("Printing system information...\n");
			printf("%s\n", FRI->GetCompleteRobotStateAndInformation());
			delay(200);
			break;
		case 'd':
		case 'D':
		case 'k':
		case 'K':
		case 'e':
		case 'E':
		case 'l':
		case 'L':
			if ( (c == 'd') || (c == 'D'))
			{
				printf("Changing the damping term of the joint impedance controller...\n");
			}
			else
			{
				if ( (c == 'k') || (c == 'K'))
				{
					printf("Changing the stiffness term of the joint impedance controller...\n");
				}
				else
				{
					if ( (c == 'e') || (c == 'E') )
					{
						printf("Changing the damping term of the Cartesian impedance controller...\n");
					}
					else
					{
						printf("Changing the stiffness term of the Cartesian impedance controller...\n");
					}
				}

			}

			if ( (c == 'd') || (c == 'D') || (c == 'k') || (c == 'K') )
			{
				LoopValue	=	 LBR_MNJ;
			}
			else
			{
				LoopValue	=	 FRI_CART_VEC;
			}

			printf("\nWould you like to enter one value for all vector elements (a) or for each individual one (i)?\n");

			d	=	0;
			while ( (d != 'a') && (d != 'A') && (d != 'i') && (d != 'I') )
			{
				d	=	WaitForKBCharacter(NULL);
				printf("%c\n", c);
			}

			if ( (d == 'a') || (d == 'A') )
			{
				printf("Please enter a new value for all vector elements:\n");
				printf(">");
				int tmp = scanf("%s", TransferString);	// "int tmp" to supress compiler warnings
				for (i = 0; i < LoopValue; i++)
				{
					FloatValues[i] = atof(TransferString);
				}
			}
			else
			{
				if ( (d == 'i') || (d == 'I') )
				{
					printf("Please enter new values for each vector element:\n");
					for (i = 0; i < LoopValue; i++)
					{
						printf("Element %d >", i);
						int tmp = scanf("%s", TransferString);	// "int tmp" to supress compiler warnings
						FloatValues[i] = atof(TransferString);
					}
				}
			}

			if ( (c == 'd') || (c == 'D'))
			{
				printf("New damping term of the joint impedance controller:");
			}
			else
			{
				if ( (c == 'k') || (c == 'K'))
				{
					printf("New stiffness term of the joint impedance controller:");
				}
				else
				{
					if ( (c == 'e') || (c == 'L'))
					{
						printf("New damping term of the Cartesian impedance controller:");
					}
					else
					{
						printf("New stiffness term of the Cartesian impedance controller:");
					}
				}
			}

			for (i = 0; i < LoopValue; i++)
			{
				printf("%8.3f " , FloatValues[i]);
			}

			printf("\n\nIs this correct? (y/n)\n");
			d	=	0;
			while ( (d != 'y') && (d != 'Y') && (d != 'n') && (d != 'N') )
			{
				d	=	WaitForKBCharacter(NULL);
				printf("%c\n", c);
			}

			if ( (d == 'y') || (d == 'Y') )
			{
				if ( (c == 'd') || (c == 'D'))
				{
					FRI->SetCommandedJointDamping(FloatValues);
					for (i = 0; i < LoopValue; i++)
					{
						JointDampingValues[i]	=	FloatValues[i];
					}
					printf("New joint damping values are set.\n");
				}
				else
				{
					if ( (c == 'k') || (c == 'K'))
					{
						FRI->SetCommandedJointStiffness(FloatValues);
						for (i = 0; i < LoopValue; i++)
						{
							JointStiffnessValues[i]	=	FloatValues[i];
						}
						printf("New joint stiffness values are set.\n");
					}
					else
					{
						if ( (c == 'e') || (c == 'L'))
						{
							FRI->SetCommandedCartDamping(FloatValues);
							for (i = 0; i < LoopValue; i++)
							{
								CartDampingValues[i]	=	FloatValues[i];
							}
							printf("New Cartesian damping values are set.\n");
						}
						else
						{
							FRI->SetCommandedCartStiffness(FloatValues);
							for (i = 0; i < LoopValue; i++)
							{
								CartStiffnessValues[i]	=	FloatValues[i];
							}
							printf("New Cartesian stiffness values are set.\n");
						}
					}
				}
			}
			else
			{
				printf("Values remain unchanged.\n");
			}
			break;
		case 'g':
		case 'G':
			printf("Getting joint position values...\n");
			FRI->GetMeasuredJointPositions(FloatValues);
			for (i = 0; i < LBR_MNJ; i++)
			{
				printf("Joint position %d: %8.3f degrees\n", i, FloatValues[i] * 180.0 / PI);
			}
			printf("\n\n");
			break;
		case 'm':
		case 'M':
			printf("Getting the current parameters of the joint impedance controller\n");
			for (i = 0; i < LBR_MNJ; i++)
			{
				printf("Joint stiffness values %d: %8.3f Nm\n", i, JointStiffnessValues[i]);
			}
			printf("\n");
			for (i = 0; i < LBR_MNJ; i++)
			{
				printf("Joint damping values %d: %8.3f Nms\n", i, JointDampingValues[i]);
			}
			printf("\n");
			break;
		case 'n':
		case 'N':
			printf("Getting the current parameters of the Cartesian impedance controller\n");
			for (i = 0; i < 3; i++)
			{
				printf("Cartesian stiffness values %d: %8.3f N/m\n", i, CartStiffnessValues[i]);
			}
			for (i = 3; i < 6; i++)
			{
				printf("Cartesian stiffness values %d: %8.3f Nm\n", i, CartStiffnessValues[i]);
			}
			printf("\n");
			for (i = 0; i < 3; i++)
			{
				printf("Cartesian damping values %d: %8.3f Ns/m\n", i, CartDampingValues[i]);
			}
			for (i = 3; i < 6; i++)
			{
				printf("Cartesian damping values %d: %8.3f Nms\n", i, CartDampingValues[i]);
			}
			printf("\n");
			break;
		case 'h':
		case 'H':
			printf("Getting joint torque values...\n");
			FRI->GetMeasuredJointTorques(FloatValues);
			for (i = 0; i < LBR_MNJ; i++)
			{
				printf("Joint position %d: %8.3f Nm\n", i, FloatValues[i]);
			}
			printf("\n\n");
			break;
		case 'i':
		case 'I':
			printf("Starting to write an output file...\n");
			ResultValue	=	FRI->PrepareLogging("Test");
			if (ResultValue == EOK)
			{
				printf("Logging successfully prepared.\n");
			}
			else
			{
				printf( "Error at FRI->PrepareLogging(): %s\n", strerror(ResultValue));
			}

			ResultValue	=	FRI->StartLogging();
			if (ResultValue == EOK)
			{
				printf("Logging successfully started.\n");
			}
			else
			{
				printf( "Error at FRI->StartLogging(): %s\n", strerror(ResultValue));
			}
			break;

		case 'j':
		case 'J':
			printf("Stopping to write an output file...\n");
			ResultValue	=	FRI->StopLogging();
			if (ResultValue == EOK)
			{
				printf("Logging successfully stopped.\n");
			}
			else
			{
				printf( "Error at FRI->StopLogging(): %s\n", strerror(ResultValue));
			}

			ResultValue	=	FRI->WriteLoggingDataFile();
			if (ResultValue == EOK)
			{
				printf("Logging data file successfully written.\n");
			}
			else
			{
				printf( "Error at FRI->WriteLoggingDataFile(): %s\n", strerror(ResultValue));
			}
			break;

		case 't':
		case 'T':
			printf("Moving along a simple trajectory... (please wait)\n");
			RunTrajectorySimple(FRI);
			printf("Motion completed.\n");
			break;
			
		case 'c':
		case 'C':
			printf("Moving to the candle position... (please wait)\n");
			MoveToCandle(FRI);
			break;

		default:
			printf("This key is not supported yet...\n");
			break;
		}
	}

	delete FRI;

	printf("\nGood bye.\n\n");

	return(EXIT_SUCCESS);
}
