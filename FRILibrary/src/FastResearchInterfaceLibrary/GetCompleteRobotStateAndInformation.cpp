//  ---------------------- Doxygen info ----------------------
//! \file GetCompleteRobotStateAndInformation.cpp
//!
//! \brief
//! Implementation file for the GetCompleteRobotStateAndInformation method
//! of the class FastResearchInterface
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
#include <string.h>
#include <OSAbstraction.h>

#define	SINGLE_LINE		"-----------------------------------------------------------------------------"
#define	DOUBLE_LINE		"============================================================================="


#ifndef PI
#define PI	((float)3.1415926535897932384626433832795)
#endif


// ****************************************************************
// GetCompleteRobotStateAndInformation()
//
const char* FastResearchInterface::GetCompleteRobotStateAndInformation(void)
{
	bool			BoolArray[FRI_USER_SIZE];

	char			StringPart[512];

	unsigned int	CharCounter			=	0
				,	i					=	0;

	int				IntArray[FRI_USER_SIZE];

	float			FloatArray[FRI_USER_SIZE]
	     		,	FloatMatrix[LBR_MNJ][FRI_CART_VEC];

	memset((void*)StringPart	, 0x0	,	512						* sizeof(char)	);
	memset((void*)FloatArray	, 0x0	,	FRI_USER_SIZE			* sizeof(float)	);
	memset((void*)IntArray		, 0x0	,	FRI_USER_SIZE			* sizeof(float)	);
	memset((void*)BoolArray		, 0x0	,	FRI_USER_SIZE			* sizeof(bool)	);

	for (i = 0; i < LBR_MNJ; i++)
	{
		memset((void*)(FloatMatrix[i])	, 0x0	,	FRI_CART_VEC	* sizeof(float)	);
	}

	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, DOUBLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "Overview about the KUKA Light-Weight Robot IV and the Fast Research Interface");
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, DOUBLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "UDP Communication Data");
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, SINGLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "Cycle time: %f seconds", this->GetFRICycleTime());
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	switch (this->GetCommunicationTimingQuality())
	{
	case FRI_QUALITY_UNACCEPTABLE:
		sprintf(StringPart, "UDP communication quality: Unacceptable");
		break;
	case FRI_QUALITY_BAD:
		sprintf(StringPart, "UDP communication quality: Bad");
		break;
	case FRI_QUALITY_OK:
		sprintf(StringPart, "UDP communication quality: Ok");
		break;
	case FRI_QUALITY_PERFECT:
		sprintf(StringPart, "UDP communication quality: Perfect");
		break;
	default:
		sprintf(StringPart, "UDP communication quality: UNKNOWN");
		break;
	}
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "UDP answer rate: %f percent", 100.0 * this->GetUDPAnswerRate());
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "UDP latency: %f seconds", this->GetUDPLatencyInSeconds());
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "UDP jitter: %f seconds", this->GetUDPJitterInSeconds());
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "UDP package loss rate: %f percent", 100.0 * this->GetUDPPackageLossRate());
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "Absolute number of missed packages: %d FRI data telegram packages", this->GetNumberOfMissedUDPPackages());
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "Sequence counter: %d", this->GetValueOfKRCSequenceCounter());
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, SINGLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "KRL Data");
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, SINGLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetKRLBoolValues(BoolArray);
	sprintf(StringPart, "Bool array: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d"
			,	BoolArray[ 0]
			,	BoolArray[ 1]
			,	BoolArray[ 2]
			,	BoolArray[ 3]
			,	BoolArray[ 4]
			,	BoolArray[ 5]
			,	BoolArray[ 6]
			,	BoolArray[ 7]
			,	BoolArray[ 8]
			,	BoolArray[ 9]
			,	BoolArray[10]
			,	BoolArray[11]
			,	BoolArray[12]
			,	BoolArray[13]
			,	BoolArray[14]
			,	BoolArray[15]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetKRLIntValues(IntArray);
	sprintf(StringPart, "Int array: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d"
			,	IntArray[ 0]
			,	IntArray[ 1]
			,	IntArray[ 2]
			,	IntArray[ 3]
			,	IntArray[ 4]
			,	IntArray[ 5]
			,	IntArray[ 6]
			,	IntArray[ 7]
			,	IntArray[ 8]
			,	IntArray[ 9]
			,	IntArray[10]
			,	IntArray[11]
			,	IntArray[12]
			,	IntArray[13]
			,	IntArray[14]
			,	IntArray[15]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetKRLFloatValues(FloatArray);
	sprintf(StringPart, "Float array: %4.f %4.f %4.f %4.f %4.f %f %4.f %4.f %4.f %4.f %4.f %4.f %4.f %4.f %4.f %4.f"
			,	FloatArray[ 0]
			,	FloatArray[ 1]
			,	FloatArray[ 2]
			,	FloatArray[ 3]
			,	FloatArray[ 4]
			,	FloatArray[ 5]
			,	FloatArray[ 6]
			,	FloatArray[ 7]
			,	FloatArray[ 8]
			,	FloatArray[ 9]
			,	FloatArray[10]
			,	FloatArray[11]
			,	FloatArray[12]
			,	FloatArray[13]
			,	FloatArray[14]
			,	FloatArray[15]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, SINGLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "Robot Control Data");
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, SINGLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetMeasuredJointPositions(FloatArray);
	sprintf(StringPart, "Measured joint positions [deg]: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f"
	        ,	FloatArray[0] * 180.0 / PI
	        ,	FloatArray[1] * 180.0 / PI
	        ,	FloatArray[2] * 180.0 / PI
	        ,	FloatArray[3] * 180.0 / PI
	        ,	FloatArray[4] * 180.0 / PI
	        ,	FloatArray[5] * 180.0 / PI
	        ,	FloatArray[6] * 180.0 / PI);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetCommandedJointPositions(FloatArray);
	sprintf(StringPart, "Commanded joint positions [deg]: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f"
	        ,	FloatArray[0] * 180.0 / PI
	        ,	FloatArray[1] * 180.0 / PI
	        ,	FloatArray[2] * 180.0 / PI
	        ,	FloatArray[3] * 180.0 / PI
	        ,	FloatArray[4] * 180.0 / PI
	        ,	FloatArray[5] * 180.0 / PI
	        ,	FloatArray[6] * 180.0 / PI);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetCommandedJointPositionOffsets(FloatArray);
	sprintf(StringPart, "Commanded joint position offsets [deg]: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f"
	        ,	FloatArray[0] * 180.0 / PI
	        ,	FloatArray[1] * 180.0 / PI
	        ,	FloatArray[2] * 180.0 / PI
	        ,	FloatArray[3] * 180.0 / PI
	        ,	FloatArray[4] * 180.0 / PI
	        ,	FloatArray[5] * 180.0 / PI
	        ,	FloatArray[6] * 180.0 / PI);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetMeasuredJointTorques(FloatArray);
	sprintf(StringPart, "Measured joint torques [Nm]: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f"
	        ,	FloatArray[0]
	        ,	FloatArray[1]
	        ,	FloatArray[2]
	        ,	FloatArray[3]
	        ,	FloatArray[4]
	        ,	FloatArray[5]
	        ,	FloatArray[6]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetEstimatedExternalJointTorques(FloatArray);
	sprintf(StringPart, "Estimated external joint torques [Nm]: %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %8.3f"
	        ,	FloatArray[0]
	        ,	FloatArray[1]
	        ,	FloatArray[2]
	        ,	FloatArray[3]
	        ,	FloatArray[4]
	        ,	FloatArray[5]
	        ,	FloatArray[6]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetMeasuredCartPose(FloatArray);
	sprintf(StringPart, "Measured Cart. pose frame: ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 0]
	        ,	FloatArray[ 1]
	        ,	FloatArray[ 2]
	        ,	FloatArray[ 3]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                           ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 4]
	        ,	FloatArray[ 5]
	        ,	FloatArray[ 6]
	        ,	FloatArray[ 7]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                           ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 8]
	        ,	FloatArray[ 9]
	        ,	FloatArray[10]
	        ,	FloatArray[11]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                           ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	0.0
	        ,	0.0
	        ,	0.0
	        ,	1.0);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetCommandedCartPose(FloatArray);
	sprintf(StringPart, "Commanded Cart. pose frame: ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 0]
	        ,	FloatArray[ 1]
	        ,	FloatArray[ 2]
	        ,	FloatArray[ 3]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                            ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 4]
	        ,	FloatArray[ 5]
	        ,	FloatArray[ 6]
	        ,	FloatArray[ 7]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                            ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 8]
	        ,	FloatArray[ 9]
	        ,	FloatArray[10]
	        ,	FloatArray[11]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                            ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	0.0
	        ,	0.0
	        ,	0.0
	        ,	1.0);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetCommandedCartPoseOffsets(FloatArray);
	sprintf(StringPart, "Commanded Cart. pose offset frame: ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 0]
	        ,	FloatArray[ 1]
	        ,	FloatArray[ 2]
	        ,	FloatArray[ 3]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                                   ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 4]
	        ,	FloatArray[ 5]
	        ,	FloatArray[ 6]
	        ,	FloatArray[ 7]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                                   ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	FloatArray[ 8]
	        ,	FloatArray[ 9]
	        ,	FloatArray[10]
	        ,	FloatArray[11]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	sprintf(StringPart, "                                   ( %12.3f %12.3f %12.3f %12.3f )"
	        ,	0.0
	        ,	0.0
	        ,	0.0
	        ,	1.0);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetEstimatedExternalCartForcesAndTorques(FloatArray);
	sprintf(StringPart, "Estimated external Cart. forces/torques: %8.3f N %8.3f N %8.3f N %8.3f Nm %8.3f Nm %8.3f Nm"
	        ,	FloatArray[0]
	        ,	FloatArray[1]
	        ,	FloatArray[2]
	        ,	FloatArray[3]
	        ,	FloatArray[4]
	        ,	FloatArray[5]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, SINGLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, "Robot Status Data");
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, SINGLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	switch (this->GetFRIMode())
	{
	case FRI_STATE_OFF:
		sprintf(StringPart, "FRI mode: The KRC has NOT opened the FRI yet (OFF).");
		break;
	case FRI_STATE_MON:
		sprintf(StringPart, "FRI mode: MONITOR Mode");
		break;
	case FRI_STATE_CMD:
		sprintf(StringPart, "FRI mode: COMMAND Mode");
		break;
	default:
		sprintf(StringPart, "UDP communication quality: UNKNOWN");
		break;
	}
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	if (this->IsRobotArmPowerOn())
	{
		sprintf(StringPart, "Robot arm power: ON");
	}
	else
	{
		sprintf(StringPart, "Robot arm power: OFF");
	}
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	if (this->DoesAnyDriveSignalAnError())
	{
		sprintf(StringPart, "Drive fault: YES");
	}
	else
	{
		sprintf(StringPart, "Drive fault: NO");
	}
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	if (this->DoesAnyDriveSignalAWarning())
	{
		sprintf(StringPart, "Drive warning: YES");
	}
	else
	{
		sprintf(StringPart, "Drive warning: NO");
	}
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	this->GetDriveTemperatures(FloatArray);
	sprintf(StringPart, "Drive temperatures: %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f"
	        ,	FloatArray[0]
	        ,	FloatArray[1]
	        ,	FloatArray[2]
	        ,	FloatArray[3]
	        ,	FloatArray[4]
	        ,	FloatArray[5]
	        ,	FloatArray[6]);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	switch (this->GetCurrentControlScheme())
	{
	case FastResearchInterface::JOINT_POSITION_CONTROL:
		sprintf(StringPart, "Control strategy: Joint position control");
		break;
	case FastResearchInterface::CART_IMPEDANCE_CONTROL:
		sprintf(StringPart, "Control strategy: Cartesian impedance control");
		break;
	case FastResearchInterface::JOINT_IMPEDANCE_CONTROL:
		sprintf(StringPart, "Control strategy: Joint impedance control");
		break;
	default:
		sprintf(StringPart, "Control strategy: UNKNOWN!");
		break;
	}
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------
	sprintf(StringPart, DOUBLE_LINE);
	sprintf(&(this->RobotStateString[CharCounter]), "%s\n", StringPart);
	CharCounter	+=	strlen(StringPart) + 1;
	// ---------------------------------------------------------------------------------------------------

	return((const char*)this->RobotStateString);
}
