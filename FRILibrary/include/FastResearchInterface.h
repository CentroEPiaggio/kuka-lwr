#ifndef __FastResearchInterface__
#define __FastResearchInterface__

#include <Console.h>
#include <DataLogging.h>
#include <friComm.h>
#include <pthread.h>

class FastResearchInterface
{
public:

	FastResearchInterface(const char *InitFileName);
	~FastResearchInterface(void);
	enum LWRControlModes
	{
		JOINT_POSITION_CONTROL	=	10,
		CART_IMPEDANCE_CONTROL	=	20,
		JOINT_IMPEDANCE_CONTROL	=	30,
	};
	int StartRobot(		const unsigned int &ControlMode
	               	,	const float &TimeOutValueInSeconds = 120.0);
	int StopRobot(void);
	void GetMeasuredJointPositions(float *MeasuredJointPositions);
	void GetCommandedJointPositions(float *CommandedJointPositions);
	void GetCommandedJointPositionOffsets(float *CommandedJointPositionOffsets);
	void GetMeasuredJointTorques(float *MeasuredJointTorques);
	void GetEstimatedExternalJointTorques(float *EstimatedExternalJointTorques);
	void GetMeasuredCartPose(float *MeasuredCartPose);
	void GetCommandedCartPose(float *CommandedCartPose);
	void GetCommandedCartPoseOffsets(float *CommandedCartPoseOffsets);
	void GetEstimatedExternalCartForcesAndTorques(float *EstimatedExternalCartForcesAndTorques);
	void SetCommandedJointPositions(const float *CommandedJointPositions);
	void SetCommandedJointTorques(const float *CommandedJointTorques);
	void SetCommandedJointStiffness(const float *CommandedJointStiffness);
	void SetCommandedJointDamping(const float *CommandedJointDamping);
	void SetCommandedCartPose(const float *CommandedCartPose);
	void SetCommandedCartForcesAndTorques(const float *CartForcesAndTorques);
	void SetCommandedCartStiffness(const float *CommandedCartStiffness);
	void SetCommandedCartDamping(const float *CommandedCartDamping);
	unsigned int GetFRIMode(void);
	unsigned int GetCurrentControlScheme(void);
	bool IsRobotArmPowerOn(void);
	bool DoesAnyDriveSignalAnError(void);
	bool DoesAnyDriveSignalAWarning(void);
	void GetDriveTemperatures(float *Temperatures);
	void GetCurrentJacobianMatrix(float **JacobianMatrix);
	void GetCurrentMassMatrix(float **MassMatrix);
	void GetCurrentGravityVector(float *GravityVector);
	float GetFRICycleTime(void);
	int GetCommunicationTimingQuality(void);
	float GetUDPAnswerRate(void);
	float GetUDPLatencyInSeconds(void);
	float GetUDPJitterInSeconds(void);
	float GetUDPPackageLossRate(void);
	unsigned int GetNumberOfMissedUDPPackages(void);
	unsigned int GetValueOfKRCSequenceCounter(void);
	void GetKRLBoolValues(bool *KRLBoolValues);
	void GetKRLIntValues(int *KRLIntValues);
	void GetKRLFloatValues(float *KRLFloatValues);
	bool GetKRLBoolValue(const unsigned int &Index);
	int GetKRLIntValue(const unsigned int &Index);
	float GetKRLFloatValue(const unsigned int &Index);
	void SetKRLBoolValues(const bool *KRLBoolValues);
	void SetKRLIntValues(const int *KRLIntValues);
	void SetKRLFloatValues(const float *KRLFloatValues);
	void SetKRLBoolValue(		const unsigned int	&Index
						  ,	const bool			&Value	);

	void SetKRLIntValue(		const unsigned int	&Index
	                    	,	const int			&Value	);
	void SetKRLFloatValue(		const unsigned int	&Index
	                      	,	const float			&Value	);
	int WaitForKRCTick(const unsigned int &TimeoutValueInMicroSeconds = 0);
	int WaitForTimerTick(const unsigned int &TimeoutValueInMicroSeconds = 0);
	bool IsMachineOK(void);
    const char* GetCompleteRobotStateAndInformation(void);
	int printf(const char* Format, ...);
	int PrepareLogging(const char *FileIdentifier = NULL);
	int StartLogging(void);
	int StopLogging(void);
	int WriteLoggingDataFile(void);
protected:
	int ReadInitFile(const char *InitFileName);
	int SetControlScheme(const unsigned int &ControlScheme);
	static void *TimerThreadMain(void *ThreadArgument);
	static void *KRCCommunicationThreadMain(void *ThreadArgument);
	enum CalledLoggingMethod
	{
		PrepareLoggingCalled		=	1,
		StartLoggingCalled			=	2,
		StopLoggingCalled			=	3,
		WriteLoggingDataFileCalled	=	4
	};
	bool					TerminateTimerThread;
	bool					TimerFlag;
	bool					TerminateKRCCommunicationThread;
	bool					NewDataFromKRCReceived;
	bool					LoggingIsActive;
	bool					ThreadCreated;
	CalledLoggingMethod		LoggingState;
	char					*RobotName;
	char					*LoggingPath;
	char					*LoggingFileName;
	char					*RobotStateString;
	//
	char					*IP; // this line was included by Manuel Bonilla, this variable is to enable the library to manage more than one robot.
	int						PORT;// this is the same but for the port configuration

	//
	unsigned int			CurrentControlScheme;
	unsigned int			NumberOfLoggingFileEntries;
	unsigned int			PriorityKRCCommunicationThread;
	unsigned int			PriorityTimerThread;
	unsigned int			PriorityMainThread;
	unsigned int			PriorityOutputConsoleThread;
	double					CycleTime;
	pthread_mutex_t			MutexForControlData;
	pthread_mutex_t			MutexForCondVarForTimer;
	pthread_mutex_t			MutexForLogging;
	pthread_mutex_t			MutexForThreadCreation;
	pthread_cond_t			CondVarForTimer;
	pthread_cond_t			CondVarForDataReceptionFromKRC;
	pthread_cond_t			CondVarForThreadCreation;	
	pthread_t				KRCCommunicationThread;
	pthread_t				TimerThread;
	pthread_t				MainThread;
	Console					*OutputConsole;
	DataLogging				*DataLogger;
	tFriMsrData 			ReadData;
	tFriCmdData				CommandData;
};	
#endif