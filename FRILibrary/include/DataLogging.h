//  ---------------------- Doxygen info ----------------------
//! \file DataLogging.h
//!
//! \brief
//! Header file for the class DataLogging
//!
//! \details
//! The class DataLogging provides a basic oscilloscope/logging
//! for all data being passed through the KUKA Fast Research
//! Interface.
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


#ifndef __DataLogging__
#define __DataLogging__


#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <friComm.h>


//  ---------------------- Doxygen info ----------------------
//! \class DataLogging
//!
//! \brief
//! Provides logging/oscilloscope/data recording functionality
//! for the KUKA Fast Research Interface
//!
//! \details
//! The class has been designed for the Fast Research Interface of the KUKA
//! KUKA Light-Weight Robot IV. It offers the possibility of writing all
//! relevant control data, which is exchanged between the KRC unit and the
//! remote host, into a file that can be read by
//! <a href="http://www.mathworks.com">Matlab</a>,
//! <a href="http://office.microsoft.com/en-us/excel">Microsoft Excel</a>, or
//! other software programs.
//!
//! The constructor DataLogging::DataLogging() allocates a
//! (rather huge) amount of heap memory, to which all the data can be written in each
//! communication (i.e., control) control cycle under real-time conditions. The user can
//! specify the maximum number of entries that will logged. Before the scope functionality
//! can be used, the method DataLogging::PrepareLogging() has to be called. If running with a
//! period of one millisecond, and a maximum number of 60,000, up to one minute can be
//! recorded by calling the method DataLogging::AddEntry() after each data exchange
//! between the remote host and the KRC unit. DataLogging::AddEntry() is real-time
//! capable. At a later time instant (e.g., when the robot was shut down or needs
//! not to be controlled by the remote host anymore), the method DataLogging::WriteToFile()
//! may be called. This method does \b not work in real-time and writes all entries that
//! were stored in the heap memory into a file. This method can also be used in multi-robot
//! robot environments as each robot arm may be addressed by a name (cf. DataLogging::MachineName).
//!
//! The actual output file name is unambiguously generated as follows:
//! <tt>Path/Date-Time-OptionalUserDefinedString-RobotName-FileName</tt> (e.g.,
//! <tt>/home/lwrcontrol/output/100729-104452-Test-980039-LWRScope.dat</tt>).
//!
//!Depending on the chosen controller, different control values are written to the log file:\n\n
//!
//! <b>Joint position controller</b>
//!  - Cycle number
//!  - Time stamp of the KRC node
//!  - Time stamp of the remote host
//!  - Measured joint torque vector: <c> ActFJ[1,...,7]</c>
//!  - Joint position set-point vector (specified by the application running on the remote host): <c>UDesJ[1,...,7]</c>
//!  - Measured joint position vector: <c>ActJ[1,...,7]</c>
//!  - Commanded joint position vector by the KRC unit: <c>KDesJ[1,...,7]</c>
//! \n\n
//!
//! <b>Cartesian impedance controller</b>
//!  - Cycle number
//!  - Time stamp of the KRC node
//!  - Time stamp of the remote host
//!  - Desired stiffness vector (specified by the application running on the remote host): <c>DesK[x,y,z,a,b,c]</c>
//!  - Desired damping vector (specified by the application running on the remote host): <c>DesD[x,y,z,a,b,c]</c>
//!  - Desired force/torque offset vector (specified by the application running on the remote host): <c>UDesF[x,y,z,a,b,c]</c>
//! \n\n
//!
//! <b>Joint impedance controller</b>
//!  - Cycle number
//!  - Time stamp of the KRC node
//!  - Time stamp of the remote host
//!  - Measured joint torque vector: <c> ActFJ[1,...,7]</c>
//!  - Joint position set-point vector (specified by the application running on the remote host): <c>UDesJ[1,...,7]</c>
//!  - Measured joint position vector: <c>ActJ[1,...,7]</c>
//!  - Commanded joint position vector by the KRC unit: <c>KDesJ[1,...,7]</c>
//!  - Desired joint stiffness vector (specified by the application running on the remote host): <c>DesKJ[1,...,7]</c>
//!  - Desired joint damping vector (specified by the application running on the remote host): <c>DesDJ[1,...,7]</c>
//!  - Desired joint torque offset vector (specified by the application running on the remote host): <c>UDesFJ[1,...,7]</c>
//!  - Commanded joint position offset vector by the KRC unit: <c>KOffP[1,...,7]</c>
//! \n\n
//!
//! <b>Joint torque controller</b>
//!  - Cycle number
//!  - Time stamp of the KRC node
//!  - Time stamp of the remote host
//!  - Measured joint torque vector: <c> ActFJ[1,...,7]</c>
//!  - Measured joint position vector: <c>ActJ[1,...,7]</c>
//!  - Desired joint torque vector (specified by the application running on the remote host): <c>UDesFJ[1,...,7]</c>
//!
//! This real-time logging/scope functionality on this low level may be an important
//! tool for debugging and control prototyping purposes as they commonly are done by
//! research and development institutions.
//  ----------------------------------------------------------
class DataLogging
{
public:

//  ---------------------- Doxygen info ----------------------
//! \fn DataLogging(const char *RobotName, const char *LoggingPath, const char *LoggingFileName, const unsigned int &MaxNumberOfEntries)
//!
//! \brief
//! Constructor
//!
//! \details
//! The constructor initializes all class attributes and allocates the required amount of
//! heap memory.
//!
//! \param  RobotName
//! A pointer to an array of \c char containing the name of the robot (e.g., its serial number).
//! This value will be used as part of the name of the file, in which the logged data is stored.
//!
//! \param  LoggingPath
//! A pointer to an array of \c char containing the directory, in which the logging file
//! will be created (e.g. <tt>"/home/lwrcontrol/output"</tt>).
//!
//! \param  LoggingFileName
//! A pointer to an array of \c char containing the final part of the file name
//! as well as the file extension (e.g. <tt>"-LWR-Scope.dat"</tt>).
//!
//! \param  MaxNumberOfEntries
//! The maximum number entries that are stored in the heap memory. One entry is required
//! for each message received by the remote host from the KRC unit. To log control data
//! for an amount of time of one minute, the values has to be <tt>60000</tt> if the
//! the controllers run at rate of 1 KHz.
//!
//! \attention
//! The implementation of the constructor does \b not feature real-time behavior.
//!
//! \sa Class DataLogging
//  ----------------------------------------------------------
	DataLogging(	const char			*RobotName
	           	,	const char			*LoggingPath
	           	,	const char			*LoggingFileName
	           	,	const unsigned int	&MaxNumberOfEntries);


//  ---------------------- Doxygen info ----------------------
//! \fn ~DataLogging(void)
//!
//! \brief
//! Destructor
//!
//! \details
//! The destructor free the memory that was allocated by the constructor DataLogging::DataLogging().
//! If DataLogging::PrepareLogging() was was called, and if DataLogging::WriteToFile() has not been
//! called yet, the destructor calls DataLogging::WriteToFile() first in order to prevent the user from
//! data loss.
//!
//! \attention
//! The implementation of the destructor does \b not feature real-time behavior.
//  ----------------------------------------------------------
	~DataLogging(void);


//  ---------------------- Doxygen info ----------------------
//! \fn int PrepareLogging(const unsigned int &ControlScheme, const char *FileIdentifier = NULL)
//!
//! \brief
//! Creates the output file and initializes all required class attributes
//!
//! \details
//! The file name for the logging file is generated, the file is opened (i.e., created)
//! using the file handler DataLogging::OutputFileHandler,
//! and all required class attributes are initializes, such that in the following, the
//! DataLogging::AddEntry() can be called after each data exchange between the remote host
//! and the KRC unit in order to perform the actual logging function.
//!
//! \param ControlScheme
//! This value has to equal one of the following set:
//!
//!  - FastResearchInterface::JOINT_POSITION_CONTROL
//!  - FastResearchInterface::CART_IMPEDANCE_CONTROL
//!  - FastResearchInterface::JOINT_IMPEDANCE_CONTROL
//!
//! Depending on this value, the logged control data is chosen (see also DataLogging).
//!
//! \param FileIdentifier
//! A pointer to an array of \c char values containing a string to identify the
//! written log file. This parameter is \em optional.
//!
//! \return
//!  - \c EBADF if the file could not be created.
//!  - \c EINVAL if the parameter \c ControlScheme is invalid.
//!  - \c EOK otherwise.
//!
//! \attention
//! The implementation of this method does \b not feature real-time behavior.
//!
//! \sa Class DataLogging
//! \sa FastResearchInterface::LWRControlModes
//  ----------------------------------------------------------
	int PrepareLogging(		const unsigned int	&ControlScheme
	                   	,	const char			*FileIdentifier = NULL);


//  ---------------------- Doxygen info ----------------------
//! \fn void AddEntry(const tFriMsrData  &ReceivedFRIData, const tFriCmdData &SentFRIData)
//!
//! \brief
//! Writes control data data to the heap memory
//!
//! \details
//! Depending on the chosen controller (specified by DataLogging::CurrentControlScheme)
//! respective data is extracted from \c ReceivedFRIData and \c SentFRIData. This data
//! is copied to the heap memory DataLogging::LoggingMemory.
//!
//! \param ReceivedFRIData
//! The complete data package received by the remote host from the KRC unit.
//!
//! \param SentFRIData
//! The complete data package sent by the remote host to the KRC unit.
//!
//! \remark
//! If the required memory capacity specified by DataLogging::MaximumNumberOfEntries
//! in the constructor DataLogging::DataLogging() is exceeded, the heap memory is used
//! as a <em>ring buffer</em> in order to keep the most recent data stored.
//!
//! \note
//! The implementation of this method is real-time capable.
//!
//! \sa Class DataLogging
//  ----------------------------------------------------------
	void AddEntry(		const tFriMsrData		&ReceivedFRIData
	             	,	const tFriCmdData		&SentFRIData		);



//  ---------------------- Doxygen info ----------------------
//! \fn int WriteToFile(void)
//!
//! \brief
//! Writes the logged data from the heap memory to the output file and closes the file
//!
//! \details
//! The data stored in the heap memory at DataLogging::LoggingMemory by
//! the method DataLogging::AddEntry() is written to
//! the file DataLogging::CompleteOutputFileString created and opened by
//! DataLogging::PrepareLogging() and pointed to by the file handler
//! DataLogging::OutputFileHandler.
//!
//! Only necessary data is written to the output file, that is, if the number of entries in
//! DataLogging::LoggingMemory, is less than DataLogging::MaximumNumberOfEntries, only
//! DataLogging::OutputCounter entries are written to the output file.
//!
//! \return
//!  - \c EOF if the file could not be closed.
//!  - \c EINVAL If the parameter \c ControlScheme is invalid.
//!  - \c EOK otherwise.
//!
//! \attention
//! The implementation of this method does \b not feature real-time behavior.
//!
//! \sa Class DataLogging
//  ----------------------------------------------------------
	int WriteToFile(void);

protected:


//  ---------------------- Doxygen info ----------------------
//! \enum ObjectState
//!
//! \brief
//! Describes the state of the object (i.e., whether a file is currently opened and in use)
//  ----------------------------------------------------------
	enum ObjectState
	{
		PrepareLoggingCalled	=	1,
		WriteToFileCalled		=	2
	};


//  ---------------------- Doxygen info ----------------------
//! \var CurrentObjectState
//!
//! \brief
//! Stores the current state of the object
//  ----------------------------------------------------------
	ObjectState			CurrentObjectState;


//  ---------------------- Doxygen info ----------------------
//! \var MachineName
//!
//! \brief
//! A pointer to an array of \c char values containing the name of the machine
//  ----------------------------------------------------------
	char				*MachineName;


//  ---------------------- Doxygen info ----------------------
//! \var OutputPath
//!
//! \brief
//! A pointer to an array of \c char values containing the output directory
//  ----------------------------------------------------------
	char				*OutputPath;


//  ---------------------- Doxygen info ----------------------
//! \var OutputFileName
//!
//! \brief
//! A pointer to an array of \c char values containing the output file name and its extension
//  ----------------------------------------------------------
	char				*OutputFileName;


//  ---------------------- Doxygen info ----------------------
//! \var CompleteOutputFileString
//!
//! \brief
//! A pointer to an array of \c char values containing the complete current output file (incl. date and time) name and its directory
//  ----------------------------------------------------------
	char				*CompleteOutputFileString;


//  ---------------------- Doxygen info ----------------------
//! \var MaximumNumberOfEntries
//!
//! \brief
//! Specifies the maximum number of entries to be stored in DataLogging::LoggingMemory
//  ----------------------------------------------------------
	unsigned int		MaximumNumberOfEntries;


//  ---------------------- Doxygen info ----------------------
//! \var OutputCounter
//!
//! \brief
//! Specifies the total number of calls of DataLogging::AddEntry()
//!
//! \remark
//! If this value is greater than the value of DataLogging::MaximumNumberOfEntries,
//! DataLogging::LoggingMemory is used as a ring buffer.
//!
//! \sa DataLogging::AddEntry()
//  ----------------------------------------------------------
	unsigned int		OutputCounter;


//  ---------------------- Doxygen info ----------------------
//! \var CurrentControlScheme
//!
//! \brief
//! Specifies the current control scheme
//!
//! \sa Class DataLogging
//! \sa FastResearchInterface::LWRControlModes
//  ----------------------------------------------------------
	unsigned int		CurrentControlScheme;


//  ---------------------- Doxygen info ----------------------
//! \var LoggingMemory
//!
//! \brief
//! A pointer to an array of pointers to arrays of \c float values containing the logged data
//  ----------------------------------------------------------
	float				**LoggingMemory;


//  ---------------------- Doxygen info ----------------------
//! \var FILE* OutputFileHandler
//!
//! \brief
//! A pointer to the file handler of the output file described by DataLogging::CompleteOutputFileString
//  ----------------------------------------------------------
	FILE*				OutputFileHandler;

};	// class DataLogging

#endif
