//  ---------------------- Doxygen info ----------------------
//! \file Doxygen.h
//!
//! \brief
//! Documentation file for Doxygen
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



/*! \mainpage Start Page

\section sec_Introduction Introduction

The Fast Research Interface Library intends to provide a simple user interface
to the <a href="http://www.kuka-labs.com" target="_blanc">KUKA</a>
Light-Weight Robot IV and hides all communication and set-up issues behind
interface. It is only an \b interface and it does \b not contain any control
functionalities. Without much installation efforts, access to different controller
interfaces of the KUKA system is provided:

- Joint position controller,
- Cartesian impedance controller, and
- Joint position controller.

The Fast Research Interface Library runs on a remote PC node with is connected
to the KRC (KUKA Robot Controller) via an Ethernet connection. In intervals of 1 to 100 milliseconds,
UDP packages are periodically sent from the KRC unit to the remote host. These packages contain
a complete set of robot control and status data (e.g., joint positions, joint torques, drive temperatures,
etc.; cf. FRI User Documentation).
The remote host (e.g., with <a href="http://www.qnx.com" target="_blanc" >QNX Neutrino RTOS</a>) has to instantaneously send a reply message after the reception of each package. A reply message
contains input data for the applied controllers (e.g., joint position set-points, joint stiffness set-points,
etc.). This way, users become able to set-up own control architectures and/or application-specific controllers
for the light-weight arm as it is often desired at research institutions.\n\n

\section sec_Download Download

The Fast Research Interface Library is available for\n
<ul>
<li>\ref page_QNX "QNX Neutrino",\n\n</li>
<li>\ref page_Linux "Linux",\n\n</li>
<li>\ref page_MACOS "MacOS", and\n\n</li>
<li>\ref page_Windows "Microsoft Windows".\n\n</li>
</ul>


\section sec_Overview Software Overview
The Fast Research Interface Library can be used in two different ways:
<ol>
<li>The class FastResearchInterface provides access to \em all functionalities offered by KUKA. This class has a simple structure,
but it consists of a rather huge number of methods.\n\n
<li>Based on the class FastResearchInterface, a set of classes has been designed in order to provide an easy-to-use and
easy-to-start-with API with all necessary functionalities while hiding all API-irrelevant parts of the library. The
base class for this controller interface is the class LWRBaseControllerInterface. Derived from this class, users can
select among interfaces for\n\n
<ul>
<li>joint position control (class LWRJointPositionController),</li>
<li>Cartesian impedance control (LWRCartImpedanceController), and</li>
<li>joint impedance control (class LWRJointImpedanceController).\n</li>
</ul>
 </ol>
The following section briefly describes how to get started with the robot and this library.\n\n


\section sec_QuickStartManual Quick Start Manual
This section gives a very brief overview about all required steps for quickly getting started with the Fast
Research Interface (see also FRI User Documentation).

<ul>
<li><b>Step 1: Install the Fast Research Interface Technology Package on the KRC unit.</b>\n\n
Please refer to
the manual of Installation/Uninstallation/Update of Tech Packages
SystemTech (KRS) from V5.x. \n\n\n</li>
<li><b>Step 2: Send your machine information to your KUKA representative in order to receive your FRI license file.</b>\n\n
 After installing the FRI software on the KRC unit, send the generated log file\n\n
 <tt> C:\\KRC\\Roboter\\Log\\lbrBoot.log</tt>\n\n
 from the KRC unit to your KUKA contact to receive your license file and respective
 KUKA instructions about how obtain an FRI license and how to install it. \n\n\n</li>
<li><b>Step 3: Configure the KRC node.</b>\n\n
Open the file <tt>C:\\Windows\\vxwin.ini</tt> on the KRC node and enter its IP address (e.g., <b>192.168.0.20</b>):\n\n
\code
[Boot]\n
Bootline=elPci(0,1)pc:vxworks h=192.0.1.2 b=192.0.1.1 e=192.168.0.20 u=target pw=vxworks
\endcode\n
The KUKA Fast Research Interface running on the KRC node requires the IP address and an open port of the remote host.
To specify both, open the file <tt> C:\\KRC\\Roboter\\INIT\\dlrrc.ini</tt> may contain\n\n
\code
[DLRRC]
FRIHOST=192.168.0.100
FRISOCK=49938,0
\endcode\n
After a reboot, you may use <tt>ping 192.168.0.20</tt> and <tt>ping 192.168.0.100</tt> on either node to check whether
the connection was successfully established. In order to achieve minimum latencies between both nodes, you may simply use
a crossed Ethernet cable.\n\n
Copy the \ref sec_KRLFiles to
\verbatim
C:\KRC\Roboter\KRC\R1\Program\FRIDemo
\endverbatim
on the KRC unit and restart it.\n\n\n</li>
<li><b>Step 4: Check out the Sample Applications.</b>\anchor StepFour\n\n
Depending on your needs, you may choose one of these simple sample applications in the folder
\verbatim src/FastResearchInterfaceTest \endverbatim
or 
\verbatim src/LWRGettingStartedExamples \endverbatim
to start with. The first folder only contains one sample application, <a href="../html/_fast_research_interface_test_8cpp_source.html">FastResearchInterfaceTest.cpp</a>
that makes use of the class FastResearchInterface and shows, how this generic API can be used. The
second folder contains a number of very simple sample applications, all which make use of one of these interface classes:\n\n
<ul>
<li><a href="../html/_l_w_r_joint_position_control_example_8cpp_source.html">LWRJointPositionControlExample.cpp</a> for an application using the <em>joint position controller</em></li>
<li><a href="../html/_l_w_r_cart_impedance_control_example_8cpp_source.html">LWRCartImpedanceControlExample.cpp</a> for an application using the <em>Cartesian impedance controller</em></li>
<li><a href="../html/_l_w_r_joint_impedance_control_example_8cpp_source.html">LWRJointImpedanceControlExample.cpp</a> for an application using the <em>joint impedance controller</em></li>
<li><a href="../html/_l_w_r_logging_example_8cpp_source.html">LWRLoggingExample.cpp</a> for an application using the interface for logging low-level control data\n\n</li>
</ul>
The respective user APIs for the controller interfaces are defined by the classes\n\n
<ul>
<li>LWRJointPositionController for the joint position controller,</li>
<li>LWRCartImpedanceController for the Cartesian impedance controller, and</li>
<li>LWRJointImpedanceController for the joint impedance controller,\n\n</li>
</ul>
 which are all derived from the class LWRBaseControllerInterface. This class contains an object of the class FastResearchInterface,
 which constitutes the actual interface and provides the full set functionalities. The class LWRBaseControllerInterface and its
 three derivatives only act as a wrapper and hide the actual (rather voluminous) library with all its relevant communication issues. Only
 necessary and required functionalities are provided for each controller.

\em Optionally, a real-time capable \c printf() function (LWRBaseControllerInterface::printf()) and a low-level real-time
data logger may be useful (class DataLogging).\n\n\n</li>
<li><b>Step 5: Compile the Fast Research Interface Library for the remote host.</b>\n\n
Instructions for the following operating systems can be found here:\n\n
<ul>
<li>\ref page_QNX "QNX Neutrino",\n</li>
<li>\ref page_Linux "Linux",\n</li>
<li>\ref page_MACOS "MacOS", and\n</li>
<li>\ref page_Windows "Microsoft Windows".\n</li>
</ul>
\n\n\n</li>
<li><b>Step 6: Execute your first application on the remote host.</b>\n\n
\em Remark: Make sure, all network settings are correct (\ref sec_NWSetup "here is an example").
The provided KRL file (\ref page_KRLFile1 "FRIControl.src") and the initialization file
(\ref page_InitFile "980039-FRI-Driver.init") are prepared for a sample time of
2 milliseconds.
\n\n\n</li>

</ul>

*/
// -----------------------------------------------------------------
/*!
\page page_KRLFiles KRL Source Files
\section sec_KRLFiles KRL Source Files
The KRL source code of the KRC unit can be found here:\n
<ul>
<li> \ref page_KRLFile1</li>
<li> \ref page_KRLFile2\n\n</li>
</ul>
The remote host is permanently communicating with the KRC unit via a UDP
connection. In order to start/stop the Fast Research Interface on the side
of the KRC unit, both hosts exchange data via the variables
\code
- $FRI_TO_BOOL[1...16],
- $FRI_TO_INT[1...16],
- $FRI_TO_REA[1...16],
- $FRI_FRM_BOOL[1...16],
- $FRI_FRM_INT[1...16], and
- $FRI_FRM_REA[1...16].
\endcode
The first three arrays are sent from the KRC unit \em to the remote host; the latter three data arrays
are received by the KRC \em from the Remote host. The remote host may call the methods
\code
- FastResearchInterface::GetKRLBoolValues(), FastResearchInterface::GetKRLBoolValue()
- FastResearchInterface::GetKRLIntValues(), FastResearchInterface::GetKRLIntValue()
- FastResearchInterface::GetKRLFloatValues(), FastResearchInterface::GetKRLFloatValue()
- FastResearchInterface::SetKRLBoolValues(), FastResearchInterface::SetKRLBoolValue()
- FastResearchInterface::SetKRLIntValues(), FastResearchInterface::SetKRLIntValue()
- FastResearchInterface::SetKRLFloatValues(), FastResearchInterface::SetKRLFloatValue()
\endcode
to read or write these variables.
*/




// -----------------------------------------------------------------
/*!
\page page_NWSetup Network Setup (Example)
\section sec_NWSetup Network Setup (Example)
This section briefly describes the network set-up.
\subsection KUKA KRC Unit
<ul>
<li><b>Network adapter:</b> 3COM 3C905CTX Ethernet adapter\n\n
<ul>
<li><b>IP address:</b> 192.168.0.20\n\n</li>
<li><b>MAC address:</b> 00:0A:5E:26:26:43\n\n</li>
<li><b>Note:</b> The network adapter is \em only used by the VxWorks system; in the MS Windows environment, only a dummy adapter can be found.\n\n</li>
</ul></li>
</ul>



\subsection Remote Host (QNX OS)
<ul>
<li><b>Network adapter:</b> Realtek RTL8111/8168B Ethernet adapter (1)\n\n
<ul>
<li><b>IP address:</b> 172.24.69.14 (connection to the Stanford network and the Internet)\n\n</li>
<li><b>MAC address:</b> E0:CB:4E:B0:84:E0\n\n</li>
</ul></li>
<li><b>Network adapter:</b> Intel E1G42ET Ethernet adapter (2 + 3)\n\n
<ul>
<li><b>IP address:</b> 192.168.0.100 (private connection to the KRC unit through a crossed Ethernet cable)\n\n</li>
<li><b>MAC address:</b> 00:1B:21:68:AC:20\n\n</li>
</ul>\n\n\n</li>
</ul>\n\n\n

To remotely access the QNX host, the following service are available (cf. <tt>/etc/inetd.conf</tt>):\n\n
<ul>
<li>FTP\n\n</li>
<li>phrelay/QConn (Phindows)\n\n</li>
<li>telnet\n\n</li>
<li>rlogin\n\n</li>
<li>rsh\n\n</li>
</ul>

*/
// -----------------------------------------------------------------
/*!
\page page_KRLFile1 KRL File: FRIControl.src
\section sec_KRLFile1 KRL File: FRIControl.src
<b>Note line 116 (friOpen(2)), which defines the control sample time (here: 2 milliseconds).</b>
\verbinclude FRIControl.src
\sa \ref sec_KRLFile2
*/
// -----------------------------------------------------------------
/*!
\page page_KRLFile2 KRL File: FRIControl.dat
\section sec_KRLFile2 KRL File: FRIControl.dat
\verbinclude FRIControl.dat
\sa \ref sec_KRLFile1
*/
// -----------------------------------------------------------------



/*!
\page page_InitFile The Initialization File for the Fast Research Interface Library
\section sec_InitFile The Initialization File for the Fast Research Interface Library
The constructor of the class FastResearchInterface requires the specification of an initialization file. This initialization file
contains basic parameters to set-up the features provided by the Fast Research Interface Library.
A sample file may be seen here:

\verbinclude 980039-FRI-Driver.init

\b Note line 15 (\c CycleTime), which defines the control sample time (here: 2 milliseconds).\n\n\n
<b>Description of all parameters of the initialization file</b>\n\n
<ul>
<li>Section \b RobotName \n\n\n
<ul>
<li> \b Name: A string that contains the robot name (to enable the simultaneous usage of multiple robot arms)\n\n\n
</ul>
<li>Section \b Priorities \n\n\n
<ul>
<li> \b KRCCommunicationThread: The priority of the thread communicating with the KRC unit via a UDP socket connection
(see also FastResearchInterface::KRCCommunicationThreadMain()). It is recommended to use a high (or even the highest) priority
 for this thread as it is of major importance to immediately respond to received messages. This may minimize the latency
 and the jitter of the UDP communication channel between the KRC unit and the remote host. \n\n
<li> \b TimerThread: As the timing quality of the VxWorks platform is not necessarily sufficient, users
can (optionally) use this local timer for their control algorithms. The timer is managed by this thread, which provides the
timer service via a simple condition variable (see also FastResearchInterface::TimerThreadMain() and
FastResearchInterface::WaitForTimerTick()). \n\n
<li> \b MainThread: This priority is assigned to the thread that calls the constructor of the class FastResearchInterface. \n\n
<li> \b OutputConsoleThread: In order to enable error (and debug) information output (e.g., to \c stdout), the class Console
is used. An object of this class contains an output thread (Console::ConsoleThreadMain()) that is supposed to run at a low
priority. \n\n\n
</ul>
<li>Section \b ControlValues \n\n\n
<ul>
<li> \b CycleTime: The communication cycle time in seconds. This value has to be the same as used by the KRL function call
 of <tt>friStart()</tt> (cf. \ref sec_KRLFile1). The KRC unit sends messages to the remote host in isochronous time intervals whose
 width is specified by this value. \n\n\n
</ul>
<li>Section \b Logging \n\n\n
<ul>
<li> \b NumberOfLoggingFileEntries: The class FastResearchInterface offers the possibility of using a low-level real-time
data logger implemented in the class DataLogging. The value \em NumberOfLoggingFileEntries specifies the maximum number of
logged entries (cf. DataLogging::MaximumNumberOfEntries). \n\n
<li> \b LoggingPath specifies the output directory used by the logging object of the class DataLogging (cf.
DataLogging::OutputPath). \n\n
<li> \b LoggingFileName specifies the output file name and its extension used by the logging object of the class DataLogging (cf.
DataLogging::OutputFileName). \n\n
</ul>
</ul>


\sa \ref InitializationFileEntry
*/

// -----------------------------------------------------------------
/*!
\page page_VersionHistory Version History
\section sec_VersionHistory1 Version 0.1 (August 2010)
- Initial version for QNX only
\section sec_VersionHistory2 Version 0.2 (September 2010)
- Porting to MS Windows
\section sec_VersionHistory3 Version 1.0 (November 2011)
- Porting to Linux
- Porting to MacOS
- Update to <em>KUKA Systems Software 5.6 lr</em>
- Bug fixes in KRL code
*/
// -----------------------------------------------------------------

/*!
\page page_QNX Download and Installation Instructions: QNX Neutrino

\n
<b>Download URL for Linux or Solaris hosted IDEs</b>: <a href="http://cs.stanford.edu/people/tkr/fri/download/fril.tar.gz"><b>http://cs.stanford.edu/people/tkr/fri/download/fril.zip</b></a>\n\n
<b>Download URL for Windows hosted IDEs</b>: <a href="http://cs.stanford.edu/people/tkr/fri/download/fril.tar.gz"><b>http://cs.stanford.edu/people/tkr/fri/download/fril.tar.gz</b></a>\n\n


Download the file and copy it to a directory of yours (e.g.,
<tt>C:\\Users\\MyUsername\\FRI</tt> under Windows or <tt>/home/MyUsername/FRI</tt>
under Linux). After unzipping the compressed file
with a tool (e.g.,
<a href="http://www.7-zip.org" title="http://www.7-zip.org" target="_blanc" ><b>7-Zip</b></a>
or
<a href="http://www.gzip.org" title="http://www.gzip.org" target="_blanc" ><b>gzip</b></a>),
you need to replace the three files <c>friComm.h</c>, <c>friudp.h</c>, and
<c>friudp.cpp</c> with the original files you received from KUKA (cf. 
<a href="files.html">List of Files</a>). Afterwards you create a new
project in you QNX Momentics IDE and specify the absolute path of the 
directory
\code
FRILibrary
\endcode
as the location of your project. In the directory \c QNX, you can find 
corresponding makefiles. In the file <c>Makefile.global</c>, you need
adapt the variable <c>ROOT_DIR</c> to the absolute path of the directory
\c FRILibrary. Now, you can rebuild your the entire project in your
QNX Momentics development environment to check
whether all files compile correctly on your system.
If so, you may continue with \ref StepFour "Step 4 of the Quick Start Manual".
The class FastResearchInterface represents
the user API.

Please note that you will need to adapt the path of the initialization
file (e.g., \ref page_InitFile "980039-FRI-Driver.init") before you can
establish a connection form the remote host to the KRC control unit.

\n\n\n


*/

// -----------------------------------------------------------------

/*!
\page page_Linux Download and Installation Instructions: Linux

\n
<b>Download URL</b>: <a href="http://cs.stanford.edu/people/tkr/fri/download/fril.tar.gz"><b>http://cs.stanford.edu/people/tkr/fri/download/fril.tar.gz</b></a>\n\n

Download the file and copy it to a directory of yours (e.g.,
<tt>/home/MyUsername/FRI</tt>). After unzipping the compressed file
with
\code
tar -xf fril.tar.gz
\endcode
you need to replace the three files <c>friComm.h</c>, <c>friudp.h</c>, and
<c>friudp.cpp</c> with the original files you received from KUKA (cf. 
<a href="files.html">List of Files</a>). Afterwards, you change to the
directory <tt>FRILibrary/Linux</tt> and enter
\code
make clean all
\endcode
to check whether all files compile correctly on your system. 
If so, you may continue with \ref StepFour "Step 4 of the Quick Start Manual".
The class FastResearchInterface represents
the user API.

Please note that you will need to adapt the path of the initialization
file (e.g., \ref page_InitFile "980039-FRI-Driver.init") before you can
establish a connection form the remote host to the KRC control unit.


\n

\note
To enable building for <b>64-bit architectures</b>, please uncomment
the marked lines in the file <b>ExternalTargets.global</b>.

\n\n\n

*/

// -----------------------------------------------------------------

/*!
\page page_MACOS Download and Installation Instructions: MacOS

\n
<b>Download URL</b>: <a href="http://cs.stanford.edu/people/tkr/fri/download/fril.tar.gz"><b>http://cs.stanford.edu/people/tkr/fri/download/fril.tar.gz</b></a>\n\n

Download the file and copy it to a directory of yours (e.g.,
<tt>/home/MyUsername/FRI</tt>). After unzipping the compressed file
with
\code
tar -xf fril.tar.gz
\endcode
you need to replace the three files <c>friComm.h</c>, <c>friudp.h</c>, and
<c>friudp.cpp</c> with the original files you received from KUKA (cf. 
<a href="files.html">List of Files</a>). Afterwards, you change to the
directory <tt>FRILibrary/Linux</tt> and enter
\code
make clean all
\endcode
to check whether all files compile correctly on your system. 
If so, you may continue with \ref StepFour "Step 4 of the Quick Start Manual".
The class FastResearchInterface represents
the user API.

Please note that you will need to adapt the path of the initialization
file (e.g., \ref page_InitFile "980039-FRI-Driver.init") before you can
establish a connection form the remote host to the KRC control unit.

\n\n\n

*/

// -----------------------------------------------------------------

/*!
\page page_Windows Download and Installation Instructions: Microsoft Windows (Visual Studio 2008)

\n
<b>Download URL</b>: <a href="http://cs.stanford.edu/people/tkr/fri/download/fril.zip"><b>http://cs.stanford.edu/people/tkr/fri/download/fril.zip</b></a>\n\n

Download the file and copy it to a directory of yours (e.g.,
<tt>C:\\Users\\MyUsername\\FRI</tt>). After unzipping the compressed file
with a tool (e.g.,
<a href="http://www.7-zip.org" title="http://www.7-zip.org" target="_blanc" ><b>7-Zip</b></a>),
you need to replace the three files <c>friComm.h</c>, <c>friudp.h</c>, and
<c>friudp.cpp</c> with the original files you received from KUKA (cf. 
<a href="files.html">List of Files</a>). Afterwards you open the Visual
Studio solution file
\code
FRILibrary\Windows\FastResearchLibrary.sln
\endcode
with your
<a href="http://www.microsoft.com/visualstudio" target="_blanc" title="" ><b>Microsoft Visual Studio</b></a>
development environment. You may rebuild the entire solution to check
whether all files compile correctly on your system.
If so, you may continue with \ref StepFour "Step 4 of the Quick Start Manual".
The class FastResearchInterface represents
the user API.

Please note that you will need to adapt the path of the initialization
file (e.g., \ref page_InitFile "980039-FRI-Driver.init") before you can
establish a connection form the remote host to the KRC control unit.

\n\n\n

*/

// -----------------------------------------------------------------
