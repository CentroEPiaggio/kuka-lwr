//  ---------------------- Doxygen info ----------------------
//! \file OSAbstraction.h
//!
//! \brief
//! Header file for simple OS-specific functions for abstraction
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


#ifndef __OSAbstraction__
#define __OSAbstraction__

#include <stdlib.h>



//  ---------------------- Doxygen info ----------------------
//! \def OS_FOLDER_SEPARATOR
//!
//! \brief
//! Slash for non-Microsoft operating system or backslash for Microsoft operating systems
//  ----------------------------------------------------------
#ifdef WIN32
#define OS_FOLDER_SEPARATOR	("\\")
#else
#define OS_FOLDER_SEPARATOR	("/")
#endif


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned char WaitForKBCharacter(bool *Abort = NULL)
//!
//! \brief Waits for one single keyboard stroke
//!
//! \details
//! The function returns after the stroke or if \c Abort becomes set.
//! If \c Abort is set at the call of the function, the function only checks once, if a key has
//! been pressed.
//!
//! \param Abort
//! Pointer to a boolean values, which lets the function terminate
//!
//! \return
//!  - Value of the pressed key
//!  - 255 if no key was pressed
//!
//! \sa CheckForKBCharacter()
//  ----------------------------------------------------------
unsigned char 	WaitForKBCharacter(bool *Abort = NULL);


//  ---------------------- Doxygen info ----------------------
//! \fn unsigned char char CheckForKBCharacter(void)
//!
//! \brief Checks for one single keyboard stroke
//!
//! \details
//! The function returns immediately.
//!
//! \return
//!  - Value of the pressed key
//!  - 255 if no key was pressed
//!
//! \sa WaitForKBCharacter()
//  ----------------------------------------------------------
unsigned char CheckForKBCharacter(void);


//  ---------------------- Doxygen info ----------------------
//! \fn float GetSystemTimeInSeconds(const bool &Reset = false)
//!
//! \brief Returns a time value in seconds
//!
//! \details
//! Operating system independent function to return the value of the system time
//! in seconds.
//!
//! \param Reset
//! If this flag is set, the value of \em this system time will be
//! set to zero.
//!
//! \return
//! The value of the system time in seconds.
//  ----------------------------------------------------------
float GetSystemTimeInSeconds(const bool &Reset = false);



// ##################################################################################
// ##################################################################################
// ##################################################################################
// OS-specific

// ############  W I N D O W S  #####################################################
#ifdef WIN32
#include <WindowsAbstraction.h>
#endif

// ################  Q N X  #########################################################
#ifdef _NTO_
#include <QNXAbstraction.h>
#endif

// ##############  L I N U X  #######################################################
#ifdef __LINUX__
#include <LinuxAbstraction.h>
#endif

// ##############  M A C O S  #######################################################
#ifdef __MACOS__
#include <MACOSAbstraction.h>
#endif


// ##################################################################################
// ##################################################################################
// ##################################################################################

#endif
