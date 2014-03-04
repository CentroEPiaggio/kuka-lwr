//  ---------------------- Doxygen info ----------------------
//! \file WindowsAbstraction.cpp
//!
//! \brief
//! Implementation file containing OS-specific functions (Windows)
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

#include <OSAbstraction.h>



#include <stdio.h>
#include <conio.h>
#include <time.h>
#include <Windows.h>
#include <stdlib.h>


ULONGLONG						StoredSystemTimeInTicks;


static bool						GetSystemTimeInSecondsCalledFirstTime	=	true;
    


// ****************************************************************
// WaitForKBCharacter()
//
unsigned char WaitForKBCharacter(bool *Abort)
{
	if (Abort == NULL)
	{
		while ( _kbhit() == 0 )
		{
			Sleep(10);
		}
	}
	else
	{
		while ( ( _kbhit() == 0 ) && (!(*Abort)) )
		{
			Sleep(10);
		}
		if (*Abort)
		{
			return(0);	
		}
	}
	
	return(_getche());
}



unsigned char CheckForKBCharacter(void)
{

	if ( _kbhit() == 0 )
	{
		return(0);	
	}
	else
	{
		return(_getche());
	}
}


float GetSystemTimeInSeconds(const bool &Reset)
{
	ULONGLONG				CurrentLocalMachineTime;

	if ( (GetSystemTimeInSecondsCalledFirstTime) || (Reset) )
	{
		StoredSystemTimeInTicks					=	GetTickCount64();
		GetSystemTimeInSecondsCalledFirstTime	=	false;
	}
	
	CurrentLocalMachineTime	=	GetTickCount64();

	return(0.001 * (float)(CurrentLocalMachineTime - StoredSystemTimeInTicks));
}
