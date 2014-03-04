//  ---------------------- Doxygen info ----------------------
//! \file InitializationFileEntry.cpp
//!
//! \brief
//! Implementation file for the class InitializationFileEntry
//!
//! \details
//! The class InitializationFileEntry provides a simple interface to read
//! entries from an initialization file and to provide in an
//! application. For details, please refer to the file InitializationFileEntry.h.
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
//!	\author Michael Borchard, Torsten Kroeger, tkr@stanford.edu
//!
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------


#include <InitializationFileEntry.h>
#include <string.h>
#include <stdio.h>


#define LINELEN		(NAMELEN + VALUELEN)


// ****************************************************************
// Constructor
//
InitializationFileEntry::InitializationFileEntry(const char *FileName)
{
	this->Next			=	NULL;
	this->Prev			=	NULL;
	this->CurrentEntry	=	NULL;

	memset(this->Name,					0x0, NAMELEN);
	memset(this->Value,					0x0, VALUELEN);
	memset(this->EntryName,				0x0, NAMELEN);
	memset(this->SectionName,			0x0, NAMELEN);
	memset(this->CurrentSectionName,	0x0, NAMELEN);

	// Now read the file if desired
	if (FileName != NULL)
    {
		this->ReadFile(FileName);
    }
}


// ****************************************************************
// Destructor
//
InitializationFileEntry::~InitializationFileEntry(void)
{
	// This destructor deletes the complete list regardless of the member it was called for.
	// For the currently unordered list, it is not necessary to delete predecessors
	// since they won't exist, but if the implementation supports an ordered list 
	// it might be useful to have this destructor.
	InitializationFileEntry *current	=	this;

	// delete the successor(s)
	if (current->Next != NULL)
    {
		current->Next->Prev = NULL;
		delete (current->Next);
		current->Next = NULL;
    }
	// delete the predecessors
	if (current->Prev != NULL)
    {
		delete (current->Prev);
		current->Prev = NULL;
    }
}


// ****************************************************************
// Add
//
bool InitializationFileEntry::Add(const char *Line)
{
	bool						Result		=	false;

	char						*Start		=	NULL
							,	*End		=	NULL;

	InitializationFileEntry		*NewEntry	=	NULL
							,	*EntryPtr	=	NULL;
	
	if (((Start = strchr ((char*)Line, '[')) != NULL) && ((End = strchr ((char*)Line, ']')) != NULL))
    {
		// (new) section name found
		strcpy (this->CurrentSectionName, Start + 1);
		this->CurrentSectionName[End - Start - 1] = '\0';
		Result = true;
    }
	else
	{	if ((Start = strchr ((char*)Line, '=')) != NULL)
    	{
			// entry = value found
			if ((this->EntryName[0] == '\0') && (this->SectionName[0] == '\0') && (this->Value[0] == '\0') && (this->CurrentSectionName[0] == '\0'))
			{	
				// use current (empty) entry
				NewEntry = this;
			}
			else
			{
				// reserve space for a new entry
				NewEntry = new (InitializationFileEntry);
			}
			if (NewEntry != NULL)
			{
				// free space is available
				strcpy (NewEntry->Value, Start+1);
				// cut line at '='
				*Start = '\0';
				// copy the entry name
				strcpy (NewEntry->EntryName, Line);
				// restore line contents
				*Start = '=';
				// set the section name
				strcpy (NewEntry->SectionName, this->CurrentSectionName);
				Result = true;
			}
    	}
		else
    	{
			// entry line found
			if ((this->EntryName[0] == '\0') && (this->SectionName[0] == '\0') && (this->Value[0] == '\0') && (this->CurrentSectionName[0] == '\0'))
			{
				// use current (empty) entry
				NewEntry = this;
			}
			else
			{
				// reserve space for a new entry
				NewEntry = new (InitializationFileEntry);
			}
			if (NewEntry != NULL)
			{
				// free space is available
				// copy the entry and the section name
				strcpy (NewEntry->SectionName, this->CurrentSectionName);
				strcpy (NewEntry->EntryName, Line);
				Result = true;
			}
    	}
	}
	
	// if line contained a new entry it will be appended to the list
	if ((NewEntry != NULL) && (NewEntry != this))
    {
		// Start with this
		EntryPtr = this;
		// and move the pointer to the end of the list
		while (EntryPtr->Next != NULL)
		{
			EntryPtr = EntryPtr->Next;
		}
		// append the new entry
		NewEntry->Prev = EntryPtr;
		EntryPtr->Next = NewEntry;
    }
	
	return(Result);
}


// ****************************************************************
// NextEntry
//
bool InitializationFileEntry::NextEntry(void)
{
	bool Result;
	
	if (this->CurrentEntry != NULL)
	{
		this->CurrentEntry = this->CurrentEntry->Next;
	}
	else
	{
		this->CurrentEntry = this;
	}
	Result = (this->CurrentEntry != NULL);
	return(Result);
}


// ****************************************************************
// FindNextsection
//
bool InitializationFileEntry::FindNextSection(void)
{
	InitializationFileEntry		*EntryPtr	=	this->CurrentEntry;  // Start with the current entry

	bool 						Found		=	false
							,	Result		=	false;
	
	char						NameOfSection[NAMELEN];

	memset(NameOfSection, 0x0, NAMELEN);

	if (EntryPtr != NULL)
    {
		// cycle through the list until a new section name is found or the end 
		// of the list is reached

		strcpy(NameOfSection, EntryPtr->SectionName);

		while ((EntryPtr != NULL) && !Found)
		{
			if (strcmp (NameOfSection, EntryPtr->SectionName))
			{
				// set Found if the section name does not match
				Found = true;
			}
			else
			{
				// else check the next entry
				EntryPtr = EntryPtr->Next;
			}
		}
		if (Found)
		{
			this->CurrentEntry	= EntryPtr;
			Result				= true;
		}
		else
			// if nothing could be found set the current pointer to the 
			// first entry -> the next call will again start cycling through
			// the list
		{
			this->CurrentEntry = this;
		}
    }
	return(Result);
}


// ****************************************************************
// GetSection
//
char *InitializationFileEntry::GetSection(void)
{
	if (this->CurrentEntry == NULL)
	{
		this->CurrentEntry = this;
	}
	return(this->CurrentEntry->SectionName);
}


// ****************************************************************
// GetAll
//
char *InitializationFileEntry::GetAll(void)
{
	if (this->CurrentEntry == NULL)
	{
		this->CurrentEntry = this;
	}
	return(this->CurrentEntry->EntryName);
}


// ****************************************************************
// GetName
//
char *InitializationFileEntry::GetName(void)
{
	int 	Index	=	0;

	if (this->CurrentEntry == NULL)
	{
		this->CurrentEntry = this;
	}

	// copy the name to the result array
	while (		(this->CurrentEntry->EntryName[Index]	!=	' '	)
			&&	(this->CurrentEntry->EntryName[Index]	!=	9	)
			&&	(this->CurrentEntry->EntryName[Index]	!=	0	))
    {
		this->Name[Index] = this->CurrentEntry->EntryName[Index];
		Index++;
    }
	this->Name[Index] = 0;
	
	return(Name);
}


// ****************************************************************
// GetValue
//
char *InitializationFileEntry::GetValue(void)
{
	if (this->CurrentEntry == NULL)
	{
		this->CurrentEntry = this;
	}
	return(this->CurrentEntry->Value);
}


// ****************************************************************
// FindEntry
//
bool InitializationFileEntry::FindEntry(const char *Name)
{
	InitializationFileEntry	*EntryPtr	=	this;  // start with this

	bool 			Found		=	false
				,	Result		=	false;
	
	// Cycle through the list until the name is found or the end of the list is reached
	while ((EntryPtr != NULL) && !Found)
    {
		if (!strcmp (Name, EntryPtr->EntryName))
		{
			// set Found if the name matches
			Found = true;
		}
		else
		{
			// else check the next entry
			EntryPtr = EntryPtr->Next;
		}
    }
	
	if (Found)
	// If the name could be found set the current entry to it
	// and set the result to true.
    {
		this->CurrentEntry = EntryPtr;
		Result = true;
    }
	
	return(Result);
}


// ****************************************************************
// ReadFile
//
void InitializationFileEntry::ReadFile(const char *FileName)
{
	bool 		Comment			=	false;

	char		Line[LINELEN];

	int			Input			=	0
			,	Counter			=	0;

	FILE 		*FileHandler			=	NULL;

	memset(Line, 0x0, LINELEN);

	if ((FileHandler = fopen (FileName, "rt")) != NULL)
	{
		while (!feof (FileHandler))
		{
			Counter = 0; // new line
			Comment = false;
			while (((Input = fgetc (FileHandler)) != '\n') && !feof(FileHandler) && !ferror(FileHandler))
			{
				if (Counter == 0 && Input == '#') // Comment lines start with a '#'.
				{
					Comment = true;
				}
				if (Input != '\r')
				{
					Line [Counter++] = (char)Input;
				}
			}
			if (!Comment && Counter > 0)
			{
				Line[Counter] = 0;
				this->Add(Line);
			}
		}
		fclose(FileHandler);
    } 
    else
    {
    	fprintf(stderr, "InitializationFileEntry: ERROR! File \"%s\" cannot be read !\n", FileName);
    }
}
