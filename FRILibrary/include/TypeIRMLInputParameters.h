//  ---------------------- Doxygen info ----------------------
//! \file TypeIRMLInputParameters.h
//!
//! \brief
//! Header file for the input parameters of the Reflexxes Motion Library
//! (Type I)
//!
//! \details
//! \n
//! \n
//! <b>GNU Lesser Public License</b>
//! \n
//! \n
//! This file is part of the Reflexxes Motion Library (Type I).
//! \n\n
//! The Reflexxes Motion Library (Type I) is free software: you can redistribute
//! it and/or modify it under the terms of the GNU General Public License
//! as published by the Free Software Foundation, either version 3 of the
//! License, or (at your option) any later version.
//! \n\n
//! The Reflexxes Motion Library (Type I) is distributed in the hope that it
//! will be useful, but WITHOUT ANY WARRANTY; without even the implied 
//! warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
//! the GNU General Public License for more details.
//! \n\n
//! You should have received a copy of the GNU General Public License
//! along with the Reflexxes Motion Library (Type I). If not, see 
//! http://www.gnu.org/licenses.
//! \n
//! \n
//! \n
//! Reflexxes GmbH\n
//! Sandknoell 7\n
//! D-24805 Hamdorf\n
//! GERMANY\n
//! \n
//! http://www.reflexxes.com\n
//!
//! \date November 2011
//! 
//! \version 1.1
//!
//!	\author Reflexxes GmbH, <info@reflexxes.com> \n
//!	
//!
//!
//  ----------------------------------------------------------
//   For a convenient reading of this file's source code,
//   please use a tab width of four characters.
//  ----------------------------------------------------------



#ifndef __TypeIRMLInputParameters__
#define __TypeIRMLInputParameters__

#include <TypeIRMLVector.h>

namespace TypeIRMLMath
{


//  ---------------------- Doxygen info ----------------------
//! \class TypeIRMLInputParameters
//!
//! \brief
//! This class describes the input parameters of the Reflexxes Motion Library
//! (Type I)
//! 
//! \details
//! It is part of the namespace TypeIRMLMath.
//! 
//! \sa TypeIRML
//! \sa TypeIRMLOutputParameters
//  ----------------------------------------------------------
class TypeIRMLInputParameters
{

public:


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLInputParameters(const unsigned int &NumberOfDOFs)
//!
//! \brief Constructor of the class TypeIRMLInputParameters
//!
//! \details
//! It allocates memory for the specified number of degrees of freedom.
//!
//! \warning
//! A call of this method is \em not real-time capable!!!
//!
//! \param NumberOfDOFs
//! Number of degrees of freedom
//!
//! \sa TypeIRML
//  ----------------------------------------------------------

	TypeIRMLInputParameters(const unsigned int &NumberOfDOFs)
	{
		this->CurrentPosition			=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->CurrentVelocity			=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->MaxVelocity				=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->MaxAcceleration			=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->TargetPosition			=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->TargetVelocity			=	new TypeIRMLDoubleVector(NumberOfDOFs);
		this->SelectionVector			=	new TypeIRMLBoolVector  (NumberOfDOFs);

		this->CurrentPosition->Set(0.0)		;
		this->CurrentVelocity->Set(0.0)		;
		this->MaxVelocity->Set(0.0)			;		
		this->MaxAcceleration->Set(0.0)		;
		this->TargetPosition->Set(0.0)		;
		this->TargetVelocity->Set(0.0)		;
		this->SelectionVector->Set(false)	;
	}


//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLInputParameters(const TypeIRMLInputParameters & IP)
//!
//! \brief
//! Copy constructor of the class TypeIRMLInputParameters
//!
//! \details
//! Allocates memory for the specified number of degrees of freedom.
//!
//! \warning
//! A call of this method is \em not real-time capable!!!
//!
//! \param IP
//! Original object reference
//  ----------------------------------------------------------
	TypeIRMLInputParameters(const TypeIRMLInputParameters & IP)
	{
		this->CurrentPosition			=	new TypeIRMLDoubleVector(IP.CurrentPosition->VectorDimension	);
		this->CurrentVelocity			=	new TypeIRMLDoubleVector(IP.CurrentVelocity->VectorDimension	);
		this->MaxVelocity				=	new TypeIRMLDoubleVector(IP.MaxVelocity->VectorDimension		);
		this->MaxAcceleration			=	new TypeIRMLDoubleVector(IP.MaxAcceleration->VectorDimension	);
		this->TargetPosition			=	new TypeIRMLDoubleVector(IP.TargetPosition->VectorDimension		);
		this->TargetVelocity			=	new TypeIRMLDoubleVector(IP.TargetPosition->VectorDimension		);
		this->SelectionVector			=	new TypeIRMLBoolVector  (IP.SelectionVector->VectorDimension	);

		this->CurrentPosition	=	IP.CurrentPosition	;
		this->CurrentVelocity	=	IP.CurrentVelocity	;
		this->MaxVelocity		=	IP.MaxVelocity		;	
		this->MaxAcceleration	=	IP.MaxAcceleration	;
		this->TargetPosition	=	IP.TargetPosition	;
		this->TargetVelocity	=	IP.TargetVelocity	;
		this->SelectionVector	=	IP.SelectionVector	;

	}

//  ---------------------- Doxygen info ----------------------
//! \fn TypeIRMLInputParameters &operator = (const TypeIRMLInputParameters &IP)
//!
//! \brief
//! Copy operator
//!
//! \param IP
//! Set of input parameters
//  ----------------------------------------------------------
	TypeIRMLInputParameters &operator = (const TypeIRMLInputParameters &IP)
	{
		this->CurrentPosition	=	IP.CurrentPosition	;
		this->CurrentVelocity	=	IP.CurrentVelocity	;
		this->MaxVelocity		=	IP.MaxVelocity		;	
		this->MaxAcceleration	=	IP.MaxAcceleration	;
		this->TargetPosition	=	IP.TargetPosition	;
		this->TargetVelocity	=	IP.TargetVelocity	;
		this->SelectionVector	=	IP.SelectionVector	;

		return (*this);
	}


//  ---------------------- Doxygen info ----------------------
//! \fn ~TypeIRMLInputParameters(void)
//!
//! \brief
//! Destructor of class TypeIRMLInputParameters, frees heap memory
//  ----------------------------------------------------------
	~TypeIRMLInputParameters(void)
	{
		delete this->CurrentPosition	;
		delete this->CurrentVelocity	;
		delete this->MaxVelocity		;	
		delete this->MaxAcceleration	;
		delete this->TargetPosition		;
		delete this->TargetVelocity		;
		delete this->SelectionVector	;

		this->CurrentPosition		=	NULL;
		this->CurrentVelocity		=	NULL;
		this->MaxVelocity			=	NULL;
		this->MaxAcceleration		=	NULL;
		this->TargetPosition		=	NULL;
		this->TargetVelocity		=	NULL;
		this->SelectionVector		=	NULL;
	}

	TypeIRMLDoubleVector*	CurrentPosition	;
	TypeIRMLDoubleVector*	CurrentVelocity	;
	TypeIRMLDoubleVector*	MaxAcceleration	;
	TypeIRMLDoubleVector*	MaxVelocity		;
	TypeIRMLDoubleVector*	TargetPosition	;
	TypeIRMLDoubleVector*	TargetVelocity	;
	TypeIRMLBoolVector*		SelectionVector	;
};

}	// namespace

#endif
