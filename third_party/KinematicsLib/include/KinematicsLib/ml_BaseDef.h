// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2001, 2003, 2005, 2006, 2009, 2012 - 2015
// KUKA Roboter GmbH, Germany. All Rights Reserved.

//! \file
//! Definition of basic constants and functions for the MathLib.
//!
//! This files defines some basic constants and functions used inside the MathLib.
//!
//! \stable{MathLib}

//<CL>
//*****************************************************************************
// Date        Programmer           Reviewer
//             Change note
//-----------------------------------------------------------------------------
// 09.08.2001  Hüttenhofer          N/A
//             Initial creation
// 30.11.2001  Weiss                N/A
//             better constants for URM trafo
// 26.09.2003  Sedlmayr
//             maintenance platform compatibility standard c++ library
// 18.03.2005  C.Spiess             N/A
//             Compiling with Workbench 2.2
// 11.08.2006  Klauser
//             change abs() to fabs(). fabs(double) is defined in all
//             environments, abs(double) is not
// 08.06.2009  Schreittmiller       Wiedemann
//             A2173: Funktionsgenerator fuer Spline, hier: TTS.
// 10.02.2011  Aurnhammer           Wiedemann
//             Zu kleines ML_EPS_ZERO in der Splinetestumgebung aufgetaucht
// 14.02.2011  Schreittmiller       Burkhart
//             Überarbeitung Doxygen-Dokumentare
// 06.02.2012  Aurnhammer           _PtpSplToBeReviewedByMP_
//             Versionierung verfeinert
// 17.07.2012  Eisensehr            Burkhart, Dürndorfer
//             Integration des GS-Maschinendatenladers in die KernelSystem-DLL.
// 30.08.2012  Aurnhammer           Tronnier
//             Umzug der MathLib nach BaseLib
// 31.10.2012  Tronnier             Aurnhammer
//             Nur Versionserhöhung durch:
//             CRDB53782: "Schnittstelle KernelSystemDll verändert"
// 20.10.2014  Aurnhammer           Wiedemann
//             Plattformmerge KRC/Sunrise (Team MP)
// 22.06.2015  Aurnhammer           Burkhart
//             Getrennte Versionierung von der MathLib im Plattformbranch im
//             Vergleich zum Rest der KernelDll
//*****************************************************************************
//</CL>

#ifndef _ML_BASEDEF_H
#define _ML_BASEDEF_H

//*****************************************************************************
// Header files

// System headers
#include <cmath>

// Public headers


//*****************************************************************************
// Version defines and checks

//! Versionnumber: Usage as date "xx.xx.xx" (e.g. 12.02.03 = 0x00120203).
//!
//! This version number is checked in between all MathLib interface headers
//! at compile time.
#define PLATFORM_MATHLIB_BASEDEF_VERSION_NUMBER 0x00121015

//*****************************************************************************
// Constant declarations and defines

const double  ML_PI           = 3.1415926535897931160; //!< Pi in double precision
const double  ML_2PI          = 2.0 * ML_PI;
const double  ML_RAD2DEG      = 180.0 / ML_PI;         //!< Converts radians to degrees
const double  ML_DEG2RAD      = ML_PI / 180.0;         //!< Converts degrees to radians


//! Converts degree angle value phi to radians
inline double ml_toRad(double phi)
{
   return phi * ML_DEG2RAD;
}

//! Converts degree angle value phi to radians, if doConvert is true
inline double ml_toRad(double phi, bool doConvert)
{
   return (doConvert ? ml_toRad(phi) : phi);
}

//! Converts radians angle value phi to degrees
inline double ml_toDeg(double phi)
{
   return phi * ML_RAD2DEG;
}

//! Converts radians angle value phi to degrees, if doConvert is true
inline double ml_toDeg(double phi, bool doConvert)
{
   return (doConvert ? ml_toDeg(phi) : phi);
}

//! Restrict radians angle value phi to the range [-ML_PI, ML_PI[
inline double ml_mod2Pi(double phi)
{
   // Truncate to ]-ML_2PI, ML_2PI[
   phi = fmod(phi, ML_2PI);

   // Truncate to [-ML_PI, ML_PI[
   if ( phi < -ML_PI ) return phi + ML_2PI;
   if ( phi >= ML_PI ) return phi - ML_2PI;

   return phi;
}

//! Restrict radians angle value phi to the range [referenceValue - ML_PI, referenceValue + ML_PI[
inline double ml_mod2Pi(double phi, double referenceValue)
{
   return ml_mod2Pi(phi-referenceValue) + referenceValue;
}

//! Returns the absolute difference radians angle of phi1 and phi2, modulo ML_2PI
inline static double ml_diffMod2Pi(double phi1, double phi2)
{
   return fabs( ml_mod2Pi(phi1-phi2) );
}


//! Minimal rotational vector length.
//!
//! Rotational vectors either have length zero or at least ML_MIN_ROTATION (unit is rad).
const double  ML_MIN_ROTATION = 0.0000020000888985016;

const double  ML_EPS_ZERO = 1E-11; //!< epsilon for zero comparision

const double  ML_EPS_ATAN2 = 1E-6; //!< epsilon  for zero comparisions in atan2 evaluations

//*****************************************************************************
// Type declarations

//*****************************************************************************
// Class declarations

//*****************************************************************************
// Function declarations

//! Inline atan2 function.
//!
//! Compute the arc tangent of y/x
//! \verbatim
//!                    [ atan(y/x)        for x > 0         ]
//!                    [ atan(y/x) + pi   for x < 0, y >= 0 ]
//!      atan2(y,x) =  [ atan(y/x) - pi   for x < 0, y <  0 ]
//!                    [ +pi/2            for x = 0, y >  0 ]
//!                    [ -pi/2            for x = 0, y <  0 ]
//!                    [ 0                for x = 0, y  = 0 ]
//! \endverbatim
//!
//! \param[in]  y  Numerator.
//! \param[in]  x  Denominator.
//!
//! \return Arc tangent of y/x.
//!
inline double ml_atan2 (double y, double x)
{
	if ((fabs(x) < ML_EPS_ATAN2) && (fabs(y) < ML_EPS_ATAN2))
	{
		return	0;
	}
	else
	{
		return 	atan2(y, x);
	}
}

//*****************************************************************************
// Typedef declarations

#endif  // _ML_BASEDEF_H
