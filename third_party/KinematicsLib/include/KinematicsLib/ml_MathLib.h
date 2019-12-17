// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2000 -- 2015
// KUKA Roboter GmbH, Germany. All Rights Reserved

//! \file
//! Public interface to all MathLib classes and methods.
//!
//! Public header file, which includes all public header files of the module
//! MathLib. So each file, that wants to use the MathLib module needs to
//! include only this header file.
//!
//! \stable{MathLib}

//<CL>
//*****************************************************************************
// Date        Programmer           Reviewer
//             Change note
//-----------------------------------------------------------------------------
// 09.08.2001  Hüttenhofer          N/A
//             Initial creation
// 04.09.2001  Hüttenhofer          N/A
//             File moved from \GeneralServices to \include\exp
// 09.04.2010  Aurnhammer           Burkhart
//             Anbindung an die KernelSystem-DLL
// 10.07.2007  Lebsack              none
//             remover the "Source" directory from MathLib
// 09.02.2011  Burkhart             Aurnhammer/Wiedemann/Deller
//             Schreittmiller       Lebsack/Sturm
//             KERNEL-DLL: Schnittstellenredesign; Multiinstanzfähigkeit Spline;
//                         DLL-Integration in die KRC; Absolutgenauigkeit
// 06.02.2012  Aurnhammer           _PtpSplToBeReviewedByMP_
//             Versionierung verfeinert
// 17.07.2012  Eisensehr            Burkhart, Dürndorfer
//             Integration des GS-Maschinendatenladers in die KernelSystem-DLL.
// 24.07.2012  Hüttenhofer          Aurnhammer
//             Dynamisch allokierbare Vektor- und Matrixklasse
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

#ifndef _ML_MATHLIB_H
#define _ML_MATHLIB_H

//*****************************************************************************
// Header files

// System headers

// Public headers
#include "ml_BaseDef.h"
#include "ml_CVector.h"
#include "ml_CMatrix.h"
#include "ml_COrientation.h"
#include "ml_CFrame.h"

//*****************************************************************************
// Version defines and checks

//! Versionnumber: Usage as date "xx.xx.xx" (e.g. 12.02.03 = 0x00120203).
//!
//! This version number is checked in between all MathLib interface headers
//! at compile time.
#define PLATFORM_MATHLIB_VERSION_NUMBER 0x00121015

#if PLATFORM_MATHLIB_VERSION_NUMBER != PLATFORM_MATHLIB_BASEDEF_VERSION_NUMBER
   #error ml_MathLib.h does not fit to ml_BaseDef.h
#endif

#if PLATFORM_MATHLIB_VERSION_NUMBER != PLATFORM_MATHLIB_VECTOR_VERSION_NUMBER
   #error ml_MathLib.h does not fit to ml_CVector.h
#endif

#if PLATFORM_MATHLIB_VERSION_NUMBER != PLATFORM_MATHLIB_MATRIX_VERSION_NUMBER
   #error ml_MathLib.h does not fit to ml_CMatrix.h
#endif

#if PLATFORM_MATHLIB_VERSION_NUMBER != PLATFORM_MATHLIB_ORI_VERSION_NUMBER
   #error ml_MathLib.h does not fit to ml_COrientation.h
#endif

#if PLATFORM_MATHLIB_VERSION_NUMBER != PLATFORM_MATHLIB_FRAME_VERSION_NUMBER
   #error ml_MathLib.h does not fit to ml_CFrame.h
#endif

//*****************************************************************************
// Constant declarations and defines

//*****************************************************************************
// Type declarations

//*****************************************************************************
// Class declarations

//*****************************************************************************
// Function declarations

//*****************************************************************************
// Typedef declarations

#endif  // _ML_MATHLIB_H
