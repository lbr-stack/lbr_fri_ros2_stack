// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2001 - 2015
// KUKA Roboter GmbH, Germany. All Rights Reserved.

//! \file
//! Public interface to MathLib frame calculus class.
//!
//! Public header file publishing all frame calculus methods of the
//! MathLib using a frame class with static memory allocation.
//!
//! \stable{MathLib}

//<CL>
//*****************************************************************************
// Date        Programmer           Reviewer
//             Change note
//-----------------------------------------------------------------------------
// 09.08.2001  Hüttenhofer          N/A
//             Initial creation
// 22.10.2001  Weiss                N/A
//             output operator
// 13.11.2001  Weiss                N/A
//             constructor from array
// 26.09.2003  Sedlmayr
//             maintenance platform compatibility standard c++ library
// 24.11.2005  Mueller-Sommer
//             Achieve compatibility for WIN32 compiler in project HFPENV
// 10.02.2006  Mueller-Sommer       Hagenauer/Fremuth-Paeger
//             A2011/A1938 Kombiroboter/Alternativer GWA
// 06.04.2010  Dürndorfer           Eisensehr, Schreittmiller
//             Funktion zum Interpolieren zweier Frames eingeführt
// 09.04.2010  Aurnhammer           Burkhart
//             Anbindung an die KernelSystem-DLL
// 09.02.2011  Burkhart             Aurnhammer/Wiedemann/Deller
//             Schreittmiller       Lebsack/Sturm
//             KERNEL-DLL: Schnittstellenredesign; Multiinstanzfähigkeit Spline;
//                         DLL-Integration in die KRC; Absolutgenauigkeit
// 14.02.2011  Schreittmiller       Burkhart
//             Überarbeitung Doxygen-Dokumentare
// 06.02.2012  Aurnhammer           _PtpSplToBeReviewedByMP_
//             Versionierung verfeinert
// 17.07.2012  Eisensehr            Burkhart, Dürndorfer
//             Integration des GS-Maschinendatenladers in die KernelSystem-DLL.
// 24.07.2012  Hüttenhofer          Aurnhammer
//             Dynamisch allokierbare Vektor- und Matrixklasse
// 30.07.2012  Sonner               Aurnhammer
//             Neue Konsturktoren und Routinen für Verwendung von DoubleArrays
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

#ifndef _ML_FRAME_H
#define _ML_FRAME_H

//*****************************************************************************
// Header files

// System headers
#include <iostream>

// Public headers
#include "dllBuildDefines.h" // _DLL_
#include "ml_CVector.h"
#include "ml_COrientation.h"

//*****************************************************************************
// Version defines and checks

//! Versionnumber: Usage as date "xx.xx.xx" (e.g. 12.02.03 = 0x00120203).
//!
//! This version number is checked in between all MathLib interface headers
//! at compile time.
#define PLATFORM_MATHLIB_FRAME_VERSION_NUMBER 0x00121015

#if PLATFORM_MATHLIB_FRAME_VERSION_NUMBER != PLATFORM_MATHLIB_VECTOR_VERSION_NUMBER
   #error ml_CFrame.h does not fit to ml_CVector.h
#endif

#if PLATFORM_MATHLIB_FRAME_VERSION_NUMBER != PLATFORM_MATHLIB_ORI_VERSION_NUMBER
   #error ml_CFrame.h does not fit to ml_COrientation.h
#endif

//*****************************************************************************
// Constant declarations and defines

//*****************************************************************************
// Type declarations

//*****************************************************************************
// Class declarations

//! Class representing a frame by means of 3x3 orthonormal matrices and a
//! 3-dimensional vector.
//!
//! This class provides a general purpose frame representation by means of
//! a 3x3 orthonormal matrix for the representation of orientation and
//! a 3-dimensional vector for the representation of translation.
//! Hence, the first column is the image of the x-axis, the second of the
//! y-axis, the third of the z-axis and the fourth column the position of the
//! origin.
//! \verbatim
//!                       [   |      |      |     Pos(x) ]
//!         T(Ori,Pos) =  [ Ori(x) Ori(y) Ori(z)  Pos(y) ]
//!                       [   |      |      |     Pos(z) ]
//!                       [   0      0      0       1    ]
//! \endverbatim
//!
class ml_CFrame
{
   public:
      //! \name Public Members.
      //! @{

      ml_CVector3    Pos;  //!< Translational part of the frame
      ml_COriMatrix  Ori;  //!< Orientational part of the frame
      //! @}

      //! \name Constructors and destructors.
      //! @{

      //! Constructor creating a frame with translational part zero vector and
      //! orientational part identity matrix.
      _DLL_ ml_CFrame();

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CFrame(const ml_CFrame& Initial);

      //! Constructor creating a frame and initializing it with orientation defined by
      //! a 3x3 orthonormal matrix and the origin as a 3-dimensional vector.
      //!
      //! \param[in] OriMatrix   Orientation matrix.
      //! \param[in] Position    Origin position.
      //!
      _DLL_ ml_CFrame(const ml_COriMatrix& OriMatrix, const ml_CVector3& Position);

      //! Constructor creating a frame and initializing it with orientation defined by
      //! the values inside a two dimensional double array and the origin as a
      //! 3-dimensional double array.
      //!
      //! \param[in] InitOri  Orientation matrix.
      //! \param[in] InitPos  Origin position.
      //!
      _DLL_ ml_CFrame(const double InitOri[3][3], const double InitPos[3]);

      //! Constructor creating a frame and initializing it with the frame
      //! corrsponding to (original) Denavit-Hartenberg parameters.
      //!
      //! \verbatim
      //! F = Rot_z(Theta) * Trans_z(D) * Rot_x(Alpha) * Trans_x(A)
      //! \endverbatim

      //! \param[in] Theta     Rotation angle about z.
      //! \param[in] D         Position shift in z.
      //! \param[in] Alpha     Rotation angle about x.
      //! \param[in] A         Position shift in x.
      //!
      _DLL_ ml_CFrame(const double& Theta, const double& D,
                      const double& A, const double& Alpha);

      //! Constructor creating a frame and initializing it with orientation defined by
      //! intrinsic ZYX_Euler angles and the origin as a 3-dimensional vector.
      //!
      //! \param[in] ABC       Orientation matrix.
      //! \param[in] Position  Origin position.
      //!
      _DLL_ ml_CFrame(const ml_CEulerZYX& ABC, const ml_CVector3& Position);

      //! Constructor creating a frame and initializing it with orientation and
      //! translation defined by the values inside a two dimensional 3x4 double array.
      //!
      //! \param[in] Frame  Orientation and position array.
      //!
      _DLL_ ml_CFrame(const double Frame[3][4]);
      //! @}

      //! \name Assignment and arithmetic operators.
      //! @{

      //! Frame assignment.
      //!
      //! \param[in] Rhs  Right-hand side in assignment.
      //!
      //! \return Assigned frame.
      //!
      _DLL_ ml_CFrame& operator=(const ml_CFrame& Rhs);

      //! Frame multiplication.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this) * Rhs.
      //!
      _DLL_ ml_CFrame operator*(const ml_CFrame& Rhs) const;

      //! Frame multiplication.
      //!
      //! (*this) = (*this) * Rhs.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      _DLL_ void operator*=(const ml_CFrame& Rhs);

      //! Frame multiplication with a vector.
      //!
      //! \param[in] Vector  Right-hand side in multiplication.
      //!
      //! \return (*this) * Rhs.
      //!
      _DLL_ ml_CVector3 operator*(const ml_CVector3& Vector) const;

      //! Invert frame.
      //!
      //! \return (*this)^(-1).
      //!
      _DLL_ ml_CFrame operator~() const;

      //! Invert matrix.
      //!
      //! (*this) = (*this)^(-1).
      //!
      _DLL_ void invertThis();
      //! @}

      //! \name Comparison operators.
      //! @{

      //! Compares two frames for equality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) == Rhs.
      //! \retval false  (*this) != Rhs.
      //!
      _DLL_ bool operator==(const ml_CFrame& Rhs) const;

      //! Compares two frames for inequality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) != Rhs.
      //! \retval false  (*this) == Rhs.
      //!
      _DLL_ bool operator!=(const ml_CFrame& Rhs) const;
      //! @}

      //! \name Quick arithmetic operations.
      //! @{

      //! Invert the actual frame and right-hand multiply the result with another frame.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this)^(-1) * Rhs.
      //!
      _DLL_ ml_CFrame  invMul(const ml_CFrame& Rhs) const;

      //! Invert the right-hand frame and multiply the result with actual frame.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (Rhs)^(-1) * (*this).
      //!
      _DLL_ ml_CFrame  mulInv(const ml_CFrame& Rhs) const;

      //! Invert the actual frame and right-hand multiply the result with another
      //! inverted frame.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this)^(-1) * (Rhs)^(-1).
      //!
      _DLL_ ml_CFrame  invMulInv(const ml_CFrame& Rhs) const;
      //! @}

      //! Conversion initializing a frame with orientation and translation defined by the
      //! values inside a two dimensional 3x4 double array.
      //!
      //! \param[in] Frame  Orientation and position array.
      //!
      _DLL_ void fromDoubleArray(const double Frame[3][4]);

      //! Conversion initializing a two dimensional 3x4 double array with values defined by the
      //! actual frame.
      //!
      //! \param[out] Frame  Orientation and position array.
      //!
      _DLL_ void toDoubleArray(double Frame[3][4]);

      //! \name Friend output operators.
      //! @{

      //! Output stream operator for frames.
      //!
      //! \param[in] o      Output stream.
      //! \param[in] Frame  Output frame.
      //!
      //! \return Output stream.
      //!
      friend _DLL_ std::ostream& operator<<(std::ostream& o, const ml_CFrame& Frame);
      //! @}
};

//*****************************************************************************
// Function declarations

//! \name Output operators.
//! @{

//! Output stream operator for frames.
//!
//! \param[in] o      Output stream.
//! \param[in] Frame  Output frame.
//!
//! \return Output stream.
//!
_DLL_ std::ostream& operator<<(std::ostream& o, const ml_CFrame& Frame);
//! @}

//! \brief   Interpolates in between two frames.
//!
//! \param[in]  Frame1  First Frame.
//! \param[in]  Frame2  Second Frame.
//! \param[in]  Fac     Number in [0..1] to set the relationship in between the
//!                     input frames. If 0 is entered Frame1 is returned, if
//!                     1 is entered Frame2 is returned.
//!
//! \return     Interpolated frame.
//!
_DLL_ ml_CFrame interpolateFrames(const ml_CFrame& Frame1, const ml_CFrame& Frame2, double Fac);

//*****************************************************************************
// Typedef declarations

#endif  // _ML_FRAME_H
