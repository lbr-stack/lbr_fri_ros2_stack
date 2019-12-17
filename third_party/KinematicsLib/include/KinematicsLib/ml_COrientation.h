// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2001 - 2015
// KUKA Roboter GmbH, Germany. All Rights Reserved.

//! \file
//! Public interface to MathLib 3-dimensional orientation representation
//! classes and methods.
//!
//! Public header file publishing all orientation representation methods of the
//! MathLib using intrinsic ZYX-Euler angles, Quaternions, 3x3 orthonormal matrices
//! and elementary rotations.
//!
//! \stable{MathLib}

//<CL>
//*****************************************************************************
// Date        Programmer           Reviewer
//             Change note
//-----------------------------------------------------------------------------
// 09.08.2001  Hüttenhofer          N/A
//             Initial creation
// 25.09.2001  Weiß                 N/A
//             Output operator, component access
// 14.11.2001  Weiß                 N/A
//             component access ml_CEulerZYX
// 26.09.2003  Sedlmayr
//             maintenance platform compatibility standard c++ library
// 29.07.2004  Wiedemann            N/A
//             New constructor for orientation matrix
// 23.02.2005  Mueller-Sommer
//             std::ostream-method declaration necessary for pedantic compiler
//             (VisualStudio 6)
// 30.08.2006  Burkhart             Duerndorfer/Wiedemann/Klauser
//             added operators and missing conversions between quaternions and
//             other orientation representations (e.g. rotational vectors)
// 04.10.2006  Burkhart             Wiedemann/Klauser
//             Code inspections results implemented.
//             Do no longer offer any "+" operator for quaternions, as
//             => in this context we are exclusively dealing with UNIT quaternions
//                (as only NORMALISED (unit) quaternions represent orientations)
//             => UNIT quaternions are a (NON COMMUTATIVE) group according to "*"
//             => but UNIT quaternions are NO GROUP AT ALL ACCORDING TO "+" !!!!
//                (requires the entire quaternions space) As the "+" operator
//                usually delivers no UNIT quaternions, its result does NOT
//                represent an orientation
//             removed  ml_COriMatrix (const ml_CVector3& RpyValues)
//             Now the default constructor delivers (1, 0, 0, 0) to avoid
//             floating point exceptions due to scaleToUnitLength
// 17.09.2007  Schreittmiller       Burkhart/Dürndorfer
//             A2215: zeitbasierte Bewegungen (Spline)
// 25.07.2008  Schreittmiller       Burkhart/Wiedemann
//             A2145: Kreiswinkel (CA) fuer SCIRC
// 28.10.2008  Schreittmiller       Aurnhammer/Burkhart/Wiedemann
//             Nacharbeiten CA, Testumgebung, Funktionsinverter
// 08.06.2009  Schreittmiller       Wiedemann
//             A2173: Funktionsgenerator fuer Spline, hier: TTS.
// 28.04.2009  Dürndorfer           Eisensehr/Hagenauer/Schreittmiller
//             RegDB2173: Spline-Funktionsgenerator
// 23.11.2009  Schreittmiller       Dürndorfer
//             C42452: Wenn TTS sprint, erscheint Meldung M_SPLFG_TTS statt
//             Sollgeschwindigkeit oder ähnlichem.
// 03.12.2009  Mueller-Sommer
//             Neu: isValid() Gueltigkeitspruefung der OriMatrix
// 05.01.2010  Schreittmiller       ---
//             calcRotAngle() in calcDeltaAngle() umbenannt.
// 12.01.2010  Aurnhammer
//             Anpassungen für FrameVisu und VC++ 6.0
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
// 29.11.2011  Schreittmiller       Burkhart
//             ml_COriMatrix::setZero entfernt
// 29.11.2011  Burkhart             Schreittmiller
//             Neue Methode isOrthonormal() für Matrizen eingeführt
// 06.02.2012  Aurnhammer           _PtpSplToBeReviewedByMP_
//             Versionierung verfeinert
// 17.07.2012  Eisensehr            Burkhart, Dürndorfer
//             Integration des GS-Maschinendatenladers in die KernelSystem-DLL.
// 24.07.2012  Hüttenhofer          Aurnhammer
//             Dynamisch allokierbare Vektor- und Matrixklasse
// 30.08.2012  Sonner
//             Entfernen des _DLL_ Defines vor ml_CElemRot - sonst keine Nutzung des Templates möglich!
// 30.08.2012  Aurnhammer           Tronnier
//             Umzug der MathLib nach BaseLib
// 31.10.2012  Tronnier             Aurnhammer
//             Nur Versionserhöhung durch:
//             CRDB53782: "Schnittstelle KernelSystemDll verändert"
// 20.10.2014  Aurnhammer           Wiedemann
//             Plattformmerge KRC/Sunrise (Team MP)
// 31.03.2015  Ramold               Schreitmiller
//             Latex im Doxygen Kommentar entfernt.
// 22.06.2015  Aurnhammer           Burkhart
//             Getrennte Versionierung von der MathLib im Plattformbranch im
//             Vergleich zum Rest der KernelDll
//*****************************************************************************
//</CL>

#ifndef _ML_CORIENTATION_H
#define _ML_CORIENTATION_H

//*****************************************************************************
// Header files

// System headers
#include <iostream>

// Public headers
#include "dllBuildDefines.h"     // _DLL_
#include "ml_BaseDef.h"          // ML_PI, ML_RAD2DEG
#include "ml_CVector.h"          // ml_CVector3


//*****************************************************************************
// Version defines and checks

//! Versionnumber: Usage as date "xx.xx.xx" (e.g. 12.02.03 = 0x00120203).
//!
//! This version number is checked in between all MathLib interface headers
//! at compile time.
#define PLATFORM_MATHLIB_ORI_VERSION_NUMBER 0x00121015

#if PLATFORM_MATHLIB_ORI_VERSION_NUMBER != PLATFORM_MATHLIB_BASEDEF_VERSION_NUMBER
   #error ml_COrientation.h does not fit to ml_BaseDef.h
#endif

#if PLATFORM_MATHLIB_ORI_VERSION_NUMBER != PLATFORM_MATHLIB_VECTOR_VERSION_NUMBER
   #error ml_COrientation.h does not fit to ml_CVector.h
#endif


//*****************************************************************************
// Constant declarations and defines

//*****************************************************************************
// Type declarations

//! Intrinsic Euler angles representation of 3-dimensional orientation.
class ml_CEulerZYX;

//! Quaternion representation of 3-dimensional orientation.
class ml_CQuaternion;


//*****************************************************************************
// Class declarations

//! Class representing an orientation by means of 3x3 orthonormal matrices.
//!
//! This class provides a general purpose orientation representation by means of
//! a 3x3 orthonormal matrix in which the columns are the images of the canonical
//! unit vectors. Hence, the first column is the image of the x-axis, the second
//! of the y-axis and the third of the z-axis:
//! \verbatim
//!                 [ | | | ]
//!         Ori  =  [ x y z ]
//!                 [ | | | ]
//! \endverbatim
//!
class ml_COriMatrix
{
   public:

      //! \name Constructors and destructors.
      //! @{

      //! Constructor creating a 3x3 orthonormal matrix initializing it with the
      //! identity matrix.
      _DLL_ ml_COriMatrix();

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_COriMatrix(const ml_COriMatrix& Initial);

      //! Constructor creating a 3x3 orthonormal matrix initializing it with given
      //! 9 values.
      //!
      //! \verbatim
      //!                 [ M00 M01 M02 ]
      //!         Ori  =  [ M10 M11 M12 ]
      //!                 [ M20 M21 M22 ]
      //! \endverbatim
      //!
      //! \param[in] M00  First row, first column.
      //! \param[in] M01  First row, second column.
      //! \param[in] M02  First row, third column.
      //! \param[in] M10  Second row, first column.
      //! \param[in] M11  Second row, second column.
      //! \param[in] M12  Second row, third column.
      //! \param[in] M20  Third row, first column.
      //! \param[in] M21  Third row, second column.
      //! \param[in] M22  Third row, third column.
      //!
      _DLL_ ml_COriMatrix(const double& M00, const double& M01, const double& M02,
                          const double& M10, const double& M11, const double& M12,
                          const double& M20, const double& M21, const double& M22);

      //! Constructor creating a 3x3 orthonormal matrix initializing it with the
      //! orientation defined by a rotation axis and a rotation angle.
      //!
      //! \param[in] RotAxis   Rotation axis.
      //! \param[in] RotAngle  Rotation angle in [rad].
      //!
      _DLL_ ml_COriMatrix(const ml_CVector3& RotAxis, const double& RotAngle);

      //! Constructor creating a 3x3 orthonormal matrix initializing it with the
      //! orientation defined by a origin, a point on the positive x-axis and
      //! a point in the x-y-plane, where y has to be positive.
      //!
      //! \param[in] Origin       Origin.
      //! \param[in] PtOnXAxis    Point on the positive x-axis.
      //! \param[in] PtOnXYPlane  Point in the x-y-plane with positive y.
      //!
      _DLL_ ml_COriMatrix(const ml_CVector3& Origin,
                          const ml_CVector3& PtOnXAxis,
                          const ml_CVector3& PtOnXYPlane);

      //! Constructor creating a 3x3 orthonormal matrix initializing it with the
      //! orientation defined by a set of ZYX-Euler angles.
      //!
      //! \param[in] Euler  Euler angles.
      //!
      _DLL_ ml_COriMatrix(const ml_CEulerZYX& Euler);

      //! Constructor creating a 3x3 orthonormal matrix initializing it with the
      //! orientation defined by a quaternion.
      //!
      //! \param[in] Quaternion  Quaternion.
      //!
      _DLL_ ml_COriMatrix(const ml_CQuaternion& Quaternion);

      //! Constructor creating a 3x3 orthonormal matrix initializing it with the
      //! values inside a two dimensional double array.
      //!
      //! \verbatim
      //!                 [ InitOri[0][0] InitOri[0][1] InitOri[0][2] ]
      //!         Ori  =  [ InitOri[1][0] InitOri[1][1] InitOri[1][2] ]
      //!                 [ InitOri[2][0] InitOri[2][1] InitOri[2][2] ]
      //! \endverbatim
      //!
      //! \param[in] InitOri  Init values.
      //!
      _DLL_ ml_COriMatrix(const double InitOri[3][3]);
      //! @}

      //! \name Init-functions and basic methods.
      //! @{

      //! Reinit with the identity matrix.
      _DLL_ void setIdentity();

      //! Check a matrix whether it is orthonormal and therefore represents an
      //! orientation.
      //!
      //! \retval true   Matrix is orthonormal.
      //! \retval false  Matrix is notorthonormal.
      _DLL_ bool isOrthonormal() const;

      //! Reinit with a vector for each column.
      //!
      //! \verbatim
      //!                 [   |    |    |  ]
      //!         Ori  =  [ Col1 Col2 Col3 ]
      //!                 [   |    |    |  ]
      //! \endverbatim
      //!
      //! \param[in] Col1  Values for first column.
      //! \param[in] Col2  Values for second column.
      //! \param[in] Col3  Values for third column.
      //!
      _DLL_ void setValues(const ml_CVector3& Col1,
                           const ml_CVector3& Col2,
                           const ml_CVector3& Col3);
      //! @}

      //! \name Assignment and arithmetic operators.
      //! @{

      //! Matrix assignment.
      //!
      //! \param[in] Rhs  Right-hand side in assignment.
      //!
      //! \return Assigned matrix.
      //!
      _DLL_ ml_COriMatrix& operator=(const ml_COriMatrix& Rhs);

      //! Matrix multiplication.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this) * Rhs.
      //!
      _DLL_ ml_COriMatrix operator*(const ml_COriMatrix& Rhs) const;

      //! Matrix multiplication.
      //!
      //! (*this) = (*this) * Rhs.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      _DLL_ void operator*=(const ml_COriMatrix& Rhs);

      //! Matrix multiplication with a vector.
      //!
      //! \param[in] Vector  Right-hand side in multiplication.
      //!
      //! \return (*this) * Rhs.
      //!
      _DLL_ ml_CVector3 operator*(const ml_CVector3& Vector) const;

      //! Invert matrix.
      //!
      //! \return (*this)^(-1).
      //!
      _DLL_ ml_COriMatrix  operator~() const;

      //! Invert matrix.
      //!
      //! (*this) = (*this)^(-1).
      //!
      _DLL_ void  invertThis();
      //! @}

      //! \name Comparison operators.
      //! @{

      //! Compares two matrices for equality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) == Rhs.
      //! \retval false  (*this) != Rhs.
      //!
      _DLL_ bool  operator==(const ml_COriMatrix& Rhs) const;

      //! Compares two matrices for inequality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) != Rhs.
      //! \retval false  (*this) == Rhs.
      //!
      _DLL_ bool  operator!=(const ml_COriMatrix& Rhs) const;
      //! @}

      //! \name Quick arithmetic operations.
      //! @{

      //! Invert the actual matrix and right-hand multiply the result with another matrix.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this)^(-1) * Rhs.
      //!
      _DLL_ ml_COriMatrix invMul(const ml_COriMatrix &Rhs) const;

      //! Invert the actual matrix and right-hand multiply the result with a vector.
      //!
      //! \param[in] Vector  Right-hand side in multiplication.
      //!
      //! \return (*this)^(-1) * Vector.
      //!
      _DLL_ ml_CVector3 invMul(const ml_CVector3& Vector) const;

      //! Invert the right-hand matrix and multiply the result with actual matrix.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (Rhs)^(-1) * (*this).
      //!
      _DLL_ ml_COriMatrix mulInv(const ml_COriMatrix &Rhs) const;

      //! Invert the actual matrix and right-hand multiply the result with another
      //! inverted matrix.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this)^(-1) * (Rhs)^(-1).
      //!
      _DLL_ ml_COriMatrix invMulInv(const ml_COriMatrix& Rhs) const;

      //! Multiply a orientation matrix around X about angle Phi from the left
      //! side on the actual matrix.
      //!
      //! (*this) = Rot_X(Phi) * (*this)
      //!
      //! \param[in] Phi  Rotation angle.
      //!
      _DLL_ void mulRotXLeft(const double& Phi);

      //! Multiply a orientation matrix around Y about angle Phi from the left
      //! side on the actual matrix.
      //!
      //! (*this) = Rot_Y(Phi) * (*this)
      //!
      //! \param[in] Phi  Rotation angle.
      //!
      _DLL_ void mulRotYLeft(const double& Phi);

      //! Multiply a orientation matrix around Z about angle Phi from the left
      //! side on the actual matrix.
      //!
      //! (*this) = Rot_Z(Phi) * (*this)
      //!
      //! \param[in] Phi  Rotation angle.
      //!
      _DLL_ void mulRotZLeft(const double& Phi);

      //! Multiply a orientation matrix around X about angle Phi from the right
      //! side on the actual matrix.
      //!
      //! (*this) = (*this) * Rot_X(Phi)
      //!
      //! \param[in] Phi  Rotation angle.
      //!
      _DLL_ void mulRotXRight(const double& Phi);

      //! Multiply a orientation matrix around Y about angle Phi from the right
      //! side on the actual matrix.
      //!
      //! (*this) = (*this) * Rot_Y(Phi)
      //!
      //! \param[in] Phi  Rotation angle.
      //!
      _DLL_ void mulRotYRight(const double& Phi);

      //! Multiply a orientation matrix around Z about angle Phi from the right
      //! side on the actual matrix.
      //!
      //! (*this) = (*this) * Rot_Z(Phi)
      //!
      //! \param[in] Phi  Rotation angle.
      //!
      _DLL_ void mulRotZRight(const double& Phi);

      //! @}

      //! \name Index based access to the matrix elements
      //! @{

      //! Get the indexed element from a matrix.
      //!
      //! \param[in] i Zero-based row index of element to get.
      //! \param[in] j Zero-based column index of element to get.
      //!
      //! \return (*this)[i][j].
      //!
      _DLL_ double& operator()(int i, int j) { return Entry[i][j]; }

      //! Get the indexed element from a matrix.
      //!
      //! \param[in] i Zero-based row index of element to get.
      //! \param[in] j Zero-based column index of element to get.
      //!
      //! \return (*this)[i][j].
      //!
      _DLL_ const double& operator()(int i, int j) const { return Entry[i][j]; }

      //! Get the indexed row from a matrix.
      //!
      //! \param[in] RowNo  One-based position index of row to get.
      //!
      //! \return (*this)[RowNo][*].
      //!
      _DLL_ ml_CVector3 getRow(const unsigned int RowNo) const;

      //! Get the indexed column from a matrix.
      //!
      //! \param[in] ColumnNo  One-based position index of column to get.
      //!
      //! \return (*this)[*][ColumnNo].
      //!
      _DLL_ ml_CVector3 getColumn(const unsigned int ColumnNo) const;
      //! @}

      //! \name Friend classes.
      //! @{

      //! Class of 3x3 orientation matrices represented by ZYX-Euler angles.
      friend class ml_CEulerZYX;

      //! Class of 3x3 orientation matrices represented by quaternions.
      friend class ml_CQuaternion;
      //! @}

      //! \name Friend output operators.
      //! @{

      //! Output stream operator for 3x3 orthonormal orientation matrices.
      //!
      //! \param[in] o       Output stream.
      //! \param[in] matrix  Output matrix.
      //!
      //! \return Output stream.
      //!
      friend _DLL_ std::ostream& operator<<(std::ostream& o, const ml_COriMatrix& matrix);
      //! @}

   protected:

      //! Statically allocatated matrix elements.
      double Entry[3][3];
};

_DLL_ const ml_COriMatrix& ml_IdentityOriMatrix();


//! Template class for elementary rotations.
//!
//! This template base class provides elementary rotations. In particular
//! this base class is used to define elementary rotations for the x-, y-
//! and z-axis.
//!
template <int n1, int n2, int n3>
class ml_CElemRot : public ml_COriMatrix
{
   public:
      //! \name Constructors and destructors.
      //! @{

      //! Constructor initializing the elementary rotation with a rotation angle.
      //!
      //! \param[in] Phi  Rotation angle.
      //!
      _DLL_ ml_CElemRot(double Phi);
      //! @}
};

//! Class representing an orientation by means of intrinsic ZYX-Euler angles.
//!
//! This class provides a general purpose orientation representation by means of
//! intrinsic ZYX-Euler angles in which the orientation is given a sequence of 3
//! elementary rotations. First a rotation about the z-axis by the angle A, then
//! a rotation about the y-axis by the angle B and last a rotation about the x-axis
//! by the angle C.
//! \verbatim
//!            Ori  =  Rot_z(A) * Rot_y(B) * Rot_x(C)
//! \endverbatim
//!
//! \attention
//! The angles A, B, C are defined in degrees here.
//!
class ml_CEulerZYX
{
   public:
      //! \name Constructors and destructors.
      //! @{

      //! Constructor initializing A,B and C with zero.
      _DLL_ ml_CEulerZYX();

      //! Constructor initializing the Euler angles using three scalars.
      //!
      //! \param[in] InitA  Rotation angle A in [deg].
      //! \param[in] InitB  Rotation angle B in [deg].
      //! \param[in] InitC  Rotation angle C in [deg].
      //!
      _DLL_ ml_CEulerZYX(const double& InitA, const double& InitB, const double& InitC);

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CEulerZYX(const ml_CEulerZYX& Initial);

      //! Constructor creating Euler angles and initializing them with the
      //! orientation defined by a 3x3 orthonormal orientation matrix.
      //!
      //! \param[in] OriMat         3x3 orthonormal orientation matrix.
      //! \param[in] KRLConvention  Setting this option to true guarantees getting the same
      //!                           Euler angles as inside the KRC controller.
      //!
      _DLL_ ml_CEulerZYX(const ml_COriMatrix& OriMat, bool KRLConvention = true);
      //! @}

      //! \name Name-based access to vector elements.
      //! @{

      //! Get A value.
      //!
      //! \return Rotation angle A in [deg].
      //!
      _DLL_ double getA() const;

      //! Get B value.
      //!
      //! \return Rotation angle B in [deg].
      //!
      _DLL_ double getB() const;

      //! Get C value.
      //!
      //! \return Rotation angle C in [deg].
      //!
      _DLL_ double getC() const;
      //! @}

      //! \name Friend classes.
      //! @{

      //! Class of 3x3 orientation matrices.
      friend class ml_COriMatrix;
      //! @}

      //! \name Friend output operators.
      //! @{

      //! Output stream operator for Euler angles.
      //!
      //! \param[in] o    Output stream.
      //! \param[in] ABC  Output vector.
      //!
      //! \return Output stream.
      //!
      friend _DLL_ std::ostream& operator<<(std::ostream& o, const ml_CEulerZYX& ABC);
      //! @}

   protected:

      double  A; //!< Rotation angle A in [deg]
      double  B; //!< Rotation angle B in [deg]
      double  C; //!< Rotation angle C in [deg]
};


_DLL_ const ml_CEulerZYX& ml_IdentityEulerZYX();


class ml_CEulerRadZYX : public ml_CEulerZYX
{
public:

   _DLL_ ml_CEulerRadZYX(const double& InitA, const double& InitB, const double& InitC)
      : ml_CEulerZYX(ml_toDeg(InitA), ml_toDeg(InitB), ml_toDeg(InitC))
   {
   }

   _DLL_ ml_CEulerRadZYX(const double* Init)
      : ml_CEulerZYX(ml_toDeg(Init[0]), ml_toDeg(Init[1]), ml_toDeg(Init[2]))
   {
   }
};


//! Class representing an orientation by means of an unit quaternion.
//!
//! This class provides a general purpose orientation representation by means of
//! an unit quaternion. The unit quaternion q is a 4-dimensional vector, describing
//! the orientation as one rotation around the vector v = [v_x, v_y, v_z] about the
//! angle phi.
//! \verbatim
//!                      [ cos (phi/2)     ]
//!                q  =  [ sin (phi/2) v_x ]
//!                      [ sin (phi/2) v_y ]
//!                      [ sin (phi/2) v_z ]
//! \endverbatim
//!
class ml_CQuaternion
{
   public:
      //! \name Constructors and destructors.
      //! @{

      //! Constructor initializing the quaternion using four scalars.
      //!
      //! If no scalar values are given this constructor intializes the
      //! quaternion to the identity quaternion.
      //!
      //! \param[in] InitialW  W = cos (phi/2).
      //! \param[in] InitialX  Value X in vector.
      //! \param[in] InitialY  Value Y in vector.
      //! \param[in] InitialZ  Value Z in vector.
      //!
      _DLL_ ml_CQuaternion(double InitialW = 1.0, double InitialX = 0.0,
                           double InitialY = 0.0, double InitialZ = 0.0);

      //! Constructor initializing the quaternion using one scalar and
      //! a 3-dimensional vector.
      //!
      //! \param[in] InitialW  W = cos (phi/2).
      //! \param[in] Initial   Initial vector.
      //!
      _DLL_ ml_CQuaternion(double InitialW, const ml_CVector3& Initial);

      //! Constructor initializing the quaternion using a 3x3 orthonormal
      //! orientation matrix.
      //!
      //! \param[in] Initial  Initial matrix.
      //!
      _DLL_ ml_CQuaternion(const ml_COriMatrix& Initial);

      //! Constructor initializing the quaternion using a 3-dimensional rotation vector.
      //!
      //! The rotation angle is assumed to be the length of the rotation vector.
      //!
      //! \param[in] RotVector  Rotation vector.
      //!
      _DLL_ ml_CQuaternion(const ml_CVector3& RotVector);

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CQuaternion(const ml_CQuaternion& Initial);

      //! Constructor initializing the quaternion using a set of intrinsic
      //! ZYX-Euler angles.
      //!
      //! \param[in] EulerAngles  Euler angles.
      //!
      _DLL_ ml_CQuaternion(const ml_CEulerZYX& EulerAngles);
      //! @}

      //! \name Init-functions and basic methods.
      //! @{

      //! Normalise the quaternion to unit length and hence to represent a unit
      //! quaternion.
      _DLL_ void scaleToUnit();
      //! @}

      //! \name Assignment and arithmetic operators.
      //! @{

      //! Quaternion assignment.
      //!
      //! \param[in] Rhs  Right-hand side in assignment.
      //!
      //! \return Assigned quaternion.
      //!
      _DLL_ ml_CQuaternion& operator=(const ml_CQuaternion& Rhs);

      //! Quaternion multiplication.
      //!
      //! Quaternion multiplication is defined by
      //! \li W       = (*this).W * Rhs.W - (*this).RotAxis * Rhs.RotAxis
      //! \li RotAxis = (*this).W * Rhs.RotAxis + Rhs.W * (*this).RotAxis + ((*this).RotAxis & Rhs.RotAxis);
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this) * Rhs.
      //!
      _DLL_ ml_CQuaternion operator*(const ml_CQuaternion& Rhs) const;

      //! Quaternion multiplication.
      //!
      //! Quaternion multiplication is defined by
      //! \li (*this).W       = (*this).W * Rhs.W - (*this).RotAxis * Rhs.RotAxis
      //! \li (*this).RotAxis = (*this).W * Rhs.RotAxis + Rhs.W * (*this).RotAxis + ((*this).RotAxis & Rhs.RotAxis);
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      _DLL_ void operator*=(const ml_CQuaternion& Rhs);

      //! Quaternion subtraction.
      //!
      //! Quaternion subtraction is defined by
      //! \li W       = -(*this).W
      //! \li RotAxis = -(*this).RotAxis
      //!
      //! \return -(*this).
      //!
      _DLL_ ml_CQuaternion operator-() const;
      //! @}

      //! \name Friend operators.
      //! @{

      //! Calculate scalar times quaternion with quaternion at Left-hand side.
      //!
      //! \param[in] Quaternion  Left-hand side in multiplication.
      //! \param[in] Lambda      Scalar factor in multiplication.
      //!
      //! \return Lambda * Quaternion.
      //!
      friend _DLL_ ml_CQuaternion operator*(const ml_CQuaternion& Quaternion,
                                            double Lambda);

      //! Calculate scalar times quaternion with quaternion at right-hand side.
      //!
      //! \param[in] Quaternion  Right-hand side in multiplication.
      //! \param[in] Lambda      Scalar factor in multiplication.
      //!
      //! \return Lambda * Quaternion.
      //!
      friend _DLL_ ml_CQuaternion operator*(double Lambda,
                                            const ml_CQuaternion& Quaternion);
      //! @}

      //! \name Classic quaternion operations.
      //! @{

      //! Dot product of two quaternions.
      //!
      //! \param[in] Rhs  Right-hand side in dot product.
      //!
      //! \return (*this).W * Rhs.W + (*this).RotAxis * Rhs.RotAxis.
      //!
      _DLL_ double scalarProduct(const ml_CQuaternion& Rhs) const;

      //! Inverse of a quaternion.
      //!
      //! \remark
      //! For unit quaternions inversion is the same as conjugation.
      //!
      //! \return (*this)^(-1).
      //!
      _DLL_ ml_CQuaternion  operator~() const;

      //! Inverse of a quaternion.
      //!
      //! \remark
      //! For unit quaternions inversion is the same as conjugation.
      //!
      //! \return (*this)^(-1).
      //!
      _DLL_ ml_CQuaternion getInverse() const;

      //! Conjugation of a quaternion.
      //!
      //! \remark
      //! For unit quaternions conjugation is the same as inversion.
      //!
      //! (*this) = (*this)^(-1).
      //!
      _DLL_ void Conjugate();

      //! Conjugation of a quaternion.
      //!
      //! \remark
      //! For unit quaternions conjugation is the same as inversion.
      //!
      //! \return (*this)^(-1).
      //!
      _DLL_ ml_CQuaternion getConjugate() const;

      //! Calculate the rotation vector represented by a quaternion (length = angle).
      //!
      //! \return Rotation vector of either length zero or at least \c ML_MIN_ROTATION
      //!         (about 0,000115 degrees).
      //!
      _DLL_ ml_CVector3 getRotVector() const;

      //! Rotate a vector by the rotation given through the actual object.
      //!
      //! \param[in] Vec  Vector to rotate.
      //!
      //! \return Rotated vector.
      //!
      _DLL_ ml_CVector3 rotate(const ml_CVector3& Vec) const;

      //! @}

      //! \name Comparison operators.
      //! @{

      //! Compares two quaternions for equality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) == Rhs.
      //! \retval false  (*this) != Rhs.
      //!
      _DLL_ bool  operator== (const ml_CQuaternion& Rhs) const;

      //! Compares two quaternions for inequality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) != Rhs.
      //! \retval false  (*this) == Rhs.
      //!
      _DLL_ bool  operator!= (const ml_CQuaternion& Rhs) const;
      //! @}

      //! \name Name-based access to quaternion elements.
      //! @{

      //! Get W value (only for 3-dimensional vectors).
      //!
      //! \return (*this).W.
      //!
      _DLL_ double getW() const;

      //! Get RotAxis value (only for 3-dimensional vectors).
      //!
      //! \return (*this).RotAxis.
      //!
      _DLL_ ml_CVector3 getRotAxis() const;
      //! @}

      //! \name Friend output operators.
      //! @{

      //! Output stream operator for quaternions.
      //!
      //! \param[in] o     Output stream.
      //! \param[in] quat  Output quaternion.
      //!
      //! \return Output stream.
      //!
      friend _DLL_ std::ostream& operator<<(std::ostream& o, const ml_CQuaternion& quat);
      //! @}

      //! \name Friend classes.
      //! @{

      //! Class of 3x3 orientation matrices.
      friend class ml_COriMatrix;
      //! @}

   protected:
      //! W = cos(phi/2), where phi denotes the rotation angle.
      double W;

      //! A vector of length sin(phi/2) whose direction is the rotation axis.
      ml_CVector3 RotAxis;
};

_DLL_ const ml_CQuaternion& ml_IdentityQuaternion();

//*****************************************************************************
// Function declarations

//! \name Output operators.
//! @{

//! Output stream operator for 3x3 orthonormal orientation matrices.
//!
//! \param[in] o       Output stream.
//! \param[in] matrix  Output matrix.
//!
//! \return Output stream.
//!
_DLL_ std::ostream& operator<<(std::ostream& o, const ml_COriMatrix& matrix);

//! Output stream operator for Euler angles.
//!
//! \param[in] o    Output stream.
//! \param[in] ABC  Euler angles.
//!
//! \return Output stream.
//!
_DLL_ std::ostream& operator<<(std::ostream& o, const ml_CEulerZYX& ABC);

//! Output stream operator for quaternions.
//!
//! \param[in] o     Output stream.
//! \param[in] quat  Output quaternion.
//!
//! \return Output stream.
//!
_DLL_ std::ostream& operator<<(std::ostream& o, const ml_CQuaternion& quat);
//! @}

//! \name Functions for orientation classes
//! @{

//! Dot product of two quaternions.
//!
//! \param[in] Q0  Left-hand side in dot product.
//! \param[in] Q1  Right-hand side in dot product.
//!
//! \return Q0.W * Q1.W + Q0.RotAxis * Q1.Rhs.RotAxis.
//!
_DLL_ double operator^(const ml_CQuaternion& Q0, const ml_CQuaternion& Q1);

//! Calculate the rotation angle in between two orientations of class ml_CQuaternion.
//!
//! \remark This functions gives the orientation in ABC-Space. In quaternion space
//!         orientation would be divided in half.
//!
//! \param[in] Q0  Quaternion.
//! \param[in] Q1  Quaternion.
//!
//! \return Angle in interval [0, 2*pi].
//!
_DLL_ double calcDeltaAngle(const ml_CQuaternion& Q0, const ml_CQuaternion& Q1);

//! Calculate the angle in between two orientations of class ml_COriMatrix.
//!
//! \param[in] M0  3x3 orthonormal orientation matrix.
//! \param[in] M1  3x3 orthonormal orientation matrix.
//!
//! \return Angle in interval [0, pi].
//!
_DLL_ double calcDeltaAngle(const ml_COriMatrix& M0, const ml_COriMatrix& M1);
//! @}

//*****************************************************************************
// Typedef declarations

//! Declare special case of template class #ml_CElemRot for rotations about the x-axis.
typedef ml_CElemRot<1,2,0> ml_CElemRotX;

//! Declare special case of template class #ml_CElemRot for rotations about the y-axis.
typedef ml_CElemRot<2,0,1> ml_CElemRotY;

//! Declare special case of template class #ml_CElemRot for rotations about the z-axis.
typedef ml_CElemRot<0,1,2> ml_CElemRotZ;

#endif // _ML_CORIENTATION_H
