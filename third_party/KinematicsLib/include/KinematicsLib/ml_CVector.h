// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2001, 2003, 2005, 2007 -- 2015
// KUKA Roboter GmbH, Germany. All Rights Reserved.

//! \file
//! Public interface to MathLib vector calculus classes and methods with dynamic
//! and static memory allocation.
//!
//! Public header file publishing all vector calculus methods of the
//! MathLib.
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
// 26.09.2003  Sedlmayr
//             maintenance platform compatibility standard c++ library
// 08.03.2005  Mueller-Sommer
//             Achieve compatibility for WIN32 compiler in project HFPENV
// 18.03.2005  C.Spiess             N/A
//             Compiling with Workbench 2.2
// 27.11.2007  Aurnhammer           N/A
//             added division by scalar and a general vector class
// 28.08.2007  Dürndorfer           Aurnhammer
//             Mergefehler behoben
// 25.07.2008  Schreittmiller       Burkhart/Wiedemann
//             A2145: Kreiswinkel (CA) fuer SCIRC
// 06.04.2009  Aurnhammer           Burkhart/Schreittmiller/Wiedemann (CI)
//             KernelSystem-DLL und Prototyp HFP-CP
// 05.01.2010  Schreittmiller       ---
//             getDeltaAngle() in calcDeltaAngle() umbenannt.
// 12.01.2010  Aurnhammer
//             Anpassungen für FrameVisu und VC++ 6.0
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
// 06.03.2015  Burkhart/Schreittmiller
//             Berechnung von Lot im R^n und orthogonaler Projektion in eine Ebene
//             im R^3 eingeführt
//*****************************************************************************
//</CL>

#ifndef _ML_CVECTOR_H
#define _ML_CVECTOR_H

//*****************************************************************************
// Header files

// System headers
#include <iostream>

// Public headers
#include "dllBuildDefines.h" // _DLL_

//*****************************************************************************
// Version defines and checks

//! Versionnumber: Usage as date "xx.xx.xx" (e.g. 12.02.03 = 0x00120203).
//!
//! This version number is checked in between all MathLib interface headers
//! at compile time.
#define PLATFORM_MATHLIB_VECTOR_VERSION_NUMBER 0x00121015

//*****************************************************************************
// Constant declarations and defines

//*****************************************************************************
// Type declarations

//! Dynamically allocated matrices.
class ml_CDynMatrix;

//! Statically allocated 3x3-dimensional orientation matrices.
class ml_COriMatrix;

//*****************************************************************************
// Class declarations

//! Template class for general purpose vector calculus with vectors of dimension \c dim.
//!
//! This template base class provides a general purpose vector calculus interface. In particular
//! this base class is used to define vector classes of various dimensions with static memory
//! allocation.
template <int dim>
class  ml_CVectorBase
{
   public:
      //! \name Constructors and destructors.
      //! @{

      //! Constructor initializing with a zero vector.
      _DLL_ ml_CVectorBase();

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CVectorBase(const ml_CVectorBase& Initial);

      //! Constructor, that initialises with a double array.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CVectorBase(const double* Initial);

      //! Destructor.
      _DLL_ ~ml_CVectorBase();
      //! @}

      //! \name Init-functions and basic methods.
      //! @{

      //! Reinit a vector with zeros.
      _DLL_ void setZero();

      //! Scale a vector to euclidian length 1.
      _DLL_ void scaleToUnitLength();

      //! Calculate a vectors euclidian length.
      //!
      //! \return Euclidian length of (*this).
      //!
      _DLL_ double getLength() const;

      //! Get a vectors dimension.
      //!
      //! \return Vector dimension of (*this).
      //!
      _DLL_ int getDim() const {return dim;}
      //! @}

      //! \name Assignment and arithmetic operators.
      //! @{

      //! Vector assignment.
      //!
      //! \param[in] Rhs  Right-hand side in assignment.
      //!
      //! \return Assigned vector.
      //!
      _DLL_ ml_CVectorBase& operator=(const ml_CVectorBase& Rhs);

      //! Vector addition.
      //!
      //! \param[in] Rhs  Right-hand side in addition.
      //!
      //! \return (*this) + Rhs.
      //!
      _DLL_ ml_CVectorBase operator+(const ml_CVectorBase& Rhs) const;

      //! Vector addition.
      //!
      //! (*this) = (*this) + Rhs
      //!
      //! \param[in] Rhs  Right-hand side in addition.
      //!
      _DLL_ void operator+=(const ml_CVectorBase& Rhs);

      //! Vector subtraction.
      //!
      //! \param[in] Rhs  Right-hand side in subtraction.
      //!
      //! \return (*this) - Rhs.
      //!
      _DLL_ ml_CVectorBase operator-(const ml_CVectorBase& Rhs) const;

      //! Vector subtraction.
      //!
      //! (*this) = (*this) - Rhs
      //!
      //! \param[in] Rhs  Right-hand side in subtraction.
      //!
      _DLL_ void operator-=(const ml_CVectorBase& Rhs);

      //! Vector complement.
      //!
      //! \return (-1) * (*this).
      //!
      _DLL_ ml_CVectorBase operator-() const;
      //! @}

      //! \name Classic vector operations.
      //!
      //! Methods implementing scalar multiplication are not defined as member functions.
      //! Due to the template mechanism this can unfortunatly not be done otherwise.
      //! @{

      //! Scalar multiplication for a vector.
      //!
      //! (*this) = lamda * (*this)
      //!
      //! \param[in] lambda  Scalar multiplication value.
      //!
      _DLL_ void operator*=(const double& lambda);

      //! Scalar division for a vector.
      //!
      //! (*this) = (1/lamda) * (*this)
      //!
      //! \param[in] lambda  Scalar division value.
      //!
      _DLL_ void operator/=(const double& lambda);

      //! Vector or inner product for two vectors.
      //!
      //! \param[in] Rhs  Right-hand side in inner product.
      //!
      //! \return (*this)^T * Rhs.
      //!
      _DLL_ double operator*(const ml_CVectorBase& Rhs) const;

      //! Computes this part of mine, which is orthogonal to w, or, in other words,
      //! the perpendicular. 
      //!
      _DLL_ ml_CVectorBase calcPerpendicular(const ml_CVectorBase& w) const;
      //! @}

      //! \name Comparison operators.
      //! @{

      //! Compares two vectors for equality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) == Rhs.
      //! \retval false  (*this) != Rhs.
      //!
      _DLL_ bool operator==(const ml_CVectorBase& Rhs) const;

      //! Compares two vectors for inequality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) != Rhs.
      //! \retval false  (*this) == Rhs.
      //!
      _DLL_ bool operator!=(const ml_CVectorBase& Rhs) const;
      //! @}

      //! \name Index based access to the elements of a vector
      //! @{

      //! Get the indexed element from a vector.
      //!
      //! \param[in] index  Zero-based position index of element to get.
      //!
      //! \return (*this)[index].
      //!
      _DLL_ double& operator[](int index);

      //! Get the indexed element from a vector.
      //!
      //! \param[in] index  Zero-based position index of element to get.
      //!
      //! \return (*this)[index].
      //!
      _DLL_ const double& operator[](int index) const;
      //! @}

      //! \name Friend classes.
      //! @{

      //! Class of 3x3 orientation matrices.
      friend class ml_COriMatrix;
      //! @}

   protected:

      //! Statically allocatated vector elements.
      double Entry[dim];
};


//! Class for general purpose vector calculus with vectors of dimension 3.
//!
//! This class provides a general purpose vector calculus interface for 3-dimensional vectore.
//! In particular the base class ml_CVectorBase is used to define a vector class of dimension 3
//! with static memory allocation.
class ml_CVector3 : public ml_CVectorBase<3>
{
   public:
      //! \name Constructors and destructors.
      //! @{

      //! Constructor initializing the vector using three scalars.
      //!
      //! \param[in] InitialX  (*this)[0].
      //! \param[in] InitialY  (*this)[1].
      //! \param[in] InitialZ  (*this)[2].
      //!
      _DLL_ ml_CVector3(const double& InitialX, const double& InitialY, const double& InitialZ);

      //! \name Dummy constructors.
      //!
      //! Due to the deduction of template #ml_CVectorBase these constructors have to be stated
      //! explicitly even though the functionalty is not overload.
      //! @{

      //! Constructor initializing with a zero vector.
      _DLL_ ml_CVector3() : ml_CVectorBase<3>() {};

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CVector3(const ml_CVectorBase<3>& Initial) : ml_CVectorBase<3>(Initial) {};

      //! Constructor, that initialises with a double array.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CVector3(const double* Initial) : ml_CVectorBase<3>(Initial) {};
      //! @}
      //! @}

      //! \name Classic operators for 3-dimensional vectors.
      //! @{

      //! Cross product.
      //!
      //! \param[in] Rhs  Right-hand side in cross product.
      //!
      //! \return Vector (*this) x Rhs.
      //!
      _DLL_ ml_CVector3 operator&(const ml_CVector3& Rhs) const;
      //! @}

      //! \name Name-based access to vector elements.
      //! @{

      //! Get x value.
      //!
      //! \return (*this)[0].
      //!
      _DLL_ double& x();

      //! Get y value.
      //!
      //! \return (*this)[1].
      //!
      _DLL_ double& y();

      //! Get z value.
      //!
      //! \return (*this)[2].
      //!
      _DLL_ double& z();

      //! Get x value.
      //!
      //! \return (*this)[0].
      //!
      _DLL_ const double& x() const;

      //! Get y value.
      //!
      //! \return (*this)[1].
      //!
      _DLL_ const double& y() const;

      //! Get z value.
      //!
      //! \return (*this)[2].
      //!
      _DLL_ const double& z() const;
      //! @}
};

//! Class for general purpose vector calculus.
//!
//! This class provides a general purpose vector calculus interface. In particular
//! this class can be used to define vectors of various dimensions with dynamic memory
//! allocation.
//!
//! \attention
//! Vectors with dynamic memory allocation should never be used in a real-time context
//! if not absolutly inevitable. For 3-dimensional vectors use #ml_CVector3,
//! for 6-dimensional vectors use #ml_CVector6 and for for 12-dimensional vectors use
//! #ml_CVector12 instead.
class ml_CDynVector
{
   public:
      //! \name Constructors and destructors.
      //! @{

      //! Constructor creating a vector of given fixed dimension and initializing
      //! it with a zero vector.
      //!
      //! \param[in] Dimension  Vector dimension.
      //!
      _DLL_ ml_CDynVector(int Dimension);

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CDynVector(const ml_CDynVector& Initial);

      //! Conversion constructor.
      //!
      //! Creates a ml_CDynVector vector of dimension 3 and intialises with the values
      //! of a ml_CVector3.
      //!
      //! \param[in] Vec3  Vector of dimension 3.
      //!
      _DLL_ ml_CDynVector(const ml_CVector3& Vec3);

      //! Destructor.
      _DLL_ virtual ~ml_CDynVector();
      //! @}

      //! \name Init-functions and basic methods.
      //! @{

      //! Reinit a vector with zeros.
      _DLL_ void setZero();

      //! Scale a vector to euclidian length 1.
      _DLL_ void scaleToUnitLength();

      //! Calculate a vectors euclidian length.
      //!
      //! \return Euclidian length of (*this).
      //!
      _DLL_ double getLength() const;

      //! Get a vectors dimension.
      //!
      //! \return Vector dimension of (*this).
      //!
      _DLL_ int getDim() const;
      //! @}

      //! \name Assignment and arithmetic operators.
      //! @{

      //! Vector assignment.
      //!
      //! \param[in] Rhs  Right-hand side in assignment.
      //!
      //! \return Assigned vector.
      //!
      _DLL_ ml_CDynVector& operator=(const ml_CDynVector& Rhs);

      //! Vector addition.
      //!
      //! \param[in] Rhs  Right-hand side in addition.
      //!
      //! \return (*this) + Rhs.
      //!
      _DLL_ ml_CDynVector operator+(const ml_CDynVector& Rhs) const;

      //! Vector addition.
      //!
      //! (*this) = (*this) + Rhs
      //!
      //! \param[in] Rhs  Right-hand side in addition.
      //!
      _DLL_ void operator+=(const ml_CDynVector& Rhs);

      //! Vector subtraction.
      //!
      //! \param[in] Rhs  Right-hand side in subtraction.
      //!
      //! \return (*this) - Rhs.
      //!
      _DLL_ ml_CDynVector operator-(const ml_CDynVector& Rhs) const;

      //! Vector subtraction.
      //!
      //! (*this) = (*this) - Rhs
      //!
      //! \param[in] Rhs  Right-hand side in subtraction.
      //!
      _DLL_ void operator-=(const ml_CDynVector& Rhs);

      //! Vector complement.
      //!
      //! \return -(*this).
      //!
      _DLL_ ml_CDynVector operator-() const;
      //! @}

      //! \name Classic vector operations.
      //!
      //! @{

      //! Scalar multiplication for a vector.
      //!
      //! (*this) = lamda * (*this)
      //!
      //! \param[in] lambda  Scalar multiplication value.
      //!
      _DLL_ void operator*=(const double& lambda);

      //! Scalar division for a vector.
      //!
      //! (*this) = (1/lamda) * (*this)
      //!
      //! \param[in] lambda  Scalar division value.
      //!
      _DLL_ void operator/=(const double& lambda);

      //! Vector or inner product for two vectors.
      //!
      //! \param[in] Rhs  Right-hand side in inner product.
      //!
      //! \return (*this)^T * Rhs.
      //!
      _DLL_ double operator*(const ml_CDynVector& Rhs) const;

      //! Cross product (only for 3-dimensional vectors).
      //!
      //! \param[in] Rhs  Right-hand side in cross product.
      //!
      //! \return Vector (*this) x Rhs.
      //!
      _DLL_ ml_CDynVector operator&(const ml_CDynVector& Rhs) const;

      //! Computes this part of mine, which is orthogonal to w, or, in other words,
      //! the perpendicular.
      //!
      _DLL_ ml_CDynVector calcPerpendicular(const ml_CDynVector& w) const;
      //! @}

      //! \name Friend operators.
      //! @{

      //! Calculate scalar times vector with vector at Left-hand side.
      //!
      //! \param[in] Lhs     Left-hand side in multiplication.
      //! \param[in] lambda  Scalar factor in multiplication.
      //!
      //! \return lambda * Lhs.
      //!
      friend _DLL_ ml_CDynVector operator*(const ml_CDynVector& Lhs, const double& lambda);

      //! Calculate scalar times vector with vector at right-hand side.
      //!
      //! \param[in] lambda  Scalar factor in multiplication.
      //! \param[in] Rhs     Right-hand side in multiplication.
      //!
      //! \return lambda * Rhs.
      //!
      friend _DLL_ ml_CDynVector operator*(const double& lambda, const ml_CDynVector& Rhs);

      //! Calculate Vector divided by scalar with vector at Left-hand side.
      //!
      //! \param[in] Lhs     Left-hand side in multiplication.
      //! \param[in] lambda  Scalar factor in multiplication.
      //!
      //! \return (1/lambda) * Lhs.
      //!
      friend _DLL_ ml_CDynVector operator/(const ml_CDynVector& Lhs, const double& lambda);

      //! Calculate matrix times vector.
      //!
      //! \param[in] LhsMatrix Matrix in multiplication.
      //! \param[in] Rhs       Vector in multiplication.
      //!
      //! \return LhsMatrix * Rhs.
      //!
      friend _DLL_ ml_CDynVector operator*(const ml_CDynMatrix& LhsMatrix, const ml_CDynVector& Rhs);
      //! @}

      //! \name Comparison operators.
      //! @{

      //! Compares two vectors for equality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) == Rhs.
      //! \retval false  (*this) != Rhs.
      //!
      _DLL_ bool operator==(const ml_CDynVector& Rhs) const;

      //! Compares two vectors for inequality.
      //!
      //! \param[in] Rhs  Right-hand side in comparision.
      //!
      //! \retval true   (*this) != Rhs.
      //! \retval false  (*this) == Rhs.
      //!
      _DLL_ bool operator!=(const ml_CDynVector& Rhs) const;
      //! @}

      //! \name Name-based access to vector elements.
      //! @{

      //! Get x value (only for 3-dimensional vectors).
      //!
      //! \return (*this)[0].
      //!
      _DLL_ double& x();

      //! Get y value (only for 3-dimensional vectors).
      //!
      //! \return (*this)[1].
      //!
      _DLL_ double& y();

      //! Get z value (only for 3-dimensional vectors).
      //!
      //! \return (*this)[2].
      //!
      _DLL_ double& z();

      //! Get x value (only for 3-dimensional vectors).
      //!
      //! \return (*this)[0].
      //!
      _DLL_ const double& x() const;

      //! Get y value (only for 3-dimensional vectors).
      //!
      //! \return (*this)[1].
      //!
      _DLL_ const double& y() const;

      //! Get z value (only for 3-dimensional vectors).
      //!
      //! \return (*this)[2].
      //!
      _DLL_ const double& z() const;
      //! @}

      //! \name Index based access to the elements of a vector
      //! @{

      //! Get the indexed element from a vector.
      //!
      //! \param[in] index  Zero-based position index of element to get.
      //!
      //! \return (*this)[index].
      //!
      _DLL_ double& operator[](int index);

      //! Get the indexed element from a vector.
      //!
      //! \param[in] index  Zero-based position index of element to get.
      //!
      //! \return (*this)[index].
      //!
      _DLL_ const double&  operator[](int index) const;
      //! @}

      //! \name Friend output operators.
      //! @{

      //! Output stream operator for a dynamically allocated vector.
      //!
      //! \param[in] o    Output stream.
      //! \param[in] vec  Output vector.
      //!
      //! \return Output stream.
      //!
      friend _DLL_ std::ostream& operator<<(std::ostream& o, const ml_CDynVector& vec);
      //! @}

   private:

      // Vector dimension
      int dim;

      // Pointer to dynamically allocated vector elements
      double *Entry;
};

//*****************************************************************************
// Function declarations

//! \name Scalar multiplication methods for template class ml_CVectorBase
//! @{

//! Calculate Vector times scalar with vector at Left-hand side.
//!
//! \param[in] Lhs     Left-hand side in multiplication.
//! \param[in] lambda  Scalar factor in multiplication.
//!
//! \return lambda * Lhs.
//!
template <int dim> _DLL_ ml_CVectorBase<dim> operator*(const ml_CVectorBase<dim>& Lhs, const double& lambda);

//! Calculate scalar times vector with vector at right-hand side.
//!
//! \param[in] lambda  Scalar factor in multiplication.
//! \param[in] Rhs     Right-hand side in multiplication.
//!
//! \return lambda * Rhs.
//!
template <int dim> _DLL_ ml_CVectorBase<dim> operator*(const double& lambda, const ml_CVectorBase<dim>& Rhs);

//! Calculate Vector divided by scalar with vector at Left-hand side.
//!
//! \param[in] Lhs     Left-hand side in multiplication.
//! \param[in] lambda  Scalar factor in multiplication.
//!
//! \return (1/lambda) * Lhs.
//!
template <int dim> _DLL_ ml_CVectorBase<dim> operator/(const ml_CVectorBase<dim>& Lhs, const double& lambda);
//! @}

//! \name Output operators.
//! @{

//! Output stream operator for a statically allocated vector of template class ml_CVectorBase.
//!
//! \param[in] o    Output stream.
//! \param[in] vec  Output vector.
//!
//! \return Output stream.
//!
template <int dim> _DLL_ std::ostream& operator<<(std::ostream& o, const ml_CVectorBase<dim>& vec);
//! @}

//! Calculate the angle in between two vectors of template class ml_CVectorBase.
//!
//! If one of the two vectors is zero zero is returned.
//!
//! \param[in]  A Vector in R^n.
//! \param[in]  B Vevtor in R^n.
//!
//! \return     Angle in interval [0, pi].
//!
template <int dim> _DLL_ double calcDeltaAngle(ml_CVectorBase<dim> A, ml_CVectorBase<dim> B);

//*****************************************************************************
// Typedef declarations

//! Declare special case of template class #ml_CVectorBase for dimension 6
typedef ml_CVectorBase<6>  ml_CVector6;

//! Declare special case of template class #ml_CVectorBase for dimension 12
typedef ml_CVectorBase<12> ml_CVector12;

#endif  // _ML_CVECTOR_H
