// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2012 - 2015
// KUKA Roboter GmbH, Germany. All Rights Reserved.

//! \file
//! Public interface to MathLib matrix calculus classes and methods with dynamic
//! and static memory allocation.
//!
//! Public header file publishing all matrix calculus methods of the
//! MathLib.
//!
//! \stable{MathLib}

//<CL>
//*****************************************************************************
// Date        Programmer           Reviewer
//             Change note
//-----------------------------------------------------------------------------
// 24.07.2012  Hüttenhofer         Aurnhammer
//             Dynamisch allokierbare Vektor- und Matrixklasse
// 30.08.2012  Aurnhammer          Tronnier
//             Umzug der MathLib nach BaseLib
// 31.10.2012  Tronnier            Aurnhammer
//             Nur Versionserhöhung durch:
//             CRDB53782: "Schnittstelle KernelSystemDll verändert"
// 20.10.2014  Aurnhammer          Wiedemann
//             Plattformmerge KRC/Sunrise (Team MP)
// 22.06.2015  Aurnhammer           Burkhart
//             Getrennte Versionierung von der MathLib im Plattformbranch im
//             Vergleich zum Rest der KernelDll
//*****************************************************************************
//</CL>

#ifndef _ML_CMATRIX_H
#define _ML_CMATRIX_H

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
#define PLATFORM_MATHLIB_MATRIX_VERSION_NUMBER 0x00121015

//*****************************************************************************
// Constant declarations and defines

//*****************************************************************************
// Type declarations

//! Dynamically allocated vectors.
class ml_CDynVector;

//*****************************************************************************
// Class declarations

//! Class for general purpose matrix calculus.
//!
//! This class provides a general purpose matrix calculus interface. In particular
//! this class can be used to define matrices of various dimensions with dynamic memory
//! allocation.
//!
//! \attention
//! Matrices with dynamic memory allocation should never be used in a real-time context
//! if not absolutly inevitable. For 3x3-dimensional orthonormal matrices use #ml_COriMatrix
//! for general 3x3-dimensional orthonormal matrices use #ml_CMatrixBase
class ml_CDynMatrix
{
   public:
      //! \name Constructors and destructors.
      //! @{

      //! Constructor creating a matrix of given fixed dimensions and initializing
      //! it with a zero matrix.
      //!
      //! \param[in] NoOfRows     Number of rows.
      //! \param[in] NoOfColumns  Number of columns.
      //!
      _DLL_ ml_CDynMatrix(const int NoOfRows, const int NoOfColumns);

      //! Copy constructor.
      //!
      //! \param[in] Initial  Initial value.
      //!
      _DLL_ ml_CDynMatrix(const ml_CDynMatrix& Initial);

      //! Destructor.
      _DLL_ virtual ~ml_CDynMatrix();
      //! @}

      //! \name Init-functions and basic methods.
      //! @{

      //! Get a matrix's number of rows.
      //!
      //! \return Number of rows of (*this).
      //!
      _DLL_ int noOfRows() const;

      //! Get a matrix's number of columns.
      //!
      //! \return Number of columns of (*this).
      //!
      _DLL_ int noOfColumns() const;
      //! @}

      //! \name Assignment and arithmetic operators.
      //! @{

      //! Matrix assignment.
      //!
      //! \param[in] Rhs  Right-hand side in assignment.
      //!
      //! \return Assigned matrix.
      //!
      _DLL_ ml_CDynMatrix operator=(const ml_CDynMatrix& Rhs);

      //! Matrix addition.
      //!
      //! \param[in] Rhs  Right-hand side in addition.
      //!
      //! \return (*this) + Rhs.
      //!
      _DLL_ ml_CDynMatrix operator+(const ml_CDynMatrix& Rhs) const;

      //! Matrix subtraction.
      //!
      //! \param[in] Rhs  Right-hand side in subtraction.
      //!
      //! \return (*this) - Rhs.
      //!
      _DLL_ ml_CDynMatrix operator-(const ml_CDynMatrix& Rhs) const;
      //! @}

      //! \name Classic matrix operations.
      //! @{

      //! Matrix multiplication.
      //!
      //! \param[in] Rhs  Right-hand side in multiplication.
      //!
      //! \return (*this) * Rhs.
      //!
      _DLL_ ml_CDynMatrix operator*(const ml_CDynMatrix& Rhs) const;

      //! Calculate scalar times matrix with matrix at Left-hand side.
      //!
      //! \param[in] LhsMatrix Left-hand side in multiplication.
      //! \param[in] RhsFactor Scalar factor in multiplication.
      //!
      //! \return RhsFactor * LhsMatrix.
      //!
      friend _DLL_ ml_CDynMatrix operator*(const ml_CDynMatrix& LhsMatrix, const double RhsFactor);

      //! Calculate scalar times matrix with matrix at right-hand side.
      //!
      //! \param[in] LhsFactor  Scalar factor in multiplication.
      //! \param[in] RhsMatrix  Right-hand side in multiplication.
      //!
      //! \return LhsFactor * RhsMatrix.
      //!
      friend _DLL_ ml_CDynMatrix operator*(const double LhsFactor, const ml_CDynMatrix& RhsMatrix);

      //! Calculate matrix times vector.
      //!
      //! \param[in] LhsMatrix Matrix in multiplication.
      //! \param[in] RhsVector Vector in multiplication.
      //!
      //! \return LhsMatrix * RhsVector.
      //!
      friend _DLL_ ml_CDynVector operator*(const ml_CDynMatrix& LhsMatrix, const ml_CDynVector& RhsVector);

      //! Transpose matrix.
      //!
      //! \return (*this)^T.
      //!
      _DLL_ ml_CDynMatrix getTranspose() const;

      //! Invert matrix.
      //!
      //! \return (*this)^(-1).
      //!
      _DLL_ ml_CDynMatrix operator~();

      //! Complement matrix.
      //!
      //! \return (-1) * (*this).
      //!
      _DLL_ ml_CDynMatrix operator-();
      //! @}

      //! \name Index based access to the matrix elements
      //! @{

      //! Get the indexed row from a matrix.
      //!
      //! \param[in] idx  Zero-based position index of row to get.
      //!
      //! \return (*this)[idx][*].
      //!
      _DLL_ ml_CDynVector& operator[](const int idx);

      //! Get the indexed row from a matrix.
      //!
      //! \param[in] idx  Zero-based position index of row to get.
      //!
      //! \return (*this)[idx][*].
      //!
      _DLL_ const ml_CDynVector& operator[](const int idx) const;

      //! Partial matrix extraction.
      //!
      //! \param[in] LineStart Zero-based position index of first row to get.
      //! \param[in] LineEnd   Zero-based position index of last row to get.
      //! \param[in] ColStart  Zero-based position index of first column to get.
      //! \param[in] ColEnd    Zero-based position index of last column to get.
      //!
      //! \return (*this)[LineStart to LineEnd][ColStart to ColEnd].
      //!
      _DLL_ ml_CDynMatrix operator()(const int LineStart, const int LineEnd,
                                     const int ColStart, const int ColEnd) const;
      //! @}

      //! \name Matrix algorithms.
      //! @{

      //! Solution of a linear least squares problem A*x = b.
      //!
      //! The linear least squares problem A*x = b is solved using the Householder algorithm.
      //!
      //! The matrix A is tested for rank deficiency and the result of this is stored in
      //! RankDeficiency:
      //! \li RankDeficieny[j] = 1: rank deficiency of matrix A in column j
      //! \li RankDeficieny[j] = 0: no rank deficiency of matrix A in column j
      //!
      //! If rank deficiency in column j occurs x[j] is not calculated and x[j] is set to zero.
      //! In ConditionNumbers the condition numbers are stored. Hence ConditionNumbers returns
      //! the diagonal entries of the triangular matrix.
      //!
      //! \param[in]  b                 Right-hand side.
      //! \param[in]  ConditionLimit    Error limit.
      //! \param[out] x                 Result vector.
      //! \param[out] ConditionNumbers  Vector of condition numbers.
      //! \param[out] RankDeficiency    Vector of rank deficiences.
      //!
      //! \return Residuum ||Ax - b||.
      _DLL_ double LS_RankCheck(ml_CDynVector& x, const ml_CDynVector& b,
                                double ConditionLimit, ml_CDynVector& RankDeficiency,
                                ml_CDynVector& ConditionNumbers) const;
      //! @}

   protected:

      //! Number of rows
      int Row;

      //! Number of columns
      int Col;

   private:

      // Pointer to dynamically allocated matrix elements
      ml_CDynVector **Entry;
};

#endif // _ML_CMATRIX_H
