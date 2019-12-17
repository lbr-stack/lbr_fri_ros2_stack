#ifndef POSITION_DESCRIPTION_H
#define POSITION_DESCRIPTION_H

// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2010-2015
// KUKA Roboter GmbH, Germany. All Rights Reserved.


//! \file
//! \ingroup KERNEL_DLL
//! \brief Several classes to describe robot positions within the Kernel DLL
//! \li CAxisPosition      axis specific position of a robot (gearbox output)
//! \li CCartesianPosition cartesian position of a robot
//! \li CMotorPosition     motor specific positions (robot drives)


//<CL>
//******************************************************************************
// Datum       Programmierer       Reviewer
//             Beschreibung
//------------------------------------------------------------------------------
// 26.10.2010  Burkhart/Schreittmiller
//             Erstellung
// 15.02.2011  Burkhart            Schreittmiller
//             Codeinspektion KERNEL-DLL eingearbeitet
// 14.02.2011  Schreittmiller      Burkhart
//             Überarbeitung Doxygen-Dokumentare.
// 05.10.2011  Aurnhammer          _PtpSplToBeReviewedByMP_
//             PTP-Spline Basisfunktionalität
// 25.11.2011  Burkhart            Schreittmiller
//             CAxisPosition, CCartesianPosition und CMotorPosition Konstruktoren
//             mit Defaultargumenten setzen alle nicht explizit übergebenen
//             Default Parameter nun ungültig
// 06.02.2012  Aurnhammer          _PtpSplToBeReviewedByMP_
//             Keine Abhängigkeiten zu KUKA-internen Headern in externen Headern
// 06.02.2012  Aurnhammer          _PtpSplToBeReviewedByMP_
//             Versionierung verfeinert
// 17.07.2012  Eisensehr           Burkhart, Dürndorfer
//             Integration des GS-Maschinendatenladers in die KernelSystem-DLL.
// 30.08.2012  Aurnhammer          Tronnier
//             Umzug der MathLib nach BaseLib
// 31.10.2012  Tronnier            Aurnhammer
//             Nur Versionserhöhung durch:
//             CRDB53782: "Schnittstelle KernelSystemDll verändert"
// 07.02.2014  Aurnhammer          Burkhart
//             Neue Schnittstelle für das Setzen der Position und Orientierung
// 20.10.2014  Aurnhammer          Wiedemann
//             Plattformmerge KRC/Sunrise (Team MP)
// 22.06.2015  Aurnhammer           Burkhart
//             Getrennte Versionierung von der MathLib im Plattformbranch im
//             Vergleich zum Rest der KernelDll
//******************************************************************************
//</CL>

#include "dllBuildDefines.h"
#include "globalConstants.h"
#include "ml_MathLib.h"

//! \brief Versionsnummer: Verwendung als "xx.xx.xx" (z.B. 12.02.03 = 0x00120203)
#define KRC_POSITION_VERSION_NUMBER 0x00150714

//! \brief Versionsnummer (MathLib): Verwendung als "xx.xx.xx" (z.B. 12.02.03 = 0x00120203)
//!        Muss genau dann auf die Version der MathLib angepasst werden, falls eine neue
//!        MathLib-Version in der Plattform zur Verfügung steht
#define KRC_MATHLIB_VERSION_NUMBER  0x00121015

// Versionsprüfungen
#if KRC_POSITION_VERSION_NUMBER != KRC_GLOBAL_CONSTANTS_VERSION_NUMBER
   #error positionDescription.h does not fit to globalConstants.h
#endif

#if KRC_MATHLIB_VERSION_NUMBER != PLATFORM_MATHLIB_VERSION_NUMBER
   #error KRC_MATHLIB_VERSION_NUMBER does not fit to PLATFORM_MATHLIB_VERSION_NUMBER
#endif

_DLL_ const double &DInvalid();
_DLL_ const int &IInvalid();

//! \name    Robot axes.
//! @{

//! \brief   Listing of all robot axis names.
enum ERobAxisNames
{
   A_1 = 1,  A_2 = 2,   A_3 = 4,   A_4 = 8,   A_5 = 16,   A_6 = 32
};

// ROB_AXES_MASK = A_1 | A_2 | A_3 | A_4 | A_5 | A_6;

//! @}


//! \name    Additional/External axis names.
//! @{

//! \brief   Listing of all additional/external axis names.
enum EExtAxisNames
{
   E_1 = 64, E_2 = 128, E_3 = 256, E_4 = 512, E_5 = 1024, E_6 = 2048
};

// EXT_AXES_MASK = E_1 | E_2 | E_3 | E_4 | E_5 | E_6;

//! @}


//! \name    Cartesian components.
//! @{

//! \brief   Listing of Cartesian components.
enum ECartPosNames
{
   C_X = 1, C_Y = 2, C_Z = 4, C_A = 8, C_B = 16, C_C = 32, C_STATUS = 4096, C_TURN = 8192
};

//! \brief   Bit field holding all Cartesian components.
const int CART_POS_MASK = C_X | C_Y | C_Z | C_A | C_B | C_C | C_STATUS | C_TURN;

//! @}


//! \name    Motor names.
//! @{

//! \brief   Listing of all motor names.
enum EMotorNames
{
   M_1 =  1, M_2 =   2, M_3 =   4, M_4  =   8, M_5  =   16, M_6  =   32,
   M_7 = 64, M_8 = 128, M_9 = 256, M_10 = 512, M_11 = 1024, M_12 = 2048
};

//! \brief   Bit field holding all motor names.
const int MOTOR_MASK = M_1 | M_2 | M_3 | M_4 | M_5 | M_6 | M_7 | M_8 | M_9 | M_10 | M_11 | M_12;

//! @}


//! \brief   Position of roboter and external axes described by axes angles
//!          following the KRL struct "e6axis" (gearbox output).
class CAxisPosition
{
   private:

      //! Gear train output values for robot and additional joints in [°] or [mm]:
      ml_CVector6 RobAxis;
      ml_CVector6 ExtAxis;

      //! Bitfield holding the validity of the position data.
      int ValidityMask;

   public:

      _DLL_ CAxisPosition(const ml_CVector6 &R, const ml_CVector6 &E);
      _DLL_ CAxisPosition(const int NumberOfAxes, const double* ValuesOfAxes);
      _DLL_ CAxisPosition(
            const double &A1 = DInvalid(), const double &A2 = DInvalid(), const double &A3 = DInvalid(),
            const double &A4 = DInvalid(), const double &A5 = DInvalid(), const double &A6 = DInvalid(),
            const double &E1 = DInvalid(), const double &E2 = DInvalid(), const double &E3 = DInvalid(),
            const double &E4 = DInvalid(), const double &E5 = DInvalid(), const double &E6 = DInvalid());

      //! \brief   Read/Write function for axis positions (default is all
      //!          axes).
      //!
      //! \param   R              Desired robot axis positions.
      //! \param   E              Desired external axis positions.
      //! \param   RequestedAxes  Bit field of axes to be read/written.
      //!
      //! \return  (Only read function) true, if every desired axis is valid
      //!          and stored in R and/or E, else false.
      //@{
      _DLL_ bool getFullPosition(ml_CVector6 &R, ml_CVector6 &E,
                           const int RequestedAxes = ROB_AXES_MASK | EXT_AXES_MASK) const;
      _DLL_ void setFullPosition(const ml_CVector6 &R, const ml_CVector6 &E,
                           const int RequestedAxes = ROB_AXES_MASK | EXT_AXES_MASK);
      //@}

      //! \brief   Read functions for single components based on the axis
      //!          index. The selected component is implicitly set valid.
      //!
      //! \param[in]   Axis    Index (beginning at 0) or name of required axis.
      //! \param[out]  Value   Position value of required axis in [°] or [mm].
      //!
      //! \return  true, if the required axis was valid, else false.
      //@{
      _DLL_ bool getRobAxis(const int Axis, double &Value) const;
      _DLL_ bool getExtAxis(const int Axis, double &Value) const;
      _DLL_ bool getRobAxis(const ERobAxisNames Axis, double &Value) const;
      _DLL_ bool getExtAxis(const EExtAxisNames Axis, double &Value) const;
      //@}

      //! \brief   Write functions for single components based on the axis
      //!          index or name. The selected component is implicitly set valid.
      //!
      //! \param[in]  Axis    Index (beginning at 0) or name of required axis.
      //! \param[in]  Value   Position value to be setted in [°] or [mm].
      //!
      //! \return   None.
      //@{
      _DLL_ void setRobAxis(const int Axis, const double Value);
      _DLL_ void setExtAxis(const int Axis, const double Value);
      _DLL_ void setRobAxis(const ERobAxisNames Axis, const double Value);
      _DLL_ void setExtAxis(const EExtAxisNames Axis, const double Value);
      //@}


      //! \brief   Function for reading the bit field holding the validity of
      //!          the position data.
      //!
      //! \return  The above described bit field.
      _DLL_ int  getValidityMask() const;

      //! \brief   Function for removing a validity flag (default is to remove
      //!          all validities).
      //!
      //! \param[in]  RequestedAxes  Bit field of axes to be invalidated.
      //!
      //! \return     None.
      _DLL_ void clearValidityMask(const int RequestedAxes = ROB_AXES_MASK | EXT_AXES_MASK);
};


//! \brief   Position of roboter and external axes described by frames and
//!          angles respectively following the KRL struct "E6POS".
class CCartesianPosition
{
   private:

      //! Spatial position of the robot:
      double m_X;
      double m_Y;
      double m_Z;

      //! Orientation of the robot (ZYX-Euler angles):
      double m_A;
      double m_B;
      double m_C;

      //! Bit field of status and turn flags as in KRL E6POS type:
      //@{
      int Status;
      int Turn;
	  int redundancy;
      //@}

      //! Additional joint values in [°] or [mm]:
      ml_CVector6 ExtAxis;

      //! Bitfield holding the validity of the position data.
      int ValidityMask;

   public:

      _DLL_ CCartesianPosition(const double X, const double Y, const double Z,
                               const double A, const double B, const double C,
                               const int S, const int T, const ml_CVector6 &E);
      _DLL_ CCartesianPosition(
         const double &X = DInvalid(), const double &Y = DInvalid(), const double &Z = DInvalid(),
         const double &A = DInvalid(), const double &B = DInvalid(), const double &C = DInvalid(),
         const int &S = IInvalid(), const int &T = IInvalid(),
         const double &E1 = DInvalid(), const double &E2 = DInvalid(),
         const double &E3 = DInvalid(), const double &E4 = DInvalid(),
         const double &E5 = DInvalid(), const double &E6 = DInvalid());

      //! \brief   Read/Write function for selected Cartesian components
      //!          (default is all components).
      //!
      //! \param   X             X coordinate in [mm]
      //! \param   Y             Y coordinate in [mm]
      //! \param   Z             Z coordinate in [mm]
      //! \param   A             Z orientation (ZYX-Euler angles) in [°]
      //! \param   B             Y orientation (ZYX-Euler angles) in [°]
      //! \param   C             X orientation (ZYX-Euler angles) in [°]
      //! \param   S             Status
      //! \param   T             Turn
      //! \param   E             Position values of external axes in [°] or [mm].
      //! \param   RequestedPos  Bit field of components to be read/written.
      //!
      //! \return  (Only read function) true, if every desired component is
      //!          valid and stored in X, Y, Z, A, B, C, S, T, and/or E, else false.
      //@{
      _DLL_ bool getFullPosition(double &X, double &Y, double &Z,
                           double &A, double &B, double &C,
                           int &S, int &T, ml_CVector6 &E,
                           const int RequestedPos = CART_POS_MASK | EXT_AXES_MASK) const;
      _DLL_ void setFullPosition(const double X, const double Y, const double Z,
                           const double A, const double B, const double C,
                           const int S, const int T, const ml_CVector6 &E,
                           const int RequestedPos = CART_POS_MASK | EXT_AXES_MASK);
      //@}

      //! \brief   Write functions for Cartesian parts.
      //@{
      _DLL_ void setFrame(const ml_CFrame &F);
      _DLL_ void setPosition(const ml_CVector3 &P);
      _DLL_ void setOrientationMatrix(const ml_COriMatrix &R);
      //@}

      //! \brief   Write functions for single Cartesian components.
      //@{
      _DLL_ void setX(const double X);
      _DLL_ void setY(const double Y);
      _DLL_ void setZ(const double Z);
      _DLL_ void setA(const double A);
      _DLL_ void setB(const double B);
      _DLL_ void setC(const double C);
      _DLL_ void setStatus(const int S);
      _DLL_ void setTurn(const int T);
      //@}

      //! \brief   Read functions for single Cartesian components.
      //@{
      _DLL_ bool getX(double &X) const;
      _DLL_ bool getY(double &Y) const;
      _DLL_ bool getZ(double &Z) const;
      _DLL_ bool getA(double &A) const;
      _DLL_ bool getB(double &B) const;
      _DLL_ bool getC(double &C) const;
      _DLL_ bool getStatus(int &S) const;
      _DLL_ bool getTurn(int &T) const;
      //@}

      //! \brief   Read functions for Cartesian parts.
       //@{
       _DLL_ bool getFrame(ml_CFrame &X) const;
       _DLL_ bool getPosition(ml_CVector3 &P) const;
       _DLL_ bool getOrientationMatrix(ml_COriMatrix &R) const;
       //@}

      //! \brief   Write functions for a single external axis.
      //!          The selected component is implicitly set valid.
      //!
      //! \param[in]  Axis    Index (beginning at 0) or name of required axis.
      //! \param[in]  Value   Position value to be setted in [°] or [mm].
      //!
      //! \return   None.
      //@{
      _DLL_ void setExtAxis(const int Axis, const double Value);
      _DLL_ void setExtAxis(const EExtAxisNames Axis, const double Value);
      //@}

      //! \brief   Read functions for a single external axis.
      //!          The selected component is implicitly set valid.
      //!
      //! \param[in]   Axis    Index (beginning at 0) or name of required axis.
      //! \param[out]  Value   Position value of required axis in [°] or [mm].
      //!
      //! \return  true, if the required axis was valid, else false.
      //@{
      _DLL_ bool getExtAxis(const int Axis, double &Value) const;
      _DLL_ bool getExtAxis(const EExtAxisNames Axis, double &Value) const;
      //@}

      //! \brief   Function for reading the bit field holding the validity of
      //!          the position data.
      //!
      //! \return  The above described bit field.
      _DLL_ int  getValidityMask() const;

      //! \brief   Function for removing a validity flag (default is to remove
      //!          all validities).
      //!
      //! \param[in]  RequestedPos  Bit field of components to be invalidated.
      //!
      //! \return     None.
      _DLL_ void clearValidityMask(const int RequestedPos = CART_POS_MASK | EXT_AXES_MASK);

      //! \brief   Function for writing the Position into a char-Buffer
      //!
      //! \param[in/out]  Buffer  Char-Field for at least [6 * ("X: %f,")] chars
      //!
      //! \return     None.
      _DLL_ void toBuffer(char* Buffer);
};


//! \brief   Position of a robot described by motor angles (drives)
class CMotorPosition
{
   private:

      //! Motor position in [°]:
      ml_CVector12 MotorAngles;

      //! Bitfield holding the validity of the position data.
      int ValidityMask;

   public:

      //! \brief   Default constructor. When called without arguments, every
      //!          component is set to zero. Every component ist set valid.
      _DLL_ CMotorPosition(const ml_CVector12& M);

      //! \brief   Special constructor expecting a number of motors to be
      //!          initialized and a pointer to a field containing the
      //!          desired values. The first NumberOfMotors many positions are
      //!          set valid, the rest is set invalid.
      _DLL_ CMotorPosition(const int NumberOfMotors, const double* ValuesOfMotors);

      //! \brief   Special constructor expecting a set of motor positions to be
      //!          assigned. Every component is set valid.
      _DLL_ CMotorPosition(
         const double &M1 = DInvalid(),  const double &M2 = DInvalid(),  const double &M3 = DInvalid(),
         const double &M4 = DInvalid(),  const double &M5 = DInvalid(),  const double &M6 = DInvalid(),
         const double &M7 = DInvalid(),  const double &M8 = DInvalid(),  const double &M9 = DInvalid(),
         const double &M10 = DInvalid(), const double &M11 = DInvalid(), const double &M12 = DInvalid() );

      //! \brief   Read/Write function for motor positions (default is all
      //!          motors).
      //!
      //! \param   M                Desired motor position(s).
      //! \param   RequestedMotors  Bit field of components to be read/written.
      //!
      //! \return  (Only read function) true, if every desired position is valid
      //!          and stored in M, else false.
      //@{
      _DLL_ bool getFullPosition(ml_CVector12& M, const int RequestedMotors = MOTOR_MASK) const;
      _DLL_ void setFullPosition(const ml_CVector12& M, const int RequestedMotors = MOTOR_MASK);
      //@}

      //! \brief   Read functions for single components.
      //!
      //! \param[in]   Motor  Index (starting at 0) or name of Motor.
      //! \param[out]  Value  Motor position in [°].
      //!
      //! \return   true, if the desired motor position was valid, else false.
      //@{
      _DLL_ bool getMotor(const int Motor, double& Value) const;
      _DLL_ bool getMotor(const EMotorNames Motor, double& Value) const;
      //@}

      //! \brief   Write functions for single components. The selected motor
      //!          is set valid.
      //!
      //! \param[in]  Motor  Index (starting at 0) or name of Motor.
      //! \param[in]  Value  Motor position in [°].
      //!
      //! \return   None.
      //@{
      _DLL_ void setMotor(const int Motor, const double Value);
      _DLL_ void setMotor(const EMotorNames Motor, const double Value);
      //@}

      //! \brief   Function for reading the bit field holding the validity of
      //!          the position data.
      //!
      //! \return  The above described bit field.
      _DLL_ int  getValidityMask() const;

      //! \brief   Function for removing a validity flag (default is to remove
      //!          all validities).
      //!
      //! \param[in]  RequestedMotors  Bit field of motors to be invalidated.
      //!
      //! \return     None.
      _DLL_ void clearValidityMask(const int RequestedMotors = MOTOR_MASK);
};

#endif  // !POSITION_DESCRIPTION_H
