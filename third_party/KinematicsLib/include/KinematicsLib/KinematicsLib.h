#ifndef _KINEMATICS_LIB_H_
#define _KINEMATICS_LIB_H_

//This material is the exclusive property of KUKA Roboter GmbH.
//Except as expressly permitted by separate agreement, this material may only
//be used by members of the development department of KUKA Roboter GmbH for
//internal development purposes of KUKA Roboter GmbH.
//
//Copyright (C) 2016, 2017
//KUKA Roboter GmbH, Germany. All Rights Reserved

//! \file KinematicsLib.h
//! \brief Header of kinematics lib class.

//<CL>
//*****************************************************************************
// Datum      Programmierer        Reviewer
//            Beschreibung
//-----------------------------------------------------------------------------
// 13.10.2016 Finke                Reichl
//            Creation
// 31.05.2017 Finke                Shahin
//            Refactoring due to memory problems
//*****************************************************************************
//</CL>

#include "Device.h"
#include "LBRMed.h"
#include "KinematicsLib.h"

//lint -esym(1754, operator*=)

//! \brief Main class of the Kinematics library providing the required interfaces to execute the
//!		   kinematic calculations!
#ifdef WIN32
#define _KINEMATICSLIB_DLL_ __declspec(dllexport)
extern "C" {
#else
#define _KINEMATICSLIB_DLL_ static
#endif

//! \brief Initialization of the device, e.g. reading MADA.
//! \param[in]	type	Robot type for which the kinematic calculations should be executed.
_KINEMATICSLIB_DLL_ bool init(Device::EVariant type);

//! \brief Calculates Cartesian pose based on a joint configuration.
//! \param[in]    axesInput   Joint configuration in degrees, 3. axis is redundancy angle.
//! \param[out]   cartOutput  Cartesian pose.
//! \param[out]   status      Status.
//! \param[out]   turn        Turn.	
//! \param[in]    tool        Pointer to a tool frame (optional).
//! \param[in]    base        Pointer to a base frame (optional).
//! \retval true  Calculation was successful, false otherwise.
_KINEMATICSLIB_DLL_ bool getForward(const double axesInput[], double *cartOutput, int *status, int *turn, const ml_CFrame *tool,
	const ml_CFrame *base);

//! \brief Calculates joint configuration based on a Cartesian pose, stats, turn and redundany angle.
//! \param[in]    cartInput      Cartesian pose.
//! \param[in]    status	        Status.
//! \param[in]    turn           Turn.
//! \param[out]   axesOutput     Joint configuration in degrees, 3. axis is redundancy angle.
//! \param[in]    axesRefInput   Pointer to previous joint configuration [°], 3. axis is redundancy angle (optional)
//! \param[in]    tool           Pointer to tool frame (optional).
//! \param[in]    base           Pointer to base frame (optional).
//! \retval true  Calculation was successful, false otherwise.
_KINEMATICSLIB_DLL_ bool getInverse(const double cartInput[], const int status, const int turn, double *axesOutput,
	const double *axesRefInput, const ml_CFrame *tool, const ml_CFrame *base);

//! \brief Returns the number of axes of the device.
//! \retval Number of axes.
_KINEMATICSLIB_DLL_ int getNumberOfAxes(void);

//! \brief Returns the device type.
//! \retval device type.
_KINEMATICSLIB_DLL_ Device::EVariant getRobotType(void);
#ifdef WIN32
}
#endif
#endif // _KINEMATICS_LIB_H_
