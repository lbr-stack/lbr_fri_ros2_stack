#ifndef _DEVICE_H_
#define _DEVICE_H_

//This material is the exclusive property of KUKA Roboter GmbH.
//Except as expressly permitted by separate agreement, this material may only
//be used by members of the development department of KUKA Roboter GmbH for
//internal development purposes of KUKA Roboter GmbH.
//
//Copyright (C) 2016
//KUKA Roboter GmbH, Germany. All Rights Reserved

//! \file Device.h
//! \brief Header of device class.

//<CL>
//*****************************************************************************
// Datum      Programmierer         Reviewer
//            Beschreibung
//-----------------------------------------------------------------------------
// 31.10.2016 Finke                 Reichl
//            Creation
//*****************************************************************************
//</CL>

#include "KukaRobotExtended.h"
#include "GlobalObjects.h"

//! \brief Base class to represent robot devices
class Device
{
public:
   
   //! \brief Supported robot types.
   enum EVariant
   {
      LBR7KG,
      LBR14KG
   };

   Device(const int numberOfAxes, const Device::EVariant variant);
   virtual ~Device();

private:
   //lint -esym(1704, Device::Device) Hide the constructors that are not used
   Device();
   Device(const Device&);
   Device& operator=(const Device&);
   
public:
   //! \brief Initialization of the device, e.g. reading MADA.
   //! \attention Must be implemented by child classes.
   virtual bool init();

   //! \brief Calculates Cartesian pose based on a joint configuration.
   //! \param[in]    axesInput   Joint configuration in degrees, 3. joint is redundancy angle.
   //! \param[out]   cartOutput  Cartesian pose.
   //! \param[in]    tool        Pointer to a tool frame (optional).
   //! \param[in]    base        Pointer to a base frame (optional).
   //! \retval true  Calculation was successful, false otherwise.
   bool getForward(const double axesInput[], CCartesianPosition& cartOutput, const ml_CFrame *tool, 
            const ml_CFrame *base) const;
   
   //! \brief Calculates joint configuration based on a Cartesian pose, stats, turn and redundany angle.
   //! \param[in]    cartInput      Cartesian pose.
   //! \param[out]   axesOutput     Joint configuration in degrees, 3. joint is redundancy angle.
   //! \param[in]    axesRefInput   Pointer to previous joint configuration in Grad, 3. joint is redundancy angle (optional).
   //! \param[in]    tool           Pointer to tool frame (optional).
   //! \param[in]    base           Pointer to base frame (optional).
   //! \retval true  Calculation was successful, false otherwise.
   bool getInverse(const CCartesianPosition& cartInput, double *axesOutput, const double *axesRefInput = NULL, 
            const ml_CFrame *tool = NULL, const ml_CFrame *base = NULL) const;

   //! \brief Returns the number of axes of the device.
   //! \retval Number of axes.
   int getNumberOfAxes() const;

   //! \brief Returns the device type.
   //! \retval device type.
   EVariant getRobotType() const;
   
protected:
   inline const char* getDeviceName(const EVariant variant) const
   {
      switch (variant)
      {
         case LBR7KG:  return "LBR_Med_7_R800_MF_SC_FLR";
         case LBR14KG: return "LBR_Med_14_R820_MF_SC_FLR";
         default:    return "INVALID";
      }
   }

   //! \brief Instance of the device.
   CRobotExtended* m_pRobot;

   //! \brief Flag whether initialization was successful
   bool m_isInitialized;

   //! \brief Number of axes of the device
   int m_numberOfAxes;

   //! \brief Device type.
   EVariant m_eVariant;
   
   //! \brief Converts joint configuration from Sunrise to KernelDll representation.
   void convertToKdll(CAxisPosition& out, const double in[]) const;
   
   //! \brief Converts joint configuration from KernelDll to Sunrise representation.
   //! \retval true, if the required axes were valid.
   bool convertToSunrise(double out[], const CAxisPosition in) const;
};

#endif // _DEVICE_H_
