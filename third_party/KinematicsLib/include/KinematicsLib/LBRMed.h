#ifndef _LBRMED_H_
#define _LBRMED_H_

//This material is the exclusive property of KUKA Roboter GmbH.
//Except as expressly permitted by separate agreement, this material may only
//be used by members of the development department of KUKA Roboter GmbH for
//internal development purposes of KUKA Roboter GmbH.
//
//Copyright (C) 2016
//KUKA Roboter GmbH, Germany. All Rights Reserved

//! \file LBRMed7kg.h
//! \brief Header of LBRMed7kg class.

//<CL>
//*****************************************************************************
// Datum       Programmierer        Reviewer
//             Beschreibung
//-----------------------------------------------------------------------------
// 31.10.2016  Finke                Reichl
//             Creation
//*****************************************************************************
//</CL>

#include "Device.h"

//! \brief Class to represent the LBR Med device.
class LBRMed : public Device
{
public:

   LBRMed(const EVariant variant);
   ~LBRMed();   
   
private:
   //lint -esym(1704, LBRMed::LBRMed) Hide the constructors that are not used
   LBRMed();
   LBRMed(const LBRMed&);
   LBRMed& operator=(const LBRMed&);
   
public:
   //! \brief Initialization of the device, e.g. reading the specific MADA configuration.
   bool init();
};

#endif // _LBRMED_H_
