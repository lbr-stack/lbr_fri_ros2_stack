// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2012
// KUKA Roboter GmbH, Germany. All Rights Reserved

//!\file
//!\brief \copybrief MotionDeviceHandler_Description \sa MotionDeviceHandler_Description
//<CL>
//*****************************************************************************
// Datum       Programmierer        Reviewer
//             Beschreibung
//------------------------------------------------------------------------------
// 01.03.2012  Sonner    
//             created
// 10.09.2013  Schreiber
//             Workaround - isDeviceKnown() eingefügt, um im Betrieb keine Deadlocks zu erzeugen
//*****************************************************************************
//</CL>

#ifndef _INCLUDE_GLOBAL_MOTION_OBJECTS_
#define _INCLUDE_GLOBAL_MOTION_OBJECTS_

#include <string>
#include <vector>

class CMotionDeviceData;
class CRobotExtended;

class IDeviceHandlerAccess
{
public:
   virtual ~IDeviceHandlerAccess() {};

   virtual bool readConfig(const std::string& ConfigName) = 0;

   virtual bool isDeviceKnown(const std::string& DeviceName)=0;

   virtual CMotionDeviceData* getDeviceData(const std::string& DeviceName) = 0;
   virtual CRobotExtended* getRobotObject(const std::string& DeviceName) = 0;
   virtual std::vector<std::string> getAllRobotNames() = 0;
   
};

// Helper-Methoden, um ohne-SystemManager-Modul-Rahmen den DeviceManager
// erzeugen, benutzen und zerstören zu können (Windows-OfficeLite oder UnitTests)
// TODO echten Singleton mit Referenzzählung daraus machen!
bool initGlobalDeviceHandler(bool useSharedMemory = false);
bool useGlobalDeviceHandler();
void cleanGlobalDeviceHandler();

extern IDeviceHandlerAccess *pGlobalDeviceHandler;


#endif

