#ifndef GLOBAL_CONSTANTS_H
#define GLOBAL_CONSTANTS_H

// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2011-2012
// KUKA Roboter GmbH, Germany. All Rights Reserved.


//! \file
//! \ingroup KERNEL_DLL
//! \brief Several systemwide constants


//<CL>
//******************************************************************************
// Datum       Programmierer       Reviewer
//             Beschreibung
//------------------------------------------------------------------------------
// 06.02.2011  Aurnhammer
//             Erstellung
// 06.02.2012  Aurnhammer          _PtpSplToBeReviewedByMP_
//             Keine Abhängigkeiten zu KUKA-internen Headern in positionDescription.h
// 17.07.2012  Eisensehr           Burkhart, Dürndorfer
//             Integration des GS-Maschinendatenladers in die KernelSystem-DLL.
// 30.08.2012  Aurnhammer          Tronnier
//             Umzug der MathLib nach BaseLib
// 31.10.2012  Tronnier            Aurnhammer
//             Nur Versionserhöhung durch:
//             CRDB53782: "Schnittstelle KernelSystemDll verändert"
//******************************************************************************
//</CL>

//! \brief Versionsnummer: Verwendung als "xx.xx.xx" (z.B. 12.02.03 = 0x00120203)
#define KRC_GLOBAL_CONSTANTS_VERSION_NUMBER 0x00150714

// Konstanten
const int MAX_ROB_ACHS = 6;                             //!< maximale Anzahl der Roboterachsen in der KRC
const int MAX_EXT_ACHS = 6;                             //!< maximale Anzahl der Zusatzachsen in der KRC
const int MAX_ACHS     = (MAX_ROB_ACHS + MAX_EXT_ACHS); //!< maximale Anzahl der Achsen eines Robotersystems in der KRC

const int ALL_AXES_MASK = (1<<MAX_ACHS)-1;                //!< Bitmaske über alle adressierbaren Achsen in der KRC
const int ROB_AXES_MASK = (1<<MAX_ROB_ACHS)-1;            //!< Bitmaske über alle Roboterachsen in der KRC
const int EXT_AXES_MASK = ALL_AXES_MASK & ~ROB_AXES_MASK; //!< Bitmaske über alle Zusatzachsen in der KRC

#endif  // !GLOBAL_CONSTANTS_H
