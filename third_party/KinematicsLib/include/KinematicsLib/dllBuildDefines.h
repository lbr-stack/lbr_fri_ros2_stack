#ifndef DLL_BUILD_DEFINES_H
#define DLL_BUILD_DEFINES_H

// This material is the exclusive property of KUKA Roboter GmbH and must be
// returned to KUKA Roboter GmbH immediately upon request.  This material and
// the information illustrated or contained herein may not be used, reproduced,
// stored in a retrieval system, or transmitted in whole or in part in any way;
// electronic, mechanical, photocopying, recording, or otherwise, without the
// prior written consent of KUKA Roboter GmbH.
//
// Copyright (C) 2010 -- 2011
// KUKA Roboter GmbH, Germany. All Rights Reserved


/*! \file
 *  \ingroup  KERNEL_DLL
 *  \brief    Definition des Makros \c _DLL_ (Definition der DLL-Schnittstellen)
 *
 *  Falls weder ein DLL-Import noch ein DLL-Export gemacht werden muss,
 *  wird \c _DLL_ als leer definiert. Wird die DLL erstellt, gilt
 *  \verbatim
       #define   _DLL_   __declspec(dllexport) \endverbatim
 *  und wird die DLL benützt, ist
 *  \verbatim
       #define   _DLL_   __declspec(dllimport) \endverbatim
 *  definiert.
 */

/*{{[CL]
********************************************************************************
 When        Programmer           Reviewer
             What
--------------------------------------------------------------------------------
 08.04.2010  Aurnhammer           none
             created
 02.11.2010  Schreittmiller       Burkhart
             - Datei von ml_ExternalInterface.h umbenannt in dllBuilddefines.h
             - Von GeneralServices/MathLib umgezogen nach include/exp
             - _MATHLIB_DLL_ umbenannt in _DLL_             
 18.02.2011  Schreittmiller      Burkhart
             Überarbeitung Doxygen-Dokumentare
 27.04.2015  Daniel Zicsi-Liess
             Labs Plattformkonsolidierung
********************************************************************************
[CL]}}
*/
#ifdef _MSC_VER                                                            
   //! Wir haben einen Visual-C-Compiler ...
   #ifdef MAT_FILE                                                          
      //! ... in Kombination mit Matlab. Dann können
      //! wir keine störende DLL-Kennung brauchen.
      #ifndef _NO_DLL_                                                      
         #define _NO_DLL_                                                    
      #endif                                                                
   #endif                                                                   
#else
   //! Wir haben keinen Visual-C-Compiler und wollen in diesem Fall nie und
   //! nimmer eine DLL-Kennung gesetzt haben.
   #ifndef _NO_DLL_  
      //! Keine DLL-Kennung benötigt
      #define _NO_DLL_                                                      
   #endif                                                                   
#endif


#undef _DLL_
#ifdef _NO_DLL_
   //! Keine DLL-Schnittstelle benötigt
   #define _DLL_
#else
   #ifdef _DLL_EXPORT_
      //! DLL wird erstellt
      #define   _DLL_   __declspec(dllexport)
   #else
      //! DLL wird benützt
      #define   _DLL_   __declspec(dllimport)
   #endif
#endif


#endif // DLL_BUILD_DEFINES_H
