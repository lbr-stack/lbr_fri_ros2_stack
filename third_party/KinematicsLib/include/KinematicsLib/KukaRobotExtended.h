#ifndef KUKA_ROBOT_EXTENDED_H
#define KUKA_ROBOT_EXTENDED_H
//! \cond KUKA_DOC_INTERN

// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2010 -- 2012
// KUKA Roboter GmbH, Germany. All Rights Reserved.


//! \file
//! \ingroup  KERNEL_DLL
//! \brief    Deklaration der CRobotExtended-Klasse. Nur für internen Gebrauch
//!           in der KUKA Labs GmbH.


/*<CL>
 *******************************************************************************
  Datum       Programmierer      Reviewer
              Beschreibung
 -------------------------------------------------------------------------------
  12.10.2012  Sonner
              Erstellung
  03.09.2015  Reichl            Rohmer, Heinig
              Methoden zur Jacobimatrixberechnung gewrappt
  09.10.2015  Marquart          Mueller-Sommer
              struct EKO_DAT sowie Methode zum auslesen der Eko-Daten hinzugefügt
  09.12.2015  Rohmer             Mueller-Sommer
              getMaxMotorTorque hinzugefügt
 *******************************************************************************
</CL>*/

#include "KukaRobotExpert.h"

class _DLL_ CRobotExtended;

class _DLL_ CDynamicModel
{
   friend class CRobotExtended;
public:
   CDynamicModel();
   ~CDynamicModel();

private:
   bool IsConfigured;
   void* pData;
};

//! \brief   Spezialklasse mit Sonderfunktionen für
//!          Transformation und Dynamikmodell
//!
//! Diese Klasse stellt Funktionen bereit, die aufgrund ihrer weitreichenden
//! Eingriffe in die Maschinendaten nur für den internen Gebrauch der KUKA Roboter
//! GmbH, speziell der Teams Bahnplanung, Bewegungsausführung, Mechatronik und
//! Lastdatenermittlung gedacht ist.

class _DLL_ CRobotExtended : public CRobotExpert
{
private:
   CRobotExtended& operator=(const CRobotExtended& Dummy);

public:
   //! \brief Konfiguraion eines eignenen Dynamikmodelldatensatzes mit Lastdaten
   //!
   //! \attention
   //! Bevor diese Routine angewendet werden kann, müssen die Maschinendaten
   //! geladen und das Dynamikmodel über den Aufruf von configureDynamicModel()
   //! konfiguriert worden sein.
   //!
   //! \param[out]     DynamicData  Konfigurierter Datensatz.
   //! \param[in] Mass         Masse am Flansch [kg]
   //! \param[in] CenterOfMass Lastschwerpunkt bzgl. Flansch [mm]
   //! \param[in] Inertia      Haupttraegheiten bzgl. Lastschwerpunkt
   //!                         (Ixx, Iyy, Izz) [kgm^2]
   //!
   //! \retval ROBOT_DYNAMIC_MODEL_NOT_CONFIGURED Dynamikmodell nicht konfiguriert
   //! \retval ROBOT_OK            Kein Fehler
   ROBOT_RETVAL configureDynamicData(CDynamicModel *DynamicData,
         double Mass, ml_CFrame CenterOfMass, ml_CVector3 Inertia);

   //! \brief Berechnung der abtriebsseitigen Motor- und Getriebmomente über
   //!        das inverse Dynamikmodell (Newton-Euler-Algorithmus).
   //!
   //! \attention
   //! Bevor diese Routine angewendet werden kann, müssen die Maschinendaten
   //! geladen und das Dynamikmodel über den Aufruf von configureDynamicModel()
   //! konfiguriert worden sein.
   //!
   //! \param[in]      DynamicData spezielle Dynamikkonfiguration
   //! \param[in]      q           geometrische Achsposition [deg] oder [mm].
   //! \param[in]      qd          geometrische Achsgeschwindigkeiten [deg/s] oder [mm/s].
   //! \param[in]      qdd         geometrische Achsbeschleunigungen [deg/s^2] oder [mm/s^2].
   //! \param[in]      CalcMode    Berechnungsmodus gemäß HFP_CALC_MODE.
   //! \param[out]     gearTorque  Getriebemomente um die Achsen am Abtrieb [Nm].
   //! \param[out]     motorTorque Motormomente um die Achsen am Abtrieb [Nm].
   //!
   //! \retval ROBOT_DYNAMIC_MODEL_NOT_CONFIGURED Dynamikmodell nicht konfiguriert
   //! \retval ROBOT_OK            Kein Fehler
   ROBOT_RETVAL calcInverseDynamics(CDynamicModel *DynamicData,
         const double *q, const double *qd, const double *qdd,
         double *gearTorque, double *motorTorque,
         int CalcMode = HFP_CALC_MOM | HFP_AD_FACTORS);

   //! \brief Berechnung der Inversdynamik in parameterlinearer Form für die
   //!        Lastdatenidentifikation.
   //!        Es gilt:
   /*! \f[
          \tau = Rest + Matrix * P, P = [m, m*[CM], J].
      \f] */
   //!        Dabei m*[CM] Schwerpunktvektor multipliziert mit der Masse und
   //!        J auf den Gelenkpunkt bezogen - Verschiebung um CM liefert
   //!        die Trägheit im Schwerpunktsystem.
   //!
   //! \attention
   //! Bevor diese Routine angewendet werden kann muessen die Maschinendaten
   //! geladen und das Dynamikmodel über den Aufruf von hfp_configureDynamicModel()
   //! konfiguriert worden sein.
   //!
   //! \param[in]      DynamicData spezielle Dynamikkonfiguration
   //! \param[in]      q           geometrische Achsposition [deg] oder [mm].
   //! \param[in]      qd          geometrische Achsgeschwindigkeiten [deg/s] oder [mm/s].
   //! \param[in]      qdd         geometrische Achsbeschleunigungen [deg/s^2] oder [mm/s^2].
   //! \param[in]      AxisToIdentify  Index der Achse für die die Starrkörperdaten linearisiert werden sollen.
   //!
   //! \param[out]     Rest        Anteil der nicht zu identifizierenden Achsen
   //! \param[out]     Matrix      12x10-Matrix der Linearisierung für AxisToIdentify!
   ROBOT_RETVAL calcInverseDynamicsParLin(CDynamicModel *DynamicData,
         const double *q, const double *qd, const double *qdd,
         double *Matrix, double *Rest, int AxisToIdentify);

   //! \brief Berechnung der direkten Dynamik (reines Starrkörpermodell abtriebsseitig).
   //!
   //! Ausgegeben werden die Komponenten M, C und (G + F_ext) aus
   /*! \f[
          \tau  = M(q) \ddot q + C(q, \dot q) + G(q) + F_{ext}.
      \f] */
   //! \attention
   //! Bevor diese Routine angewendet werden kann, müssen die Maschinendaten
   //! geladen und das Dynamikmodel über den Aufruf von configureDynamicModel()
   //! konfiguriert worden sein.
   //!
   //! \param[in]      DynamicData spezielle Dynamikkonfiguration
   //! \param[in]      q           geometrische Achsposition [deg] oder [mm].
   //! \param[in]      qd          geometrische Achsgeschwindigkeiten [deg/s] oder [mm/s].
   //! \param[in]      CalcMode    Auswahl der zu berechnenden Komponenten der Dynamikgleichung
   //! \param[out]     MassMatrix  Massenträgheitsmatrix als Vektor der Länge (MAX_ACHS^2)!!
   //!                             (wird spaltenweise belegt)
   //! \param[out]     Coriolis    Vektor der Corioliskräfte/momenten [N/Nm]
   //! \param[out]     Gravity     Vektor der statischen Anteile Gravitation + evtl. GWA [N/Nm]
   //!
   //! \retval ROBOT_DYNAMIC_MODEL_NOT_CONFIGURED Dynamikmodell nicht konfiguriert
   //! \retval ROBOT_OK            Kein Fehler
   ROBOT_RETVAL calcDirectDynamics(CDynamicModel *DynamicData,
         const double *q, const double *qd,
         double *MassMatrix, double *Coriolis, double *Gravity,
         int CalcMode =  HFP_CALC_STATIC | HFP_CALC_COR | HFP_CALC_MASS);

   //! \brief Berechnet zur gegebenen Gelenkstellung q die Jacobimatrix J(q) im entsprechenden Koordinatensystem. 
   //! \brief Dabei ist q in KernelDLL-Notation angegeben (Grad, Achsreihenfolge ROB_ACHS 1...6, EXT_ACHS 1...6).
   //! \brief Die Matrix ist wie folgt aufgebaut:<br>
   //! \brief                            ROB_ACHS 1  ...  ROB_ACHS 6      EXT_ACHS 1 ... EXT_ACHS 6
   //! \brief   Translationsänderung x        
   //! \brief   Translationsänderung y  
   //! \brief   Translationsänderung z 
   //! \brief   Rotationsänderung um x
   //! \brief   Rotationsänderung um y
   //! \brief   Rotationsänderung um z
   //! \brief
   //! \brief Die Achsreihenfolge entspricht der KernelDLL-Notation (MAX_ROB_ACHS Roboterachsen + MAX_EXT_ACHS Zusatzachsen), sofern nicht der 
   //! \brief Parameter "calculateKinematicChainJacobi == true" gesetzt wird.
   //! \param[in] DynamicData Dynamikkonfiguration
   //! \param[in] q[MAX_ACHS] Achsstellung des Roboters (in Grad), maximal MAX_ACHS Achsen
   //! \param[in] tool Toolframe mit Translation in Millimetern
   //! \param[out] J[6xMAX_ACHS] Jacobimatrix
   //! \param[in] toolToWorldJacobi Optional: Parameter zur Berechnung der Jacobimatrix im <b>Tool</b> Koordinatensystem (default ist: <b>Basissystem der dynamischen Kette</b> // false)
   //! \param[in] calculateKinematicChainJacobi Optional: Falls true entspricht die Achsreihenfolge der in Chaindata.xml angegebenen (default ist: false)
   //! \retval ROBOT_OK Alles OK
   //! \retval ROBOT_ERROR Nicht alles ok
   ROBOT_RETVAL calculateJacobianExtended(CDynamicModel *DynamicData, const double (&q)[MAX_ACHS], ml_CFrame &tool, double (&J)[6][MAX_ACHS], bool toolToBaseJacobi = false, bool calculateKinematicChainJacobi = false);

   //! \brief Berechnet die Jacobimatrix (Details siehe oben). Einziger Unterschied zur anderen calculateJacobian Methode: Es muss kein Toolframe übergeben werden,
   //! \brief das globale Toolframe wird verwendet.
   ROBOT_RETVAL calculateJacobianExtended(CDynamicModel *DynamicData, const double (&q)[MAX_ACHS], double (&J)[6][MAX_ACHS], bool toolToBaseJacobi = false, bool calculateKinematicChainJacobi = false);
   
   //! \brief   true, wenn mathematisch / mechanische Nullpunkt Verschiebung vorhanden ist.
   bool hasIndividualMames() const;
   
   //! \brief   Getter für mathematisch / mechanische Nullpunkt Verschiebung in Motorwinkeln
   //! @{ 
   ROBOT_RETVAL getMames(double (&fMamesMotorAngle)[MAX_ACHS]) const;
   //! @}

   //! \brief   True, wenn Roboter- oder externe Achsen mechanische Kopplung haben, sonst false.
   //! @{ 
   bool hasAxisCoupling() const;
   //! @}
   
   //! \brief   Getter für die Kopplungsmatrix der Roboterachsen.
   //! @{ 
   double getAxisCoupling(int i, int j) const;
   //! @}

   //! \brief   Getter für die Kopplungsmatrix der externen Achsen.
   //! @{ 
   double getAxisCouplingExt(int i, int j) const;
   //! @}

   //! \brief  Maschinendaten der Elastizitätskompensation (EKO)
   typedef struct EKO_DAT
   {
      double FilterLength;
      int BacklashLength;
      int SerialNumber;
      struct AxisData
      {
         bool EkoOn;
         double StaticFrictionArm;
         double ViscousFrictionArm;
         double SteepnessArm;
         double StiffnessConstant0;
         double StiffnessConstant1;
         double WidthSoft;
         double WidthSpline;
         double GearDamping;
         double KT;
         double Inertia;
         double StaticFrictionMotor;
         double ViscousFrictionMotor;
         double SteepnessMotor;
      } AxisData[MAX_ACHS];
      struct
      {
         bool AdaptionOn;
         double LowerBound;
         double UpperBound;
         double HighLoadFactor;
         double PolynomialCoefficients[16];
      } AdaptionData;
   };
   
   //! \brief  Getter für Eko-Daten; 
   //!         liefert die Eko-Daten, die aus der robcor.dat eingelesen wurden
   //! \attention  Diese Methode ist "subject to change", d.h. soll umgehend wieder entfernt werden.
   //!             Bitte nicht in neuen Implementiereungen referenzieren.
   //! \param[out] *EkoDat Zeiger auf ein EKO_DAT struct, das innerhalb der Methode gefüllt wird
   //! \param[out] ratMotAx[MAX_ACHS] Array mit den Getriebeübersetzungen, das innerhalb der Methode gefüllt wird
   void getEkoDat(EKO_DAT *EkoDat, double ratMotAx[MAX_ACHS]);

   //! \brief Konvertiert die Fehlernummer in einen Fehlertext
   static const char* getRobRetValErrString(ROBOT_RETVAL RobRetVal);

   //! \brief Gibt die Motormomentgrenzen zurück
   //!
   //! \attention
   //! Bevor diese Routine angewendet werden kann, müssen die Maschinendaten
   //! geladen und das Dynamikmodel über den Aufruf von configureDynamicModel()
   //! konfiguriert worden sein.
   //!
   //! \param[in]      DynamicData spezielle Dynamikkonfiguration
   //! \param[in]      qd          geometrische Achsgeschwindigkeiten [deg/s] oder [mm/s].
   //! \param[out]     MaMax       Motormomentgrenzen
   //!
   //! \retval ROBOT_DYNAMIC_MODEL_NOT_CONFIGURED Dynamikmodell nicht konfiguriert
   //! \retval ROBOT_OK            Kein Fehler
   ROBOT_RETVAL getMaxMotorTorque(CDynamicModel *DynamicData, const double qd[MAX_ACHS], double MaMax[MAX_ACHS]);
   
   //! \brief Achskopplung einrechnen.
   //!
   //! \param[in/out]  q           geometrische Achsposition [rad] oder [mm].
   //!
   //! \retval ROBOT_OK            Kein Fehler
   ROBOT_RETVAL addAxisLinkingInRad(double q[MAX_ACHS]);
   
   //! \brief Achskopplung rausrechnen.
   //!
   //! \param[in/out]  q           geometrische Achsposition [rad] oder [mm].
   //!
   //! \retval ROBOT_OK            Kein Fehler
   ROBOT_RETVAL subtractAxisLinkingInRad(double q[MAX_ACHS]);
   
};

#endif
