#ifndef KUKA_ROBOT_EXPERT_H
#define KUKA_ROBOT_EXPERT_H

//! \cond KUKA_DOC_INTERN

// This material is the exclusive property of KUKA Roboter GmbH.
// Except as expressly permitted by separate agreement, this material may only
// be used by members of the development department of KUKA Roboter GmbH for
// internal development purposes of KUKA Roboter GmbH.
//
// Copyright (C) 2010 -- 2014
// KUKA Roboter GmbH, Germany. All Rights Reserved.


//! \file
//! \ingroup  KERNEL_DLL
//! \brief    Deklaration der CRobotExpert-Klasse. Nur f�r internen Gebrauch
//!           in der KUKA Roboter GmbH (R12-S, R12-M).


/*<CL>
*******************************************************************************
 Datum       Programmierer       Reviewer
             Beschreibung
-------------------------------------------------------------------------------
 05.10.2010  Burkhart/Schreittmiller
             Erstellung
 09.02.2011  Burkhart            Aurnhammer/Wiedemann/Deller
             Schreittmiller      Lebsack/Sturm
             KERNEL-DLL: Schnittstellenredesign; Multiinstanzf�higkeit Spline;
                         DLL-Integration in die KRC; Absolutgenauigkeit
 14.02.2011  Schreittmiller      Burkhart
             �berarbeitung Doxygen-Dokumentare; insbesondere ist diese Datei
             kein Bestandteil der Dokumentation f�r externe Anwender.
 23.02.2011  Aurnhammer          Wiedemann
             CRDB47088: Unstetigkeit im Reibmodell beseitigen
 10.03.2011  Aurnhammer          Burkhart
             Initialisierung des Dynamikmodell korrigiert
 07.03.2011  Deller              R�ssig/Wiedemann/Purrucker
             neue Methoden f�r die Absolutgenauigkeit (Erstellen eines "Null-Pid")
 27.05.2011  Aurnhammer          Burkhart
             Bessere Behandlung der �berschleifkriterien
 20.07.2011  Aurnhammer          Wiedemann
             Meldungshandling f�r Splinetestumgebung verbessert
 05.10.2011  Schreittmiller      _PtpSplToBeReviewedByMP_
             PTP-Spline Basisfunktionalit�t
 06.02.2012  Aurnhammer          _PtpSplToBeReviewedByMP_
             Versionierung verfeinert
 09.03.2012  Burkhart
             CRobotExpert Copy Konstruktor erstellt
 16.05.2012  Huettenhofer        Aurnhammer
             Automatische Berechnung von DH-Parametern aus der Transformation
 17.07.2012  Eisensehr           Burkhart, D�rndorfer
             Integration des GS-Maschinendatenladers in die KernelSystem-DLL.
 30.08.2012  Aurnhammer          Tronnier
             Umzug der MathLib nach BaseLib
 31.10.2012  Tronnier            Aurnhammer
             Nur Versionserh�hung durch:
             CRDB53782: "Schnittstelle KernelSystemDll ver�ndert"
 15.02.2013  Aurnhammer          Burkhart
             Dynamikmodell f�r Zusatzachsen im Spline
 15.04.2013  Aurnhammer          Wiedemann
             A4154: Kopplung externer Zusatzachsen im HFP/Dynamikmodell
 07.02.2014  Aurnhammer          Burkhart
             Neue Schnittstellen f�r $LENGTH_A/B und $TX3P3 eingef�hrt
 25.04.2014  Aurnhammer          Burkhart
             CRDB60051: DH-Parameter f�r das schiefe Portal
 15.05.2014  Aurnhammer          Boettner
             Schreib-/Lesefunktionen f�r Dynamikmodell erweitert
*******************************************************************************
</CL>*/


#include "KukaRobot.h"

//! \brief Versionsnummer: Verwendung als "xx.xx.xx" (z.B. 12.02.03 = 0x00120203)
#define KRC_EXPERT_VERSION_NUMBER 0x00150714

// Versionspr�fung
#if KRC_EXPERT_VERSION_NUMBER != KRC_ROBOT_VERSION_NUMBER
   #error KukaRobotExpert.h does not fit to KukaRobot.h
#endif

class CMaDaForRoboteam;

//! \brief   Spezialklasse mit Sonderfunktionen zur Manipulation von
//!          Maschinendaten vorbei am Maschinendatenlader.
//!
//! Diese Klasse stellt Funktionen bereit, die aufgrund ihrer weitreichenden
//! Eingriffe in die Maschinendaten nur f�r den internen Gebrauch der KUKA Roboter
//! GmbH, speziell der Teams Bahnplanung, Bewegungsausf�hrung, Mechatronik und
//! Lastdatenermittlung gedacht ist.

class _DLL_ CRobotExpert : public CRobot
{
private:
   CRobotExpert& operator=(const CRobotExpert& Dummy);

public:
   //! \brief Konstruktoren
   //! @{
   CRobotExpert();

   //! \attention  Folgenden Konstruktor mit besonderer Sorgfallt verwenden!
   //              Jedes CRobotExpert Objekt verschlinkt ca. 0,9 MB Speicher!
   //              (stets als Referenzen / Zeiger �bergeben)
   CRobotExpert(const CRobotExpert& RobotIn, const bool skipExtAxisAndRobRoot = false, const bool deactivatePalletizerTrafo = false);
   //! @}

   //! \brief   Zus�tzliche Schreib- und Lesfunktionen, um \b direkt die
   //!          Lastdaten der einzelnen Roboterkomponenten selbst (Karusel,
   //!          Schwinge, Arm, ...) in den internen Maschinendatenstrukturen zu
   //!          setzen bzw. zu lesen.
   //! @{
   ROBOT_RETVAL getLoadDataA1(double* Mass, double CenterOfMass[3], double Inertia[3]);
   ROBOT_RETVAL setLoadDataA1(double* Mass, double CenterOfMass[3], double Inertia[3], int* Error = NULL);
   ROBOT_RETVAL getLoadDataA2(double* Mass, double CenterOfMass[3], double Inertia[3]);
   ROBOT_RETVAL setLoadDataA2(double* Mass, double CenterOfMass[3], double Inertia[3], int* Error = NULL);
   ROBOT_RETVAL getLoadDataA3(double* Mass, double CenterOfMass[3], double Inertia[3]);
   ROBOT_RETVAL setLoadDataA3(double* Mass, double CenterOfMass[3], double Inertia[3], int* Error = NULL);
   ROBOT_RETVAL getLoadDataHand(double* Mass, double CenterOfMass[3], double Inertia[3]);
   ROBOT_RETVAL setLoadDataHand(double* Mass, double CenterOfMass[3], double Inertia[3], int* Error = NULL);
   ROBOT_RETVAL getLoadDataA5(double* Mass, double CenterOfMass[3], double Inertia[3]);
   ROBOT_RETVAL setLoadDataA5(double* Mass, double CenterOfMass[3], double Inertia[3], int* Error = NULL);
   //! @}

   //! \brief   Zus�tzliche Schreib- und Lesfunktionen, um \b direkt die
   //!          Lastdaten (Masse in [kg], Schwerpunklage in [m]) der einzelnen Links
   //!          des Dynamikmodells selbst zu setzen bzw. zu lesen.
   //! @{
   ROBOT_RETVAL resetLoadDataLink(int LinkIdx);
   ROBOT_RETVAL getLoadDataLink(int LinkIdx, double* Mass, double CenterOfMass[3], double Inertia[6]);
   ROBOT_RETVAL setDefaultLoadDataLink(int LinkIdx, double* Mass, ml_CFrame* CenterOfMass, ml_CVector3* Inertia);
   ROBOT_RETVAL getDefaultLoadDataLink(int LinkIdx, double* Mass, ml_CFrame* CenterOfMass, ml_CVector3* Inertia);
   ROBOT_RETVAL setRigidBodyLoadDataLink(int LinkIdx, double* Mass, ml_CFrame* CenterOfMass, ml_CVector3* Inertia);
   ROBOT_RETVAL getRigidBodyLoadDataLink(int LinkIdx, double* Mass, ml_CFrame* CenterOfMass, ml_CVector3* Inertia);
   //! @}


   //! \brief   Berechnung von Masse, Massenschwerpunkt und Traegheitstensor eines Koerpers
   //!          K1, der sich aus zwei Koerpern (K2 und K3) mit gegebenen Massen, Massenschwerpunkten
   //!          und Traegheitstensor zusammensetzt. Die Traegheitstensoren muessen bzgl. desselben
   //!          Koordinatensystems gegeben sein!
   //!
   //! \param[in]  body2m      Masse von K2 [kg]
   //! \param[in]  body2cm     Schwerpunktlage von K2 [m]
   //! \param[in]  body2I      Haupttr�gheiten von K2
   //! \param[in]  body3m      Masse von K3  [kg]
   //! \param[in]  body3cm     Schwerpunktlage von K3 [m]
   //! \param[in]  body3I      Haupttr�gheiten von K3
   //!
   //! \param[out]  body1m      Masse von K1
   //! \param[out]  body1cm     Schwerpunktlage von K1
   //! \param[out]  body1I      Haupttr�gheiten von K1
   ROBOT_RETVAL JoinBodies(      double *body1m,       double body1cm[3],       double body1I[6],
                           const double *body2m, const double body2cm[3], const double body2I[6],
                           const double *body3m, const double body3cm[3], const double body3I[6]);

   //! \brief   Direkt im Dynamikmodell max. Planungsmoment lesen/setzen
   //!          (ohne Konfiguration):
   //! @{
   ROBOT_RETVAL getMaxPlanningMotorTorque(double MaMax[MAX_ACHS]);
   ROBOT_RETVAL setMaxPlanningMotorTorque(double MaMax[MAX_ACHS]);
   //! @}

   //! \brief   Zugriffe direkt auf Madalader
   //!          p_robot->mas_dat.md_pptp.dynamik.geartorque:
   //!          (ohne Konfiguration):
   //! @{
   ROBOT_RETVAL getMaxGearTorque(double MgMax[MAX_ACHS]);
   ROBOT_RETVAL setMaxGearTorque(double MgMax[MAX_ACHS], int* Error = NULL);
   //! @}

   //! \brief   Zugriffe direkt auf Madalader
   //!          p_robot->mas_dat.md_pptp.dynamik.mottorque:
   //!          (ohne Konfiguration):
   //! @{
   ROBOT_RETVAL getMaxMotorTorque(double MaMax[MAX_ACHS]);
   ROBOT_RETVAL setMaxMotorTorque(double MaMax[MAX_ACHS], int* Error = NULL);
   //! @}

   //! \brief   Getter f�r Prozentfaktor fuer das Maximale Getriebemoment
   //! @{       fuer die Getriebemomentueberwachung  
   ROBOT_RETVAL getMaxGearTorqueFactor(double (&fMgMax)[MAX_ACHS]);
   //! @}


   //! \brief   Konfigurierte Kennline abtasten und durch mot_moment_fakt[i]
   //!          teilen, was ein 1:1-Projektierung gem. $ROBCOR.dat bzw.
   //!          Motor.xml-File liefert.
   //! @{
   ROBOT_RETVAL getMaxGearTorque(const double qd[MAX_ACHS], double MgMax[MAX_ACHS]);
   ROBOT_RETVAL getMaxMotorTorque(const double qd[MAX_ACHS], double MaMax[MAX_ACHS]);
   //! @}

   //! \brief   Crash-Energie lesen/setzen
   //! @{
   ROBOT_RETVAL getMaximalKineticEnergy(double CMax[MAX_ACHS]);
   ROBOT_RETVAL setMaxKineticEnergy(double CMax[MAX_ACHS], int* Error = NULL);
   //! @}

   //! \brief   Gewichtsausgleich lesen/setzen
   //! @{
   ROBOT_RETVAL getCBSModel(double* r, double* l, double* A,
         double* V0, double* P0, double* P1,
         double* SIGN, double* K0, double* kappa,
         double* C, double* F1);
   ROBOT_RETVAL setCBSModel(double* r, double* l, double* A,
         double* V0, double* P0, double* P1,
         double* SIGN, double* K0, double* kappa,
         double* C, double* F1, int* Error = NULL);
   //! @}

   //! \name  Wirkungsgrad(=Zahnflankenreibung) lesen/setzen
   //! @{
   //**************************************************************************
   //! \brief Lesen der Parametrierung des Wirkungsgrads im Dynamikmodell
   //!
   //! \param[out] Rz      Koeffizienten des Wirkungsgrads
   //**************************************************************************
   ROBOT_RETVAL getEfficiencyFactor(double Rz[MAX_ACHS]);

   //**************************************************************************
   //! \brief Lesen des Werts des Wirkungsgrads im Dynamikmodell
   //!        in Abh�ngigkeit von Achsgeschwindigkeit und Getriebemoment
   //!
   //! In der Berechnung des Antriebsstrangs im inversen Dynamikmodell
   //! wird das Getriebemoment mit einem Wirkungsgrad beaufschlagt. Damit
   //! ergibt sich das aus dem Getriebemoment resultierende Moment im
   //! Antriebsstrang als Mg = Rz * Mg = Rz(qd, Mg) * Mg.
   //!
   //! \param[in] qd      Achsgeschwindigkeit [deg/s] bzw. [mm/s]
   //! \param[in] Mg      Getriebemoment [Nm]
   //!
   //! \param[out] Rz     Wirkungsgrad bei Geschwindigkeit qd
   //!                    und Getriebemoment Mg
   //**************************************************************************
   ROBOT_RETVAL getEfficiencyFactor(const double qd[MAX_ACHS],
                                    const double Mg[MAX_ACHS],
                                          double Rz[MAX_ACHS]);

   //**************************************************************************
   //! \brief Holen der Parametrierung des Wirkungsgrads im Dynamikmodell
   //!
   //! \param[out] Rz      Koeffizienten des Wirkungsgrads
   //! \param[out] Error   Fehlerkennung HFP_MODEL_NOT_ACTIVE.
   //**************************************************************************
   ROBOT_RETVAL setEfficiencyFactor(double Rz[MAX_ACHS], int* Error = NULL);
   //! @}

   //! \name Haftreibung lesen
   //! @{
   //**************************************************************************
   //! \brief Setzen der Parametrierung der Haftreibung im Dynamikmodell
   //!
   //! \param[in] Rh   Wert der Haftreibung [Nm]
   //**************************************************************************
   ROBOT_RETVAL setStaticFriction(double Rh[MAX_ACHS], int* Error = NULL);

   //**************************************************************************
   //! \brief Holen der Parametrierung der Haftreibung im Dynamikmodell
   //!
   //! \param[out] Rh   Wert der Haftreibung [Nm]
   //**************************************************************************
   ROBOT_RETVAL getStaticFriction(double Rh[MAX_ACHS]);

   //**************************************************************************
   //! \brief Holen des Werts der Haftreibung im Dynamikmodell in Abh�ngigkeit
   //!        von der Achsgeschwindigkeit
   //!
   //! \param[in] qd      Achsgeschwindigkeit [deg/s] bzw. [mm/s]
   //!
   //! \param[out] Rh      Wert der Haftreibung bei Geschwindigkeit qd [Nm]
   //**************************************************************************
   ROBOT_RETVAL getStaticFriction(const double qd[MAX_ACHS], double Rh[MAX_ACHS]);
   //! @}

   //! \name Gleitreibung lesen
   //! @{
   ROBOT_RETVAL getViscoseFriction(double Rv[MAX_ACHS]);
   ROBOT_RETVAL setViscoseFriction(double Rv[MAX_ACHS], int* Error = NULL);
   //! @}

   //! \name   Motortr�gheiten lesen/setzen
   //! @{
   ROBOT_RETVAL getMotorInertia(double Ja[MAX_ACHS]);
   ROBOT_RETVAL setMotorInertia(double Ja[MAX_ACHS], int* Error = NULL);
   //! @}

   //! \brief   Rechnet die Achskopplung hinzu.
   //!
   //! \param[in, out]  q  Achsposition [deg] oder [mm].
   ROBOT_RETVAL addAxisLinking(double q[MAX_ACHS]);

   //! \brief   Rechnet die Achskopplung heraus.
   //!
   //! \param[in, out]  q  Achsposition [deg] oder [mm].
   ROBOT_RETVAL subtractAxisLinking(double q[MAX_ACHS]);

   //! \name Lastdatenermittlung
   //! @{
   ROBOT_RETVAL getLength56(double &L56);

   //! \brief   Holt die Achskopplungsfaktoren (vorw�rts) der Handachsen bei einer
   //!          Zentralhand wie am Standard-6-Achser.
   //!
   //! \param[out]  V     Vektor der Handachs-Kopplungsfaktoren:
   //!                    \li V[0]: A4 -> A6
   //!                    \li V[1]: A5 -> A6
   //!                    \li V[2]: A4 -> A5
   ROBOT_RETVAL getForwardCoupling(double V[3]);
   //! @}

   //! \brief   Holen der DH-Parametrierung (modifizierte Version nach Craig)
   //!          des Dynamikmodells
   //!
   //! Um die Angabe von Dynamik-Daten zu erleichtern werden alle Werte auf
   //! Denavit-Hardenberg (DH) Koordinatensysteme bezogen. Dazu wird am Gelenk
   //! zwischen zwei Achsen A(i) und A(i+1) ein Bezugssystem A(i)-DH angebracht,
   //! \li dessen Ursprung auf der Achse A(i) liegt, und zwar an dem (oder: einem)
   //!     Punkt mit Minimalabstand zur Achse A(i+1),
   //! \li dessen X-Achse entweder zum Punkt auf der Achse A(i+1) mit Minimalabstand
   //!     oder in die entgegengesetzte Richtung zeigt,
   //! \li und dessen Z-Achse in Richtung der Achse A(i) liegt.
   //! In einer DH-Modellierung werden jedes Gelenk durch zwei translatorische
   //! (a und d) und zwei rotatorische (alpha und theta) Parameter beschrieben,
   //! wobei jeweils der erste in X-Richtung und der zweite in Z-Richtung des gerade
   //! aktuellen Koordinatensystems wirkt.
   //! Jedem Parameter entspricht dann eine elementare Transformation mit der
   //! prinzipiellen Anwendungsreihenfolge �, a, alpha, d, theta, a, alpha, d, theta, ...,
   //! wobei d mit theta und a mit alpha kommutiert. Es gibt zwei Notationen, von denen die
   //! eine (in der KRC und hier verwendete) zuerst a und alpha, die andere zuerst d
   //! und theta anwendet.
   //! Um eine DH-Darstellung anwenden zu k�nnen, addiert das Dynamik-Modell intern zur
   //! in der KRC sichtbaren Achsstellung noch Achs-Offsets, sogenannte elDH-Parameter.
   //! Die elDH-Werte sind also die f�r theta (rotatorische Achsen) bzw. d (Linearachsen)
   //! einzusetzenden Werte, bei denen die �u�ere Nullstellung angenommen wird.
   //! Z.B. Am Standard-Roboter und am Scara-6-Achser gibt es einen solchen Offset nur f�r die
   //! Achse A3, deren Nullstellung entweder um +90deg oder -90deg verschoben werden muss, um eine
   //! g�ltige DH-Darstellung zu erhalten.
   //!
   //! Mit den Ausgangsdaten dieser Funktion l�sst sich auch eine Vorw�rtskinematik f�r
   //! alle Robotertypen aufbauen, die vom Dynamikmodell unterst�tzt werden. Nachdem sich
   //! aber �ber die DH-Parametrisierung des Dynamkimodells nicht alle Robotertypen vollst�ndig
   //! kinematisch abbilden lassen ist hier zus�tzlich das Frame TDH1RO (Achse 1 DH-System
   //! bez�glich $ROBROOT) notwendig. Bei einer Vorw�rtsrechnung ergibt sich dann f�r die
   //! Achse 6 in $ROBROOT die Transformationskette
   //!
   //!         TDH6RO = TDH1RO * DH1 * DH2 * ... * DH6.
   //!
   //! Ferner liegt das letzte DH-System nicht so orientiert wie in der Grundsystemtrafo,
   //! so dass noch ein Korrekturframe notwendig ist, das das letzte DH-System auf
   //! den Handwurzelpunkt abbildet
   //!
   //!         TWPRO = TDH1RO * DH1 * DH2 * ... * DH6 * TWPDHN.
   //!
   //! Um dann noch auf den Flansch des Grundsystems zu kommen muss nur noch
   //! das Anwenderframe $TFLWP aus R1/$machine.dat angewendet werden:
   //!
   //!         TFLRO = TDH1RO * DH1 * DH2 * ... * DH6 * TWPDHN * $TFLWP.
   //!
   //! \attention
   //! Um den Unterschied zwischen elektrisch und mechanisch Null bei den
   //! KUKA-Kinematiken zu ber�cksichtigen, gibt es den zus�tzlichen Parameteroffset
   //! elDH auf d bzw. theta.
   //!
   //! \param[out] a          Zeiger auf Vektor mit den a-DH-Parametern
   //!                        aller Roboterachsen [mm]
   //! \param[out] alpha      Zeiger auf Vektor mit den alpha-DH-Parametern
   //!                        aller Roboterachsen [deg]
   //! \param[out] d          Zeiger auf Vektor mit den d-DH-Parametern
   //!                        aller Roboterachsen [mm]
   //! \param[out] theta      Zeiger auf Vektor mit den theta-DH-Parametern
   //!                        aller Roboterachsen [deg]
   //! \param[out] elDH       Zeiger auf Vektor mit den elDH-DH-Parametern
   //!                        aller Roboterachsen [deg] oder [mm]
   //! \param[out] PDH1Ro     Zeiger auf den Versatz zwischen Roboterfuss (ROBROOT)
   //!                        und DH-System der Achse 1 (DH1) [mm]
   //! \param[out] RDH1Ro     Zeiger auf die Verdrehung zwischen Roboterfuss (ROBROOT)
   //!                        und DH-System der Achse 1 (DH1) [deg]
   //! \param[out] PWpDHn     Zeiger auf den Versatz zwischen DH-System der letzen Achse (DHn)
   //!                        und dem System im Handwurzelpunkt (WP) [mm]
   //! \param[out] RWpDHn     Zeiger auf die Verdrehung zwischen DH-System der letzen Achse (DHn)
   //!                        und dem System im Handwurzelpunkt (WP) [deg]
   //! \param[out] PFlWp      Zeiger auf den Versatz zwischen dem System im Handwurzelpunkt (WP)
   //!                        und dem System im Flansch (FL) ($TFLWP) [mm]
   //! \param[out] RFlWp      Zeiger auf die Verdrehung zwischen dem System im Handwurzelpunkt (WP)
   //!                        und System im Flansch (FL) ($TFLWP) [deg]
   //! \param[in] useDynamicModel  Merker, ob Dynamik-Modell verwendet werden soll. Default true.
   //!
   //! \retval   ROBOT_DYNAMIC_MODEL_NOT_CONFIGURED
   //! \retval   ROBOT_ERROR
   //! \retval   ROBOT_OK
   ROBOT_RETVAL getDenavitHartenbergParameters(double a[MAX_ROB_ACHS], double alpha[MAX_ROB_ACHS],
                            double d[MAX_ROB_ACHS], double theta[MAX_ROB_ACHS], double elDH[MAX_ROB_ACHS],
                            ml_CVector3 *PDH1Ro = NULL, ml_CEulerZYX *RDH1Ro = NULL,
                            ml_CVector3 *PWpDHn = NULL, ml_CEulerZYX *RWpDHn = NULL,
                            ml_CVector3 *PFlWp = NULL, ml_CEulerZYX *RFlWp = NULL,
                            bool useDynamicModel = true);

   // Methode die komplett verschwinden sollte !!!
   ROBOT_RETVAL setDenavitHartenbergParameters(double a[MAX_ROB_ACHS], double alpha[MAX_ROB_ACHS],
                                               double d[MAX_ROB_ACHS], double theta[MAX_ROB_ACHS],
                                               double elDH[MAX_ROB_ACHS]);

   //! \brief Lesen von $DEF_A4FIX
   //!
   //! \param[out]  defA4Fix  Wert von $DEF_A4FIX
   //!
   //! \retval     ROBOT_MADA_INVALID Maschinendaten ung�ltig.
   //! \retval     ROBOT_OK     Kein Fehler
   ROBOT_RETVAL getDefA4Fix(bool *defA4Fix);

   //! \brief Lesen von $DEF_A5LINK
   //!
   //! \param[out]  defA5Link  Wert von $DEF_A5LINK
   //!
   //! \retval     ROBOT_MADA_INVALID Maschinendaten ung�ltig.
   //! \retval     ROBOT_OK     Kein Fehler
   ROBOT_RETVAL getDefA5Link(bool *defA5Link);

   //! \brief Lesen von \$LENGTH_A
   //!
   //! \param[out]  lengthA  Wert von \$LENGTH_A
   //!
   //! \retval     ROBOT_MADA_INVALID Maschinendaten ung�ltig.
   //! \retval     ROBOT_INPUTS_INVALID  Ergebniszeiger ung�ltig
   //! \retval     ROBOT_OK     Kein Fehler
   ROBOT_RETVAL getLengthA(double *lengthA);

   //! \brief Lesen von \$LENGTH_B
   //!
   //! \param[out]  lengthB  Wert von \$LENGTH_B
   //!
   //! \retval     ROBOT_MADA_INVALID Maschinendaten ung�ltig.
   //! \retval     ROBOT_INPUTS_INVALID  Ergebniszeiger ung�ltig
   //! \retval     ROBOT_OK     Kein Fehler
   ROBOT_RETVAL getLengthB(double *lengthB);

   //! \brief Lesen von \$TX3P3.
   //!
   //! \attention
   //! Bevor diese Routine angewendet werden kann, m�ssen die Maschinendaten
   //! geladen worden sein.
   //!
   //! \param[out] TX3P3        Wert von \$TX3P3
   //!
   //! \retval     ROBOT_MADA_INVALID    Maschinendaten ung�ltig
   //! \retval     ROBOT_INPUTS_INVALID  Ergebniszeiger ung�ltig
   //! \retval     ROBOT_OK              Kein Fehler
   ROBOT_RETVAL getTX3P3(ml_CFrame* TX3P3);

   //! \brief Ausgabe des aktuell verwendeten Dynamikmodells in Datei "KR_DAT-1.h"
   //!
   //! Im Grundsystem kann �ber den Aufruf von printHfpData() im putty das Dynamikmodell
   //! im Vorlauf in eine Datei "KR_DAT-1.h". Diese Methode stellt den gleichen Mechanismus
   //! f�r CRobotExpert-Objekte zur Verf�gung.
   ROBOT_RETVAL printDynamicModel();

   //! \brief Erzeugen eines CRobot-Objekts aus Maschinendaten, welche in einem
   //!        CMaDaForRoboteam-Objekt abgelegt wurden.
   //!
   //! \param[in]  pMaDa  Spezielles Objekt f�r MaDa-Austausch bei Roboteam
   //!
   //!
   //! \retval     ROBOT_MADA_INVALID Maschinendaten ung�ltig.
   //! \retval     ROBOT_ERROR  Robcor- oder MachineDat-Objekt ung�ltig
   //! \retval     ROBOT_OK     Kein Fehler
   ROBOT_RETVAL assignMADA(CMaDaForRoboteam* pMaDa);

   //! \brief Erstellen eines XRob-ParamRepositorys ("Null-Pid")
   //!  unter Verwendung der geladenen Maschinendaten
   //!
   //! \param[out] paramRepository  Internals->pRsys->prb_RepositoryAccess
   //!
   //! \retval   ROBOT_ERROR
   //! \retval   ROBOT_OK
   //!
   //! Das absolutgenaue Modell wird nicht geladen!
   ROBOT_RETVAL createNewXRobParamRepository(void*& paramRepository);

   //! \brief Laden des absolutgenauen XRob-Modells
   bool reloadXRobModel();

   //! \brief Schreiben des XPids (Internals->pRsys->prb_RepositoryAccess)
   //!
   //! \param[in] serialNumber   Roboter-Seriennummer
   //! \param[in] pathStr        Pfad, in dem das XPid abgelegt wird
   ROBOT_RETVAL writeXPid(unsigned long serialNumber, char* pathStr);
};

//! \endcond

#endif // KUKA_ROBOT_EXPERT_H
