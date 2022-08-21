/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD Core (Header)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
  **************************************************************************/

#pragma once

#include "MFDButtonPage.hpp"
#include "ShuttleFDOMFDButtons.h"
#include "LWP.h"
#include "DeorbitOpportunities.h"
#include "OrbMech.h"

using namespace OrbMech;

const double OMS_THRUST = 26700.0;
const double OMS_ISP0 = 316 * 9.80665;
const double RCS_THRUST = 7740.0;
const double RCS_ISP0 = OMS_ISP0;
const double LBM2KG = 0.45359237;
const double MPS2FPS = 3.280839895;
const unsigned MAXSECONDARIES = 4;

class OMPDefs
{
public:
	typedef enum { NOMAN, APSO, CIRC, DVPY, DVYP, EXDV, HA, HASH, LSDV, NOSH, PC, NC, NCC, NH, NHRD, NPC, NS, NSR, SOI, SOM, SOR, TPF, TPI, TPM } MANTYPE;
	typedef enum { NOTHR, THRES_APS, THRES_CAN, THRES_DLT, THRES_DT, THRES_DTL, THRES_M, THRES_REV, THRES_T, THRES_N, THRES_WT} THRESHOLD;
	typedef enum { NOSEC, A, ALT, APO, SEC_APS, ARG, ASC, CN, DEC, DSC, EL, LAT, LONG, N, NA, NP, OPT, P, PER, RAS, TGTA, TGTP, U, HD, DV, DVLV,
					LITI, LITM, LITO, NITI, NITM, NITO} SECONDARIES;
	typedef enum { NOTHRU, PX4, PX3, PX2, MXL, YL, MYL, ZH, ZL, MZH, MZL, M1, M2, OL, OR, OBP} THRUSTERS;
	typedef enum { NOGUID, M50, P7} GUID;
};

struct SecData
{
	char type[5];
	double value = 0.0;
};

struct TIGSecondaries
{
	OMPDefs::SECONDARIES type;
	double value = 0.0;
};

struct ManeuverConstraints
{
	char name[64];
	OMPDefs::MANTYPE type;
	OMPDefs::THRESHOLD threshold;
	double thresh_num;	//time, delta time or revs
	std::vector<SecData> secondaries;
};

struct ITERCONSTR
{
	int type = 0;	//type of iterator (1 = NC, 2 = NH, 3 = NPC)
	unsigned man = 0;	//maneuver that is applying the DV
	unsigned constr = 0;	//maneuver for which the constraint is applied
	double value = 0.0;		//Value of the constraint
};

struct MANEUVER
{
	char name[64];
	double TIG_GMT;
	VECTOR3 dV_LVLH;
	OMPDefs::MANTYPE type;
};

struct MANEVALDATA
{
	char type[5];
	char name[11];
	double DVMag;
	double GMTIG;
	double METIG;
	double DT;
	VECTOR3 DV;
	double HA;
	double HP;
	double DH;
	double RANGE;
	double PHASE;
	bool noon;
	double TTN;
	double Y;
	double Ydot;
	bool sunrise;
	double TTS;
};

struct MANTRANSDATA
{
	int MNVR;
	char NAME[5];
	char COMMENT[11];
	int SLOT;
	OMPDefs::THRUSTERS thrusters;
	OMPDefs::GUID guid;
	bool ITER;
	bool IMP;
	bool RREF;
	double ROLL;

};

struct MTTSLOTDATA
{
	int SLOT;
	OMPDefs::THRUSTERS thrusters;
	OMPDefs::GUID guid;
	//false = no, true = yes
	bool ITER;
	//false = IMP, true = OPT
	bool IMP;
	//false = ADI, true = TVR
	bool RREF;
	double ROLL;
};

struct DMTINPUT
{
	SV sv_tig;
	VECTOR3 DV_iner;
	double TV_ROLL;
	OMPDefs::THRUSTERS thrusters;
	char comment[11];
};

struct DetailedManeuverTable
{
	double GMTI;
	double PETI;
	double DV_M;
	char CODE[11];
	double TV_ROLL;
	double TRIMS_P;
	double TRIMS_LY;
	double TRIMS_RY;
	double WEIGHT;
	double TIG;
	double PEG4_C1;
	double PEG4_C2;
	double PEG4_HT;
	double PEG4_THETAT;
	double PEG4_PRPLT;
	VECTOR3 PEG7_DV;
	VECTOR3 BURN_ATT;
	double DVTOT;
	double TGO;
	VECTOR3 VGO;
	double TGT_HA;
	double TGT_HP;
};

class ShuttleFDOCore {
public:
	ShuttleFDOCore(VESSEL* v);
	~ShuttleFDOCore();

	void MinorCycle(double SimT, double SimDT, double mjd);
	int subThread();
	int startSubthread(int fcn);

	void CalcMCT();
	void CalculateManeuverEvalTable(SV sv_A0, SV sv_P0);
	void CalcLaunchTime();
	bool MET2MTT();
	void LoadMTTSlotData(MANTRANSDATA &man, int slot);
	void ExecuteMTT();
	void CalcDMT();
	void CalcDeorbitOpportunities();

	void AddManeuver(OMPDefs::MANTYPE type, char *name, unsigned ins = 0);
	void AddManeuverThreshold(unsigned num, OMPDefs::THRESHOLD type, double time);
	void AddManeuverSecondary(unsigned num, char *type, double value);
	void ModifyManeuver(unsigned num, OMPDefs::MANTYPE type, char *name);
	void ChangeMTTManeuverSlot(unsigned mnvr, int slot);

	void GetOPMManeuverType(char *buf, OMPDefs::MANTYPE type);
	void GetMTTThrusterType(char *buf, OMPDefs::THRUSTERS type);
	void GetDMTThrusterType(char *buf, OMPDefs::THRUSTERS type);
	void GetDMTManeuverID(char *buf, char *name);

	void SetLaunchMJD(int Y, int D, int H, int M, double S);
	double GETfromGMT(double GMT) { return GMT - LaunchGMT; }
	double GMTfromGET(double GET) { return GET + LaunchGMT; }

	SV StateVectorCalc(VESSEL *v, double SVGMT = 0.0);
	SV coast_auto(SV sv0, double dt);
	void ApsidesDeterminationSubroutine(SV sv0, SV &sv_a, SV &sv_p);
	void ApsidesMagnitudeDetermination(SV sv0, double &r_A, double &r_P);
	void ApsidesArgumentofLatitudeDetermination(SV sv0, double &u_x, double &u_y);
	SV PositionMatch(SV sv_A, SV sv_P);
	VECTOR3 LambertAuto(VECTOR3 RA, VECTOR3 VA, double GMT0, VECTOR3 RP_off, double dt, int N, bool prog);
	VECTOR3 SOIManeuver(SV sv_A, SV sv_P, double GMT1, double dt, VECTOR3 off);
	VECTOR3 SORManeuver(SV sv_A, SV sv_P, double GMT1, VECTOR3 off);
	VECTOR3 NPCManeuver(SV sv_A, VECTOR3 H_P);
	VECTOR3 CircManeuverAuto(SV sv_A);
	VECTOR3 NSRManeuver(SV sv_A, SV sv_P);
	SV timetoapo_auto(SV sv_A, double revs);
	SV AEG(SV sv0, int opt, double dval, double DN = 0.0);
	SV DeltaOrbitsAuto(SV sv0, double M);
	SV FindNthApsidalCrossingAuto(SV sv0, double N);
	double CalculateYDot(VECTOR3 V_A, VECTOR3 R_P, VECTOR3 V_P);
	SV PoweredFlightProcessor(SV sv_tig, VECTOR3 DV_iner, double f_T, double v_ex, bool nonspherical);
	SV FindOrbitalSunriseRelativeTime(SV sv0, bool sunrise, double dt1);
	SV FindOrbitalMidnightRelativeTime(SV sv0, bool midnight, double dt1);
	bool FindSVAtElevation(SV sv_A, SV sv_P, double t_guess, double elev_D, SV &sv_A2);
	VECTOR3 HeightManeuverAuto(SV sv_A, double r_D);

	double FindCommonNode(SV sv_A, SV sv_P, VECTOR3 &u_d);
	//Calculates the OMS trim gimbal angles as a function of the Shuttle CG (in inches), either parellel or through the CG
	void OMSTVC(VECTOR3 CG, bool parallel, double &P, double &LY, double &RY);

	double GetLaunchGMT() { return LaunchGMT; }

	std::vector<MANEUVER> ManeuverTable;
	std::vector<ManeuverConstraints> ManeuverConstraintsTable;
	std::vector<MANEVALDATA> ManeuverEvaluationTable;
	std::vector<MANTRANSDATA> ManeuverTransferTable;
	std::vector<DMTINPUT> DMTInputTable;
	MTTSLOTDATA MTTSlotData[10];
	DetailedManeuverTable DMT;

	unsigned DMT_MNVR;

	double InPlaneGMT;

	// SUBTHREAD MANAGEMENT
	HANDLE hThread;
	int subThreadMode;										// What should the subthread do?
	int subThreadStatus;									// 0 = done/not busy, 1 = busy, negative = done with error

	VESSEL* vessel;
	VESSEL* target;
	int targetnumber;
	VESSEL* shuttle;
	int shuttlenumber;

	//Displayed launch time
	//Year, Day, Hour, Minutes of launch
	int launchdate[4];
	//Seconds of launch
	double launchdateSec;

	bool useNonSphericalGravity;
	int OMPErrorCode;
	//false = vessel, true = LWP
	bool chaserSVOption;
	LWPSettings LWP_Settings;
	LWPSummary LPW_Summary;
	LWPParameterTable LWP_Parameters;
	//0 = manual, 1 = LC-39A, 2 = LC-39B
	int LWP_LaunchSite;
	//DTIG between MECO and ET SEP
	double DTIG_ET_SEP;
	//DTIG between MECO and MPS Dump
	double DTIG_MPS;
	//ET SEP DV
	VECTOR3 DV_ET_SEP;
	//MPS Dump DV
	VECTOR3 DV_MPS;
	double LWP_PlanarOpenGMT, LWP_PlanarCloseGMT;

	//Deorbit Opportunities
	LOPTOutput DODS_Output;
	double DOPS_GETS;
	double DOPS_GETF;
	int DOPS_InitialRev;
	double DOPS_MaxXRNG;

	OBJHANDLE hEarth;
	double mu;
protected:
	int CalculateOMPPlan();
	bool IsOMPConverged(ITERSTATE *iters, int size);
	void GetThrusterData(OMPDefs::THRUSTERS type, double &F, double &isp);

	//MJD at midnight before launch
	double BaseMJD;
	//GMT of launch
	double LaunchGMT;
	//Rotation matrix from TEG (true-equator and Greenwich meridian of date) to Ecliptic, left handed
	MATRIX3 M_EFTOECL_AT_EPOCH;
	double R_E;
	double w_E;

	SV sv_chaser, sv_target;

	LaunchWindowProcessor LWP;
};