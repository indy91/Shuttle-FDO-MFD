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
#include "DMP.h"
#include "OrbitalManeuverProcessor.h"
#include "OrbMech.h"

using namespace OrbMech;

const double OMS_THRUST = 26700.0;
const double OMS_ISP0 = 316 * 9.80665;
const double RCS_THRUST = 7740.0;
const double RCS_ISP0 = OMS_ISP0;

class FDODefs
{
public:
	typedef enum { NOTHRU, PX4, PX3, PX2, MXL, YL, MYL, ZH, ZL, MZH, MZL, M1, M2, OL, OR, OBP } THRUSTERS;
	typedef enum { NOGUID, M50, P7 } GUID;
};

struct MANTRANSDATA
{
	int MNVR;
	std::string NAME;
	std::string COMMENT;
	int SLOT;
	FDODefs::THRUSTERS thrusters;
	FDODefs::GUID guid;
	bool ITER;
	bool IMP;
	bool RREF;
	double ROLL;
};

struct MTTSLOTDATA
{
	int SLOT;
	FDODefs::THRUSTERS thrusters;
	FDODefs::GUID guid;
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
	FDODefs::THRUSTERS thrusters;
	std::string comment;
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
	void CalcLaunchTime();
	bool MET2MTT();
	void LoadMTTSlotData(MANTRANSDATA &man, int slot);
	bool ModifyMTTManeuverData(unsigned mnvr, const std::string& type, const std::string& value);
	void ExecuteMTT();
	void CalcDMT();
	void CalcDeorbitOpportunities();
	void CalcDMP();
	void CalcLTP();
	void ExportLTP();

	bool AddManeuver(char *type, char *name, unsigned ins = 0);
	void AddManeuverThreshold(unsigned num, OMP::OMPDefs::THRESHOLD type, double time);
	void AddManeuverSecondary(unsigned num, char *type, double value);
	void ModifyManeuver(unsigned num, OMP::OMPDefs::MANTYPE type, char *name);

	void ChangeMTTManeuverSlot(unsigned mnvr, int slot);

	void GetMTTThrusterType(char *buf, FDODefs::THRUSTERS type);
	FDODefs::THRUSTERS GetMTTThrusterType(const std::string &type) const;
	void GetMTTGuidanceType(char* buf, FDODefs::GUID type) const;
	FDODefs::GUID GetMTTGuidanceType(const std::string& type) const;

	void GetDMTThrusterType(char *buf, FDODefs::THRUSTERS type);
	void GetDMTManeuverID(char *buf, const char *name);

	void SetLaunchDay();
	void SetLaunchDay(int Y, int D);
	void SetLaunchTime(int H, int M, double S);
	double GETfromGMT(double GMT) { return GMT - sescnst.GMTLO; }
	double GMTfromGET(double GET) { return GET + sescnst.GMTLO; }

	SV StateVectorCalc(VESSEL *v, double SVGMT = 0.0);
	SV PoweredFlightProcessor(SV sv_tig, VECTOR3 DV_iner, double f_T, double v_ex, bool nonspherical);
	//Calculates the OMS trim gimbal angles as a function of the Shuttle CG (in inches), either parellel or through the CG
	void OMSTVC(VECTOR3 CG, bool parallel, double &P, double &LY, double &RY);

	OMP::ManeuverConstraintsTable MCT;
	OMP::MANEVALTABLE ManeuverEvaluationTable;
	std::vector<MANTRANSDATA> ManeuverTransferTable;
	std::vector<DMTINPUT> DMTInputTable;
	MTTSLOTDATA MTTSlotData[10];
	DetailedManeuverTable DMT;

	unsigned DMT_MNVR;

	// SUBTHREAD MANAGEMENT
	HANDLE hThread;
	int subThreadMode;										// What should the subthread do?
	int subThreadStatus;									// 0 = done/not busy, 1 = busy, negative = done with error

	VESSEL* vessel;
	VESSEL* target;
	int targetnumber;
	VESSEL* shuttle;
	int shuttlenumber;

	OrbMech::SessionConstants sescnst;

	bool useNonSphericalGravity;
	//false = vessel, true = LWP
	bool chaserSVOption;
	LWPSettings LWP_Settings;
	LWPOutput LWP_Output;
	LTPOutput LTP_Output;
	//0 = manual, 1 = LC-39A, 2 = LC-39B, 3 = SLC-6
	int LWP_LaunchSite;

	//Orbital Maneuver Processor
	OMP::OrbitalManeuverProcessor omp;
	std::string OMPErrorMessage;

	//Deorbit Opportunities
	double DOPS_GETS;
	double DOPS_GETF;
	int DOPS_InitialRev;
	double DOPS_MaxXRNG;
	bool DOPS_ConUS; //True = only use three main landing sites on the continental US, false = use full list
	LOPTOutput DODS_Output;
	int DOPS_Page, DOPS_MaxPage;

	//Deorbit Maneuver Planning
	DMPOptions DMPOpt;
	DMPResults DMPRes;
	std::string DMPLandingSite;

	int ErrorCode; //0 = no error, 1 = launch day not initialized

	OBJHANDLE hEarth;
	double mu;
protected:
	void CalculateOMPPlan();
	void GetThrusterData(FDODefs::THRUSTERS type, double &F, double &isp);
	void ReadDOPSLandingSiteData(std::vector<LOPTSite> &sites, bool FirstThreeSites) const;
	void ReadDMPLandingSiteData(std::vector<DMPSite> &sites) const;

	VECTOR3 TEG2M50(VECTOR3 v_TEG);
	MATRIX3 TEG_to_EF_Matrix(double gmt) const;

	SV sv_chaser;

	LaunchWindowProcessor LWP;
};