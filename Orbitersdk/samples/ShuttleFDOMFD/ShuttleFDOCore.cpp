/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD Core

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

#include "Orbitersdk.h"
#include "ShuttleFDOCore.h"

static DWORD WINAPI OMPMFD_Trampoline(LPVOID ptr) {
	ShuttleFDOCore *core = (ShuttleFDOCore *)ptr;
	return(core->subThread());
}

const double PITCH_BIAS = 15.82*RAD;
const double YAW_BIAS = 6.5*RAD;

ShuttleFDOCore::ShuttleFDOCore(VESSEL* v) :
	omp(sescnst)
{
	vessel = v;

	hEarth = oapiGetObjectByName("Earth");
	mu = GGRAV * oapiGetMass(hEarth);
	
	shuttlenumber = -1;
	shuttle = vessel;

	OBJHANDLE hShuttle = shuttle->GetHandle();
	for (unsigned i = 0;i < oapiGetVesselCount();i++)
	{
		if (hShuttle == oapiGetVesselByIndex(i))
		{
			shuttlenumber = i;
		}
	}

	targetnumber = -1;
	target = NULL;

	OBJHANDLE hTarget = oapiGetVesselByName("ISS");
	if (hTarget)
	{
		target = oapiGetVesselInterface(hTarget);
		for (unsigned i = 0;i < oapiGetVesselCount();i++)
		{
			if (hTarget == oapiGetVesselByIndex(i))
			{
				targetnumber = i;
			}
		}
	}

	//LoadPlanC();

	DMT_MNVR = 0;
	useNonSphericalGravity = vessel->NonsphericalGravityEnabled();
	chaserSVOption = false;

	subThreadMode = 0;
	subThreadStatus = 0;

	MTTSlotData[0].SLOT = 1;
	MTTSlotData[0].thrusters = FDODefs::THRUSTERS::OBP;
	MTTSlotData[0].guid = FDODefs::GUID::P7;
	MTTSlotData[0].ITER = false;
	MTTSlotData[0].IMP = true;
	MTTSlotData[0].RREF = true;
	MTTSlotData[0].ROLL = 0;

	MTTSlotData[1].SLOT = 2;
	MTTSlotData[1].thrusters = FDODefs::THRUSTERS::OBP;
	MTTSlotData[1].guid = FDODefs::GUID::P7;
	MTTSlotData[1].ITER = false;
	MTTSlotData[1].IMP = true;
	MTTSlotData[1].RREF = true;
	MTTSlotData[1].ROLL = PI;

	MTTSlotData[2].SLOT = 3;
	MTTSlotData[2].thrusters = FDODefs::THRUSTERS::OL;
	MTTSlotData[2].guid = FDODefs::GUID::P7;
	MTTSlotData[2].ITER = false;
	MTTSlotData[2].IMP = true;
	MTTSlotData[2].RREF = true;
	MTTSlotData[2].ROLL = 0;

	MTTSlotData[3].SLOT = 4;
	MTTSlotData[3].thrusters = FDODefs::THRUSTERS::OL;
	MTTSlotData[3].guid = FDODefs::GUID::P7;
	MTTSlotData[3].ITER = false;
	MTTSlotData[3].IMP = true;
	MTTSlotData[3].RREF = true;
	MTTSlotData[3].ROLL = PI;

	MTTSlotData[4].SLOT = 5;
	MTTSlotData[4].thrusters = FDODefs::THRUSTERS::OR;
	MTTSlotData[4].guid = FDODefs::GUID::P7;
	MTTSlotData[4].ITER = false;
	MTTSlotData[4].IMP = true;
	MTTSlotData[4].RREF = true;
	MTTSlotData[4].ROLL = 0;

	MTTSlotData[5].SLOT = 6;
	MTTSlotData[5].thrusters = FDODefs::THRUSTERS::OR;
	MTTSlotData[5].guid = FDODefs::GUID::P7;
	MTTSlotData[5].ITER = false;
	MTTSlotData[5].IMP = true;
	MTTSlotData[5].RREF = true;
	MTTSlotData[5].ROLL = PI;

	MTTSlotData[6].SLOT = 7;
	MTTSlotData[6].thrusters = FDODefs::THRUSTERS::PX2;
	MTTSlotData[6].guid = FDODefs::GUID::P7;
	MTTSlotData[6].ITER = false;
	MTTSlotData[6].IMP = true;
	MTTSlotData[6].RREF = true;
	MTTSlotData[6].ROLL = 0;

	MTTSlotData[7].SLOT = 8;
	MTTSlotData[7].thrusters = FDODefs::THRUSTERS::PX2;
	MTTSlotData[7].guid = FDODefs::GUID::P7;
	MTTSlotData[7].ITER = false;
	MTTSlotData[7].IMP = true;
	MTTSlotData[7].RREF = true;
	MTTSlotData[7].ROLL = PI;

	MTTSlotData[8].SLOT = 9;
	MTTSlotData[8].thrusters = FDODefs::THRUSTERS::PX2;
	MTTSlotData[8].guid = FDODefs::GUID::P7;
	MTTSlotData[8].ITER = false;
	MTTSlotData[8].IMP = false;
	MTTSlotData[8].RREF = true;
	MTTSlotData[8].ROLL = 0;

	MTTSlotData[9].SLOT = 10;
	MTTSlotData[9].thrusters = FDODefs::THRUSTERS::OL;
	MTTSlotData[9].guid = FDODefs::GUID::P7;
	MTTSlotData[9].ITER = false;
	MTTSlotData[9].IMP = false;
	MTTSlotData[9].RREF = true;
	MTTSlotData[9].ROLL = 0;

	DMT.BURN_ATT = _V(0, 0, 0);
	sprintf_s(DMT.CODE, "");
	DMT.DVTOT = 0.0;
	DMT.PEG4_C1 = 0.0;
	DMT.PEG4_C2 = 0.0;
	DMT.PEG4_HT = 0.0;
	DMT.PEG4_PRPLT = 0.0;
	DMT.PEG4_THETAT = 0.0;
	DMT.PEG7_DV = _V(0, 0, 0);
	DMT.TGO = 0.0;
	DMT.TGT_HA = 0.0;
	DMT.TGT_HP = 0.0;
	DMT.TIG = 0.0;
	DMT.TRIMS_LY = 0.0;
	DMT.TRIMS_P = 0.0;
	DMT.TRIMS_RY = 0.0;
	DMT.TV_ROLL = 0.0;
	DMT.VGO = _V(0, 0, 0);
	DMT.WEIGHT = 0.0;
	DMT.GMTI = 0.0;
	DMT.PETI = 0.0;
	DMT.DV_M = 0.0;

	LWP_Settings.NS = 0;
	LWP_Settings.TSTART = -5.0*60.0;
	LWP_Settings.TEND = 5.0*60.0;
	LWP_Settings.DTOPT = -(5.0*60.0 + 40.0);
	LWP_Settings.WRAP = 0;
	LWP_Settings.NEGTIV = 0;
	LWP_Settings.GAMINS = 0.6*RAD;
	LWP_Settings.LATLS = 28.6084030*RAD;
	LWP_Settings.LONGLS = -80.6232502*RAD;
	LWP_Settings.PFA = 14.4*RAD;
	LWP_Settings.PFT = 8.0*60.0 + 39.0;
	LWP_Settings.RINS = 21241700.0*0.3048;
	LWP_Settings.VINS = 25818.88*0.3048;//25928.0*0.3048;
	LWP_Settings.YSMAX = 14.0*RAD;
	LWP_Settings.DELNO = 0.0;
	LWP_Settings.CWHT = 251679.0*LBM2KG;
	LWP_Settings.OMS1.DTIG = 2.0*60.0;
	LWP_Settings.OMS1.HTGT = 120.0*1852.0;
	LWP_Settings.OMS1.THETA = 133.0*RAD;
	LWP_Settings.OMS1.C1 = 0.0;
	LWP_Settings.OMS1.C2 = 0.0;
	LWP_Settings.OMS2.DTIG = 29.0*60.0 + 18.0;
	LWP_Settings.OMS2.HTGT = 111.0*1852.0;
	LWP_Settings.OMS2.THETA = 315.0*RAD;
	LWP_Settings.OMS2.C1 = 0.0;
	LWP_Settings.OMS2.C2 = 0.0;
	LWP_Settings.DirectInsertion = true;

	LWP_LaunchSite = 1;

	DOPS_GETS = DOPS_GETF = 0.0;
	DOPS_InitialRev = 1;
	DOPS_MaxXRNG = 800.0;
	DOPS_ConUS = true;
	DOPS_Page = DOPS_MaxPage = 0;

	DMPLandingSite = "EDW22";

	ErrorCode = 0;

	// SUPERSIGHTER
	IDT[0].BuildInstrumentData("+X BODY (R,P)", 121, 0.0, 0.0, 0.0, 0.0, 360.0, 0.0, 180.0, false, identity());
	//IDT[1].BuildInstrumentData("-Z STAR TRACKER", 453, 0.0, 0.0, 0.0, -5.0, 5.0, -5.0, 5.0, true, _M(-0.0056491, 0.9994101, -0.0338744, 0.9894338, 0.0006786, -0.1449833, -0.1448747, -0.0343355, - 0.988854));
	//IDT[2].BuildInstrumentData("-Y STAR TRACKER", 453, 0.0, 0.0, 0.0, -5.0, 5.0, -5.0, 5.0, true, _M(-0.9662658, -0.1833851, 0.1808317, -0.1839513, 0.0, -0.9829353, 0.1802558, -0.9830411, -0.0337339));
	IDT[3].BuildInstrumentData("+X BODY (P,Y)", 231, 0.0, 0.0, 0.0, 0.0, 360.0, -90.0, 90.0, false, identity());
	IDT[12].BuildInstrumentData("-Z COAS", 243, 90.0, 90.0, 90.0, -10.0, 10.0, -10.0, 10.0, true, identity());

	IDT_Input_Num = 1;
	GTF_Input_Num = 1;
	// Ground Targets (for now just the STDN sites)
	GTF.Set(1, "Antigua", 17.137222, -61.775833, 0.0 / OrbMech::FPS2MPS); // TBD
	GTF.Set(2, "Ascension", -7.94354, -14.37105, 528.0 / OrbMech::FPS2MPS);
	GTF.Set(3, "Bermuda", 32.36864, -64.68563, -33.0 / OrbMech::FPS2MPS);
	GTF.Set(4, "Goldstone", 35.33820, -116.87421, 919.0 / OrbMech::FPS2MPS);
	GTF.Set(5, "Grand Bahama", 26.62022, -78.35825, 0.0 / OrbMech::FPS2MPS); // TBD
	GTF.Set(6, "Guam", 13.30929, 144.73694, 116.0 / OrbMech::FPS2MPS);
	GTF.Set(7, "Hawaii", 21.44719, -157.76307, 1139.0 / OrbMech::FPS2MPS);
	GTF.Set(8, "Madrid", 40.45443, -4.16990, 808.0 / OrbMech::FPS2MPS);
	GTF.Set(9, "Merritt", 28.40433, -80.60192, -55.0 / OrbMech::FPS2MPS);
	GTF.Set(10, "Santiago", -33.1489208, -70.6683031, 730.0 / OrbMech::FPS2MPS);
	GTF.Set(11, "Vandenberg", 34.74007, -120.61909, 0.0 / OrbMech::FPS2MPS); // TBD

	// Initialize celestial targets
	ReadStarCatalog(CTF);
	// Initialize ground targets

	CO_MON_Time = 0.0;
}

ShuttleFDOCore::~ShuttleFDOCore()
{

}

void ShuttleFDOCore::MinorCycle(double SimT, double SimDT, double mjd)
{

}

void ShuttleFDOCore::CalcMCT()
{
	startSubthread(1);
}

SV ShuttleFDOCore::StateVectorCalc(VESSEL *v, double SVGMT)
{
	VECTOR3 R, V;
	double dt;
	SV sv, sv1;

	v->GetRelativePos(hEarth, R);
	v->GetRelativeVel(hEarth, V);
	sv.GMT = (oapiGetSimMJD() - sescnst.GMTBASE)*24.0*3600.0;

	sv.R = _V(R.x, R.z, R.y);
	sv.V = _V(V.x, V.z, V.y);

	//Use TEG coordinate system
	sv.R = tmul(sescnst.M_TEG_TO_J2000, sv.R);
	sv.V = tmul(sescnst.M_TEG_TO_J2000, sv.V);

	sv.mass = v->GetMass();

	if (SVGMT != 0.0)
	{
		dt = SVGMT - sv.GMT;
		sv1 = coast(sv, dt);
	}
	else
	{
		sv1 = sv;
	}

	return sv1;
}

bool ShuttleFDOCore::AddManeuver(char* type, char* name, unsigned ins)
{
	OMP::ManeuverConstraints man;

	//Check if maneuver type is valid
	man.type = OMP::GetOPMManeuverType(type);
	if (man.type == OMP::OMPDefs::MANTYPE::NOMAN) return false;

	//Maneuver name/comment exceeds maximum?
	man.name = name;
	if (man.name.size() > OMP::MAXMANEUVERNAMELENGTH) return false;

	man.threshold = OMP::OMPDefs::THRESHOLD::NOTHR;
	man.thresh_num = 0.0;

	if (ins == 0 || ins == MCT.Table.size() + 1)
	{
		//Maximum number of maneuvers reached?
		if (MCT.Table.size() >= OMP::MAXMANEUVERS) return false;

		MCT.Table.push_back(man);
	}
	else
	{
		MCT.Table.insert(MCT.Table.begin() + ins - 1, man);
	}
	return true;
}

void ShuttleFDOCore::ModifyManeuver(unsigned num, OMP::OMPDefs::MANTYPE type, char *name)
{
	if (num >= 0 && num < MCT.Table.size())
	{
		MCT.Table[num].name.assign(name);
		MCT.Table[num].type = type;
		//ManeuverConstraintsTable[num].threshold = OMPDefs::THRESHOLD::NOTHR;
		//ManeuverConstraintsTable[num].thresh_num = 0.0;
		//ManeuverConstraintsTable[num].secondaries.clear();
	}
}

void ShuttleFDOCore::AddManeuverThreshold(unsigned num, OMP::OMPDefs::THRESHOLD type, double time)
{
	MCT.Table[num].threshold = type;
	MCT.Table[num].thresh_num = time;
}

void ShuttleFDOCore::AddManeuverSecondary(unsigned num, char *type, double value)
{
	OMP::SecData sec;

	OMP::OMPDefs::SECONDARIES typ = OMP::GetSecondaryType(type);
	if (typ == OMP::OMPDefs::SECONDARIES::NOSEC) return;

	sec.type = typ;
	sec.value = value;
	MCT.Table[num].secondaries.push_back(sec);
}

void ShuttleFDOCore::CalculateOMPPlan()
{
	OMPErrorMessage = "";

	if (shuttle == NULL)
	{
		OMPErrorMessage = "Error: select Shuttle vessel";
		return;
	}
	if (target == NULL)
	{
		OMPErrorMessage = "Error: select target vessel";
		return;
	}

	OMP::OMPInputs OMPIn;
	OMP::OMPOutputs OMPOut;

	if (chaserSVOption)
	{
		OMPIn.CHASER = sv_chaser;
		OMPIn.OMPChaserFile.assign("LWP");
	}
	else
	{
		OMPIn.CHASER = StateVectorCalc(shuttle);
		OMPIn.OMPChaserFile.assign(shuttle->GetName());
	}
	OMPIn.TARGET = StateVectorCalc(target);
	OMPIn.OMPTargetFile.assign(target->GetName());
	OMPIn.useNonSphericalGravity = useNonSphericalGravity;
	OMPIn.MCT = MCT;
	OMPIn.PRINT = false;
	OMPIn.DEBUG = false; // TBD

	// Run calculation
	omp.Calculate(OMPIn, OMPOut);

	OMPErrorMessage = OMPOut.ErrorMessage;

	ManeuverEvaluationTable = OMPOut.ManeuverEvaluationTable;

	char Buffer[256];
	for (unsigned i = 0; i < OMPOut.DebugOutput.size(); i++)
	{
		sprintf_s(Buffer, OMPOut.DebugOutput[i].c_str());
		oapiWriteLog(Buffer);
	}
}

void ShuttleFDOCore::CalcLaunchTime()
{
	startSubthread(2);
}

void ShuttleFDOCore::CalcDeorbitOpportunities()
{
	startSubthread(3);
}

void ShuttleFDOCore::CalcDMP()
{
	startSubthread(4);
}

void ShuttleFDOCore::CalcLTP()
{
	startSubthread(5);
}

void ShuttleFDOCore::ExportLTP()
{
	VECTOR3 IYD, IYD_OMS1, IYD_OMS2;
	double T_GMTLO_REF, T_ET_SEP;

	T_GMTLO_REF = LTP_Output.GMTLO + sescnst.DayOfYear * 24.0*3600.0;
	T_ET_SEP = LTP_Output.MET_MECO + 12.0;

	IYD = mul(sescnst.M_TEG_TO_M50, LTP_Output.IY_MECO);
	IYD_OMS1 = mul(sescnst.M_TEG_TO_M50, LTP_Output.IY_OMS1);
	IYD_OMS2 = mul(sescnst.M_TEG_TO_M50, LTP_Output.IY_OMS2);

	//LAUNCH TARGETING LOAD
	//T_GMTLO_REF, IY_MIN_EF, IYD, IYD_NOM, DELTA_PSI, DELTA_NODE_PHASE, T_GMTLO_PHASE

	//LAUNCH TARGETING LOAD OMS TGT
	//IYD_OMS1, IYD_OMS2, DTIG_OMS1, HTGT_OMS1, THETA_OMS1, C1_OMS1, C2_OMS1, DTIG_OMS2, HTGT_OMS2, THETA_OMS2, C1_OMS2, C2_OMS2

	std::ofstream file;

	file.open("FDOLaunchTargeting.txt");

	if (file.is_open() == false) return;

	char Buffer[256];

	sprintf_s(Buffer,"Shuttle launch targeting for %04d-%02d-%02d", sescnst.Year, sescnst.Month, sescnst.Day);
	file << Buffer << std::endl;
	OrbMech::MET2String(Buffer, LTP_Output.GMTLO);
	file << "GMTLO = " << Buffer << std::endl;
	file << std::endl;

	//Target vectors at T-9min and T-31sec
	OrbMech::SV sv_T_t9min, sv_T_t31sec, sv_T_t9min_J2000_lh, sv_T_t31sec_J2000_lh;
	double gmt_t9min, gmt_t31sec;

	gmt_t9min = LTP_Output.GMTLO - (9.0 * 60.0 + 0.1);
	gmt_t31sec = LTP_Output.GMTLO - 31.0;

	sv_T_t9min = OrbMech::coast(LTP_Output.sv_T, gmt_t9min - LTP_Output.sv_T.GMT);
	sv_T_t31sec = OrbMech::coast(LTP_Output.sv_T, gmt_t31sec - LTP_Output.sv_T.GMT);

	sv_T_t9min_J2000_lh = sv_T_t9min;
	sv_T_t31sec_J2000_lh = sv_T_t31sec;

	sv_T_t9min_J2000_lh.R = mul(sescnst.M_TEG_TO_J2000, sv_T_t9min.R);
	sv_T_t9min_J2000_lh.V = mul(sescnst.M_TEG_TO_J2000, sv_T_t9min.V);
	sv_T_t31sec_J2000_lh.R = mul(sescnst.M_TEG_TO_J2000, sv_T_t31sec.R);
	sv_T_t31sec_J2000_lh.V = mul(sescnst.M_TEG_TO_J2000, sv_T_t31sec.V);

	sv_T_t9min_J2000_lh.R = _V(sv_T_t9min_J2000_lh.R.x, sv_T_t9min_J2000_lh.R.z, sv_T_t9min_J2000_lh.R.y);
	sv_T_t9min_J2000_lh.V = _V(sv_T_t9min_J2000_lh.V.x, sv_T_t9min_J2000_lh.V.z, sv_T_t9min_J2000_lh.V.y);

	sv_T_t31sec_J2000_lh.R = _V(sv_T_t31sec_J2000_lh.R.x, sv_T_t31sec_J2000_lh.R.z, sv_T_t31sec_J2000_lh.R.y);
	sv_T_t31sec_J2000_lh.V = _V(sv_T_t31sec_J2000_lh.V.x, sv_T_t31sec_J2000_lh.V.z, sv_T_t31sec_J2000_lh.V.y);

	file << "Target state vector for scenario (T-9min):" << std::endl;
	sprintf_s(Buffer, "RPOS %lf %lf %lf", sv_T_t9min_J2000_lh.R.x, sv_T_t9min_J2000_lh.R.y, sv_T_t9min_J2000_lh.R.z);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "RVEL %lf %lf %lf", sv_T_t9min_J2000_lh.V.x, sv_T_t9min_J2000_lh.V.y, sv_T_t9min_J2000_lh.V.z);
	file << Buffer << std::endl;
	file << std::endl;

	file << "Target state vector for scenario (T-31sec):" << std::endl;
	sprintf_s(Buffer, "RPOS %lf %lf %lf", sv_T_t31sec_J2000_lh.R.x, sv_T_t31sec_J2000_lh.R.y, sv_T_t31sec_J2000_lh.R.z);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "RVEL %lf %lf %lf", sv_T_t31sec_J2000_lh.V.x, sv_T_t31sec_J2000_lh.V.y, sv_T_t31sec_J2000_lh.V.z);
	file << Buffer << std::endl;
	file << std::endl;

	file << "I-Loads:" << std::endl;
	file << "DELTA_NODE_PHASE = 0.013526" << std::endl; //TBD
	file << "DELTA_PSI = 0.0" << std::endl; //TBD
	file << "EFPSW = FALSE" << std::endl;
	file << "GAMD = " << LTP_Output.G_MECO * RAD << std::endl;
	file << "GAMDN = " << LTP_Output.G_MECO * RAD << std::endl;
	file << "IYD = " << IYD.x << " " << IYD.y << " " << IYD.z << std::endl;
	file << "IYD_NOM = " << IYD.x << " " << IYD.y << " " << IYD.z << std::endl;
	sprintf_s(Buffer, "IYD_OMS1 = %lf %lf %lf", IYD_OMS1.x, IYD_OMS1.y, IYD_OMS1.z);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "IYD_OMS2 = %lf %lf %lf", IYD_OMS2.x, IYD_OMS2.y, IYD_OMS2.z);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "DTIG_OMS1 = %lf", LTP_Output.TIG_MPS - T_ET_SEP);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "HTGT_OMS1 = %lf", LTP_Output.OMS1.HTGT / 0.3048);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "THETA_OMS1 = %lf", LTP_Output.OMS1.THETA);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "C1_OMS1 = %lf", LTP_Output.OMS1.C1 / 0.3048);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "C2_OMS1 = %lf", LTP_Output.OMS1.C2);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "DTIG_OMS2 = %lf", LTP_Output.TIG_OMS2 - T_ET_SEP);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "HTGT_OMS2 = %lf", LTP_Output.OMS2.HTGT / 0.3048);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "THETA_OMS2 = %lf", LTP_Output.OMS2.THETA);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "C1_OMS2 = %lf", LTP_Output.OMS2.C1 / 0.3048);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "C2_OMS2 = %lf", LTP_Output.OMS2.C2);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "RDMAG = %lf", LTP_Output.R_MECO * 1852.0 / 0.3048);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "RDNOM = %lf", LTP_Output.R_MECO * 1852.0 / 0.3048);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "T_GMTLO_PHASE = %lf", 365.0*24.0*3600.0);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "T_GMTLO_REF = %lf", T_GMTLO_REF);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "VDMAG = %lf", LTP_Output.V_MECO);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "VDNOM = %lf", LTP_Output.V_MECO);
	file << Buffer << std::endl;
	sprintf_s(Buffer, "NODE_SLOPE = 0.0");
	file << Buffer << std::endl;

	file.close();
}

int ShuttleFDOCore::subThread()
{
	ErrorCode = 0;

	//Do nothing if mission not initialized
	if (sescnst.GMTBASE == 0.0)
	{
		ErrorCode = 1;
		subThreadStatus = 0;
		if (hThread != NULL) { CloseHandle(hThread); }
		return(0);
	}

	int Result = 0;

	subThreadStatus = 2; // Running
	switch (subThreadMode) {
	case 0: // Test
		Sleep(5000); // Waste 5 seconds
		Result = 0;  // Success (negative = error)
		break;
	case 1: //Maneuver Plan
	{
		CalculateOMPPlan();

		Result = 0;
	}
	break;
	case 2: //Launch Window Processor
	{
		SV sv_T;

		sv_T = StateVectorCalc(target);

		if (useNonSphericalGravity)
		{
			LWP_Settings.SVPROP = 1;
		}
		else
		{
			LWP_Settings.SVPROP = 0;
		}
		LWP_Settings.TPLANE = sv_T.GMT;
		LWP_Settings.TRGVEC = sv_T;
		LWP_Settings.LOT = 6;
		LWP_Settings.LW = 2;
		LWP_Settings.lwp_table = &LWP_Output;

		LWP.Init(LWP_Settings);
		LWP.LWP();

		LWP_Settings.GMTLOR = LWP_Output.GMTOPT;

		Result = 0;
	}
	break;
	case 3: //Deorbit Opportunities
	{
		LandingOpportunitiesProcessor lop;
		LOPTInput opt;

		ReadDOPSLandingSiteData(opt.sites, DOPS_ConUS);

		if (opt.sites.size() > 0)
		{
			opt.sv_in = StateVectorCalc(vessel);
			opt.GETS = DOPS_GETS;
			opt.GETF = DOPS_GETF;
			opt.INORB = DOPS_InitialRev;
			opt.SVPROP = useNonSphericalGravity;
			opt.GMTR = sescnst.GMTLO;
			opt.XRNG = DOPS_MaxXRNG;
			opt.BaseMJD = sescnst.GMTBASE;
			opt.RM = sescnst.M_TEG_TO_M50;

			lop.LOPT(opt, DODS_Output);

			DOPS_Page = DOPS_MaxPage = 0;
			if (DODS_Output.data.size() > 0)
			{
				DOPS_MaxPage = (DODS_Output.data.size() - 1) / 25;
			}
		}
	}
	break;
	case 4: //Deorbit Maneuver Planning
	{
		DMPOptions opt2 = DMPOpt;

		if (opt2.ITIGFR == 0)
		{
			opt2.TIG += sescnst.GMTLO;
			opt2.TTHRSH = 0.0;
		}
		else
		{
			opt2.TIG = 0.0;
			opt2.TTHRSH += sescnst.GMTLO;
		}

		opt2.INTEGF = useNonSphericalGravity;

		SV sv = StateVectorCalc(vessel);

		opt2.TIMEC = sv.GMT;

		opt2.XYZI = sv.R;
		opt2.XYZID = sv.V;
		opt2.CD = 2.0;
		opt2.AREA = 0.0;
		opt2.WT = sv.mass;

		std::vector<DMPSite> sites;
		ReadDMPLandingSiteData(sites);

		bool found = false;
		unsigned i;
		for (i = 0; i < sites.size(); i++)
		{
			if (sites[i].name == DMPLandingSite)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			DMPRes.ErrorMessage = "DTM: Landing site not in table (Error)";
			Result = 0;
			break;
		}

		opt2.TLATD = sites[i].Lat;
		opt2.TLONG = sites[i].Lng;
		opt2.TALTD = sites[i].Alt;
		opt2.RAZ = sites[i].Azi;

		if (opt2.WCGOMS != 0.0)
		{
			opt2.IFUEL = 1;
		}
		else
		{
			opt2.IFUEL = 2;
		}
		opt2.IPOUT = 0; //Debug option

		DMP dmp;

		dmp.Executive(opt2, DMPRes);

		if (DMPRes.ErrorCode == 0)
		{
			DMPRes.Site = DMPLandingSite;
			DMPRes.TIG -= sescnst.GMTLO;

			//Calculate MM304 attitude
			SV sv_MM304;
			MATRIX3 Rot;
			VECTOR3 R, V;
			
			sv_MM304 = coast_auto(DMPRes.sv_EI, -5.0*60.0, useNonSphericalGravity);
			R = TEG2M50(sv_MM304.R);
			V = TEG2M50(sv_MM304.V);

			Rot = _M(cos(40.0*RAD), 0.0, sin(40.0*RAD), 0.0, 1.0, 0.0, -sin(40.0*RAD), 0.0, cos(40.0*RAD));

			VECTOR3 z_unit = -unit(R);
			VECTOR3 y_unit = unit(crossp(V, R));
			VECTOR3 x_unit = unit(crossp(y_unit, z_unit));

			MATRIX3 Rot2 = _M(x_unit.x, x_unit.y, x_unit.z,
				y_unit.x, y_unit.y, y_unit.z,
				z_unit.x, z_unit.y, z_unit.z);

			MATRIX3 LVLHMatrix = tmat(Rot2);
			MATRIX3 M50Matrix = mul(LVLHMatrix, Rot);

			DMPRes.EIminus5Att.x = atan2(-M50Matrix.m32, M50Matrix.m22);
			DMPRes.EIminus5Att.y = atan2(-M50Matrix.m31, M50Matrix.m11);
			DMPRes.EIminus5Att.z = asin(M50Matrix.m21);

			for (int i = 0; i < 3; i++)
			{
				if (DMPRes.EIminus5Att.data[i] < 0.0)
				{
					DMPRes.EIminus5Att.data[i] += PI2;
				}
			}
			DMPRes.EIminus5Att *= DEG;
		}
	}
	break;
	case 5: //Launch Targeting Processor
	{
		SV sv_T;

		sv_T = StateVectorCalc(target);

		if (useNonSphericalGravity)
		{
			LWP_Settings.SVPROP = 1;
		}
		else
		{
			LWP_Settings.SVPROP = 0;
		}
		LWP_Settings.TRGVEC = sv_T;
		LWP_Settings.LW = 1;
		LWP_Settings.LOT = 1;
		LWP_Settings.DELNO = 0.0;
		LWP_Settings.ltp_table = &LTP_Output;

		LWP.Init(LWP_Settings);
		LWP.LWP();

		//Post insertion state vector processing
		//Store SV
		sv_chaser = LWP.LWPSV.sv_P_MPS_Dump;
		chaserSVOption = true;

		//Save launch time
		int hh, mm;
		double ss;

		OrbMech::days2hms(LWP_Settings.GMTLOR / 24.0 / 3600.0, hh, mm, ss);
		SetLaunchTime(hh, mm, ss);

		Result = 0;
	}
	break;
	case 6: // Supersighter
	{
		if (shuttle == NULL)
		{
			Result = 0;
			break;
		}

		Supersighter ss;

		SSInputs.IDT = IDT;
		SSInputs.CTF = &CTF;
		SSInputs.GTF = &GTF;
		SSInputs.sescnst = &sescnst;
		SSInputs.sv0 = StateVectorCalc(shuttle);
		if (target)
		{
			SSInputs.sv_T = StateVectorCalc(target);
		}
		else
		{
			SSInputs.sv_T.GMT = 0.0; // Signals Supersighter that target is invalid
		}
		SSInputs.useNonSphericalGravity = useNonSphericalGravity;

		ss.RUN(SSInputs, SSOutputs);

		Result = 0;
	}
	break;
	case 7: // Checkout Monitor
	{
		if (shuttle == NULL)
		{
			Result = 0;
			break;
		}
		OrbMech::SV sv1, sv2;
		double dt;
		
		sv1 = StateVectorCalc(shuttle);
		dt = GMTfromGET(CO_MON_Time) - sv1.GMT;
		sv2 = coast_auto(sv1, dt, useNonSphericalGravity);

		CheckoutMonitor cm(sescnst);

		cm.RUN(sv2, useNonSphericalGravity, CO_DISP);

		Result = 0;
	}
	break;
	}

	subThreadStatus = Result;
	if (hThread != NULL) { CloseHandle(hThread); }

	return(0);
}

int ShuttleFDOCore::startSubthread(int fcn) {
	if (subThreadStatus < 1) {
		// Punt thread
		subThreadMode = fcn;
		subThreadStatus = 1; // Busy
		DWORD id = 0;
		hThread = CreateThread(NULL, 0, OMPMFD_Trampoline, this, 0, &id);
	}
	else {
		//Kill thread
		DWORD exitcode = 0;
		if (TerminateThread(hThread, exitcode))
		{
			subThreadStatus = 0;
			if (hThread != NULL) { CloseHandle(hThread); }
		}
		return(-1);
	}
	return(0);
}

void ShuttleFDOCore::ReadDOPSLandingSiteData(std::vector<LOPTSite> &sites, bool FirstThreeSites) const
{
	sites.clear();

	std::ifstream myfile;
	myfile.open(".\\Config\\MFD\\ShuttleFDOMFD\\DOPSLandingSites.txt");
	if (myfile.is_open())
	{
		char Buffer[128];
		LOPTSite temp;

		std::string line;
		while (std::getline(myfile, line))
		{
			if (sscanf(line.c_str(), "%s %lf %lf %lf %d", Buffer, &temp.lat, &temp.lng, &temp.rad, &temp.timezone) == 5)
			{
				temp.name.assign(Buffer);
				temp.lat *= RAD;
				temp.lng *= RAD;
				sites.push_back(temp);
				if (FirstThreeSites && sites.size() >= 3) break;
			}
		}
	}
}

void ShuttleFDOCore::ReadDMPLandingSiteData(std::vector<DMPSite> &sites) const
{
	sites.clear();

	std::ifstream myfile;
	myfile.open(".\\Config\\MFD\\ShuttleFDOMFD\\LandingSites.txt");
	if (myfile.is_open())
	{
		char Buffer[128];
		DMPSite temp;

		std::string line;
		while (std::getline(myfile, line))
		{
			if (sscanf(line.c_str(), "%s %lf %lf %lf %lf", Buffer, &temp.Lat, &temp.Lng, &temp.Alt, &temp.Azi) == 5)
			{
				temp.name.assign(Buffer);
				temp.Lat *= RAD;
				temp.Lng *= RAD;
				temp.Azi *= RAD;
				sites.push_back(temp);
			}
		}
	}
}

void ShuttleFDOCore::ReadStarCatalog(CelestialTargetFile& file) const
{
	std::ifstream myfile;
	CelestialTargetFileEntry temp;
	double RA, DEC;
	int num;

	// Null all data
	for (int i = 0; i < 400; i++)
	{
		file.stars[i].MAG = 0.0;
		sprintf_s(file.stars[i].Name, "");
		file.stars[i].u_vec = _V(0, 0, 1);
	}

	myfile.open(".\\Config\\MFD\\ShuttleFDOMFD\\StarCatalog.txt");

	if (myfile.is_open() == false) return;

	std::string line;
	while (std::getline(myfile, line))
	{
		if (sscanf_s(line.c_str(), "%d;%[^;];%lf;%lf;%lf", &num, temp.Name, 31, &RA, &DEC, &temp.MAG) == 5)
		{
			temp.u_vec = OrbMech::r_from_latlong(DEC * RAD, RA * RAD);
			if (num < 1 || num > 400) continue;
			file.stars[num - 1] = temp;
		}
	}

	myfile.close();
}

bool ShuttleFDOCore::MET2MTT()
{
	if (ManeuverEvaluationTable.Maneuvers.size() < 1) return false;

	ManeuverTransferTable.clear();

	MANTRANSDATA man;

	for (unsigned i = 0;i < ManeuverEvaluationTable.Maneuvers.size();i++)
	{
		man.MNVR = i + 1;
		man.NAME = ManeuverEvaluationTable.Maneuvers[i].type;
		man.COMMENT = ManeuverEvaluationTable.Maneuvers[i].name;
		LoadMTTSlotData(man, 1);

		ManeuverTransferTable.push_back(man);
	}
	return true;
}

void ShuttleFDOCore::LoadMTTSlotData(MANTRANSDATA &man, int slot)
{
	if (slot < 1 || slot > 10) return;

	man.SLOT = slot;
	man.thrusters = MTTSlotData[slot - 1].thrusters;
	man.guid = MTTSlotData[slot - 1].guid;
	man.ITER = MTTSlotData[slot - 1].ITER;
	man.IMP = MTTSlotData[slot - 1].IMP;
	man.RREF = MTTSlotData[slot - 1].RREF;
	man.ROLL = MTTSlotData[slot - 1].ROLL;
}

void ShuttleFDOCore::ChangeMTTManeuverSlot(unsigned mnvr, int slot)
{
	LoadMTTSlotData(ManeuverTransferTable[mnvr], slot);
}

bool ShuttleFDOCore::ModifyMTTManeuverData(unsigned mnvr, const std::string& type, const std::string& value)
{
	if (mnvr < 1 || mnvr > ManeuverTransferTable.size()) return false;

	MANTRANSDATA* data = &ManeuverTransferTable[mnvr - 1];

	if (type == "THR")
	{
		FDODefs::THRUSTERS thr = GetMTTThrusterType(value);
		if (thr == FDODefs::THRUSTERS::NOTHRU) return false;

		data->SLOT = 0;
		data->thrusters = thr;
		return true;
	}
	else if (type == "GUID")
	{
		FDODefs::GUID guid = GetMTTGuidanceType(value);
		if (guid == FDODefs::GUID::NOGUID) return false;

		data->SLOT = 0;
		data->guid = guid;
		return true;
	}
	else if (type == "ITER")
	{
		bool iter;

		if (value == "YES") iter = true;
		else if (value == "NO") iter = false;
		else return false;

		data->SLOT = 0;
		data->ITER = iter;
		return true;
	}
	else if (type == "IMP")
	{
		bool IMP;

		if (value == "OPT") IMP = true;
		else if (value == "IMP") IMP = false;
		else return false;

		data->SLOT = 0;
		data->IMP = IMP;
		return true;
	}
	else if (type == "RREF")
	{
		bool RREF;

		if (value == "TVR") RREF = true;
		else if (value == "ADI") RREF = false;
		else return false;

		data->SLOT = 0;
		data->RREF = RREF;
		return true;
	}
	else if (type == "ROLL")
	{
		double roll;

		if (sscanf_s(value.c_str(), "%lf", &roll) == 1)
		{
			if (roll >= 0.0 && roll <= 360.0)
			{
				data->SLOT = 0;
				data->ROLL = roll * RAD;
				return true;
			}
		}
	}
	
	return false;
}

void ShuttleFDOCore::ExecuteMTT()
{
	//Sanity checks
	if (ManeuverTransferTable.size() < 1) return;
	if (ManeuverEvaluationTable.Maneuvers.size() < 1) return;
	if (ManeuverTransferTable.size() != ManeuverEvaluationTable.Maneuvers.size()) return;

	DMTInputTable.clear();

	DMTINPUT man;
	SV sv_cur, sv_tig;
	double dt, F, isp, dt_burn, W_dot, cutoff_mass;

	for (unsigned i = 0;i < ManeuverTransferTable.size();i++)
	{
		//Get state vector before maneuver
		sv_cur = ManeuverEvaluationTable.Maneuvers[i].sv_before;
		//Get thruster data
		GetThrusterData(ManeuverTransferTable[i].thrusters, F, isp);
		W_dot = F / isp;
		//Inertial DV
		man.DV_iner = ManeuverEvaluationTable.Maneuvers[i].V_after - ManeuverEvaluationTable.Maneuvers[i].sv_before.V;
		//Calculate burn time
		dt_burn = sv_cur.mass / W_dot * (1.0 - exp(-length(man.DV_iner) * W_dot / F));
		//Estimate cutoff mass
		cutoff_mass = sv_cur.mass - W_dot * dt_burn;

		//Very small burn, bypass logic
		if (ManeuverEvaluationTable.Maneuvers[i].DVMag < 0.1)
		{
			sv_tig = sv_cur;
		}
		else
		{
			if (ManeuverTransferTable[i].IMP)
			{
				//Balance DV before and after impulsive TIG
				double S, A, B, dt_tig;
				S = F / W_dot;
				A = S * log(sv_cur.mass / cutoff_mass);
				B = (sv_cur.mass / W_dot)*A - S * dt_burn;
				dt_tig = -B / A;

				sv_tig = OrbMech::coast_auto(sv_cur, dt_tig, useNonSphericalGravity);
			}
			else
			{
				sv_tig = sv_cur;
			}
		}

		man.sv_tig = sv_tig;

		man.TV_ROLL = ManeuverTransferTable[i].ROLL;
		man.thrusters = ManeuverTransferTable[i].thrusters;
		man.comment = ManeuverTransferTable[i].COMMENT;

		DMTInputTable.push_back(man);
	}
}

void ShuttleFDOCore::CalcDMT()
{
	if (DMT_MNVR < 1 || DMT_MNVR > DMTInputTable.size()) return;

	DMTINPUT input = DMTInputTable[DMT_MNVR - 1];
	SV sv_cut;
	MATRIX3 Rot;
	VECTOR3 u_A, DV_LVLH, DV_iner_act;
	double F, isp, P, LY, RY, p_T, y_T;
	char Buffer[100], Buffer2[100];

	//Column 1
	DMT.GMTI = input.sv_tig.GMT;
	DMT.PETI = GETfromGMT(input.sv_tig.GMT);

	//PAD Data
	GetDMTThrusterType(Buffer, input.thrusters);
	GetDMTManeuverID(Buffer2, input.comment.c_str());

	sprintf_s(DMT.CODE, "%sE%02d%s", Buffer, DMT_MNVR, Buffer2);
	DMT.TV_ROLL = input.TV_ROLL*DEG;
	if (input.thrusters == FDODefs::THRUSTERS::OBP)
	{
		OMSTVC(_V(1071.75429, 0.0, 364.71665), true, P, LY, RY);

		DMT.TRIMS_P = P * DEG;
		DMT.TRIMS_LY = LY * DEG;
		DMT.TRIMS_RY = RY * DEG;

		p_T = P - PITCH_BIAS;
		y_T = 0.0;
	}
	else if (input.thrusters == FDODefs::THRUSTERS::OL || input.thrusters == FDODefs::THRUSTERS::OR)
	{
		OMSTVC(_V(1071.75429, 0.0, 364.71665), false, P, LY, RY);

		DMT.TRIMS_P = P * DEG;
		DMT.TRIMS_LY = LY * DEG;
		DMT.TRIMS_RY = RY * DEG;

		if (input.thrusters == FDODefs::THRUSTERS::OL)
		{
			p_T = P - PITCH_BIAS;
			y_T = LY + YAW_BIAS;
		}
		else
		{
			p_T = P - PITCH_BIAS;
			y_T = RY - YAW_BIAS;
		}
	}
	else
	{
		// RCS
		DMT.TRIMS_P = 0.0;
		DMT.TRIMS_LY = 0.0;
		DMT.TRIMS_RY = 0.0;

		p_T = -10.0 * RAD;
		y_T = 0.0;
	}
	DMT.WEIGHT = input.sv_tig.mass / LBM2KG;
	DMT.TIG = GETfromGMT(input.sv_tig.GMT);
	
	DMT.PEG4_C1 = 0.0;
	DMT.PEG4_C2 = 0.0;
	DMT.PEG4_HT = 0.0;
	DMT.PEG4_PRPLT = 0.0;
	DMT.PEG4_THETAT = 0.0;

	Rot = OrbMech::LVLH_Matrix(input.sv_tig.R, input.sv_tig.V);
	DV_LVLH = mul(Rot, input.DV_iner) / FPS2MPS;

	// Round this, so that it agrees with the input for the Shuttle computer
	DV_LVLH.x = round(DV_LVLH.x*10.0) / 10.0;
	DV_LVLH.y = round(DV_LVLH.y*10.0) / 10.0;
	DV_LVLH.z = round(DV_LVLH.z*10.0) / 10.0;
	//Actual inertial DV vector
	DV_iner_act = tmul(Rot, DV_LVLH) * FPS2MPS;

	DMT.PEG7_DV = DV_LVLH;

	u_A = _V(cos(y_T)*cos(p_T), sin(y_T), -cos(y_T)*sin(p_T));

	MATRIX3 MTP;
	VECTOR3 u_D, VEC_BOD, RR_BOD, VEC_M50, RR_M50, YN, YT, vec_A, vec_B, RORB, VORB, Att;
	double ROLL;

	u_D = unit(DV_iner_act);
	VEC_BOD = u_A;
	if (abs(sin(y_T)) <= 0.999848)
	{
		RR_BOD = _V(0, 1, 0);
	}
	else
	{
		RR_BOD = _V(0, 0, -1);
	}
	RORB = TEG2M50(input.sv_tig.R);
	VORB = TEG2M50(input.sv_tig.V);
	VEC_M50 = TEG2M50(u_D);
	ROLL = input.TV_ROLL + PI05;
	RR_M50 = -unit(crossp(RORB, VORB));
	YN = unit(crossp(VEC_BOD, RR_BOD));
	YT = unit(crossp(VEC_M50, RR_M50))*sin(ROLL) - crossp(VEC_M50, unit(crossp(VEC_M50, RR_M50)))*cos(ROLL);
	vec_A = crossp(VEC_BOD, YN);
	vec_B = crossp(VEC_M50, YT);
	
	MTP = mul(OrbMech::tmat(_M(VEC_BOD.x, VEC_BOD.y, VEC_BOD.z, vec_A.x, vec_A.y, vec_A.z, -YN.x, -YN.y, -YN.z)), _M(VEC_M50.x, VEC_M50.y, VEC_M50.z, vec_B.x, vec_B.y, vec_B.z, -YT.x, -YT.y, -YT.z));

	Att.z = asin(MTP.m12);
	if (abs(cos(Att.z)) < 0.005)
	{
		Att.x = 0.0;
		Att.y = atan2(MTP.m31, MTP.m33);
	}
	else
	{
		Att.y = atan2(-MTP.m13, MTP.m11);
		Att.x = atan2(-MTP.m32, MTP.m22);
	}

	for (int i = 0;i < 3;i++)
	{
		if (Att.data[i] < 0) Att.data[i] += PI2;
	}

	DMT.BURN_ATT = Att * DEG;
	DMT.DVTOT = DMT.DV_M = length(DV_iner_act) / FPS2MPS;

	GetThrusterData(input.thrusters, F, isp);
	DMT.TGO = isp / F * input.sv_tig.mass*(1.0 - exp(-length(DV_iner_act) / isp));
	DMT.VGO = u_A*DMT.DVTOT;

	sv_cut = PoweredFlightProcessor(input.sv_tig, DV_iner_act, F, isp, useNonSphericalGravity);

	double apo, peri;

	if (useNonSphericalGravity)
	{
		ApsidesMagnitudeDetermination(sv_cut, apo, peri);
	}
	else
	{
		OrbMech::periapo(sv_cut.R, sv_cut.V, mu, apo, peri);
	}

	DMT.TGT_HA = (apo - OrbMech::EARTH_RADIUS_EQUATOR) / 1852.0;
	DMT.TGT_HP = (peri - OrbMech::EARTH_RADIUS_EQUATOR) / 1852.0;
}

void ShuttleFDOCore::GetThrusterData(FDODefs::THRUSTERS type, double &F, double &isp)
{
	if (type == FDODefs::THRUSTERS::OBP)
	{
		F = 2.0*OMS_THRUST;
		isp = OMS_ISP0;
	}
	else if (type == FDODefs::THRUSTERS::OL || type == FDODefs::THRUSTERS::OR)
	{
		F = OMS_THRUST;
		isp = OMS_ISP0;
	}
	else
	{
		isp = RCS_ISP0;
		if (type == FDODefs::THRUSTERS::PX2) F = 2.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::PX3) F = 3.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::PX3) F = 4.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::MXL) F = 2.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::YL) F = 2.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::MYL) F = 2.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::ZH) F = 3.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::ZL) F = 4.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::MZL) F = 6.0*RCS_THRUST;
		else if (type == FDODefs::THRUSTERS::MZH) F = 2.0*RCS_THRUST;
		else F = 2.0*RCS_THRUST;
	}
}

void ShuttleFDOCore::GetMTTThrusterType(char *buf, FDODefs::THRUSTERS type)
{
	if (type == FDODefs::THRUSTERS::PX4)
	{
		sprintf_s(buf, 100, "PX4");
	}
	else if (type == FDODefs::THRUSTERS::PX3)
	{
		sprintf_s(buf, 100, "PX3");
	}
	else if (type == FDODefs::THRUSTERS::PX2)
	{
		sprintf_s(buf, 100, "PX2");
	}
	else if (type == FDODefs::THRUSTERS::MXL)
	{
		sprintf_s(buf, 100, "MXL");
	}
	else if (type == FDODefs::THRUSTERS::YL)
	{
		sprintf_s(buf, 100, "YL");
	}
	else if (type == FDODefs::THRUSTERS::MYL)
	{
		sprintf_s(buf, 100, "MYL");
	}
	else if (type == FDODefs::THRUSTERS::ZH)
	{
		sprintf_s(buf, 100, "ZH");
	}
	else if (type == FDODefs::THRUSTERS::ZL)
	{
		sprintf_s(buf, 100, "ZL");
	}
	else if (type == FDODefs::THRUSTERS::MZH)
	{
		sprintf_s(buf, 100, "MZH");
	}
	else if (type == FDODefs::THRUSTERS::M1)
	{
		sprintf_s(buf, 100, "M1");
	}
	else if (type == FDODefs::THRUSTERS::M2)
	{
		sprintf_s(buf, 100, "M2");
	}
	else if (type == FDODefs::THRUSTERS::OL)
	{
		sprintf_s(buf, 100, "OL");
	}
	else if (type == FDODefs::THRUSTERS::OR)
	{
		sprintf_s(buf, 100, "OR");
	}
	else if (type == FDODefs::THRUSTERS::OBP)
	{
		sprintf_s(buf, 100, "OBP");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

FDODefs::THRUSTERS ShuttleFDOCore::GetMTTThrusterType(const std::string& type) const
{
	if (type == "PX4") return FDODefs::THRUSTERS::PX4;
	else if (type == "PX3") return FDODefs::THRUSTERS::PX3;
	else if (type == "PX2") return FDODefs::THRUSTERS::PX2;
	else if (type == "MXL") return FDODefs::THRUSTERS::MXL;
	else if (type == "YL") return FDODefs::THRUSTERS::YL;
	else if (type == "MYL") return FDODefs::THRUSTERS::MYL;
	else if (type == "ZH") return FDODefs::THRUSTERS::ZH;
	else if (type == "ZL") return FDODefs::THRUSTERS::ZL;
	else if (type == "MZH") return FDODefs::THRUSTERS::MZH;
	else if (type == "MZL") return FDODefs::THRUSTERS::MZL;
	else if (type == "M1") return FDODefs::THRUSTERS::M1;
	else if (type == "OL") return FDODefs::THRUSTERS::OL;
	else if (type == "OR") return FDODefs::THRUSTERS::OR;
	else if (type == "OBP") return FDODefs::THRUSTERS::OBP;
	else return FDODefs::THRUSTERS::NOTHRU;
}

void ShuttleFDOCore::GetMTTGuidanceType(char* buf, FDODefs::GUID type) const
{
	if (type == FDODefs::GUID::M50)
	{
		sprintf_s(buf, 100, "M50");
	}
	else if (type == FDODefs::GUID::P7)
	{
		sprintf_s(buf, 100, "P7");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

FDODefs::GUID ShuttleFDOCore::GetMTTGuidanceType(const std::string& type) const
{
	if (type == "M50") return FDODefs::GUID::M50;
	else if (type == "P7") return FDODefs::GUID::P7;
	else return FDODefs::GUID::NOGUID;
}

void ShuttleFDOCore::GetDMTThrusterType(char *buf, FDODefs::THRUSTERS type)
{
	if (type == FDODefs::THRUSTERS::PX4 || type == FDODefs::THRUSTERS::PX3)
	{
		sprintf_s(buf, 100, "XH");
	}
	else if (type == FDODefs::THRUSTERS::PX2)
	{
		sprintf_s(buf, 100, "XL");
	}
	else  if (type == FDODefs::THRUSTERS::ZL)
	{
		sprintf_s(buf, 100, "ZL");
	}
	else if (type == FDODefs::THRUSTERS::MXL)
	{
		sprintf_s(buf, 100, "MX");
	}
	else if (type == FDODefs::THRUSTERS::YL)
	{
		sprintf_s(buf, 100, "YL");
	}
	else if (type == FDODefs::THRUSTERS::MYL)
	{
		sprintf_s(buf, 100, "MY");
	}
	else if (type == FDODefs::THRUSTERS::ZH)
	{
		sprintf_s(buf, 100, "ZH");
	}
	else if (type == FDODefs::THRUSTERS::MZH)
	{
		sprintf_s(buf, 100, "ZM");
	}
	else if (type == FDODefs::THRUSTERS::M1)
	{
		sprintf_s(buf, 100, "M1");
	}
	else if (type == FDODefs::THRUSTERS::M2)
	{
		sprintf_s(buf, 100, "M2");
	}
	else if (type == FDODefs::THRUSTERS::OL)
	{
		sprintf_s(buf, 100, "OL");
	}
	else if (type == FDODefs::THRUSTERS::OR)
	{
		sprintf_s(buf, 100, "OR");
	}
	else if (type == FDODefs::THRUSTERS::OBP)
	{
		sprintf_s(buf, 100, "BP");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOCore::GetDMTManeuverID(char *buf, const char *name)
{
	if (strcmp(name, "OMS-1") == 0)
	{
		sprintf_s(buf, 100, "O1");
	}
	else if (strcmp(name, "OMS-2") == 0)
	{
		sprintf_s(buf, 100, "O2");
	}
	else if (strlen(name) >= 2)
	{
		sprintf_s(buf, 100, "%.2s", name);
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOCore::SetLaunchDay()
{
	int Y, D, H, M;
	double MJD, S;

	MJD = oapiGetSimMJD();

	OrbMech::mjd2ydoy(MJD, Y, D, H, M, S);

	SetLaunchDay(Y, D);
}

void ShuttleFDOCore::SetLaunchDay(int Y, int D)
{
	double stemp;
	int Ytemp, htemp, mtemp;

	// Calculate base MJD
	sescnst.GMTBASE = OrbMech::Date2MJD(Y, D, 0, 0, 0.0);
	// Calculate month and day of month
	OrbMech::mjd2date(sescnst.GMTBASE, Ytemp, sescnst.Month, sescnst.Day, htemp, mtemp, stemp);
	// Reset launch time to zero
	sescnst.GMTLO = 0.0;

	// TEG to J2000 ecliptic (left handed)
	MATRIX3 M_EFTOECL_AT_EPOCH = OrbMech::GetRotationMatrix(sescnst.GMTBASE);
	// M50 to J2000 (left handed)
	MATRIX3 M_M50TOECL = OrbMech::GetObliquityMatrix(33281.923357);
	// TEG to M50 (right handed)
	sescnst.M_TEG_TO_M50 = OrbMech::MatrixRH_LH(mul(OrbMech::tmat(M_M50TOECL), M_EFTOECL_AT_EPOCH));
	// TEG to J2000 ecliptic (right handed)
	sescnst.M_TEG_TO_J2000 = OrbMech::MatrixRH_LH(M_EFTOECL_AT_EPOCH);

	sescnst.Year = Y;
	sescnst.DayOfYear = D;
	sescnst.Hours = sescnst.Minutes = 0;
	sescnst.launchdateSec = 0.0;

	ErrorCode = 0;
}

void ShuttleFDOCore::SetLaunchTime(int H, int M, double S)
{
	//Launch time
	sescnst.GMTLO = (double)(H * 3600 + M * 60) + S;

	sescnst.Hours = H;
	sescnst.Minutes = M;
	sescnst.launchdateSec = S;
}

void ShuttleFDOCore::OMSTVC(VECTOR3 CG, bool parallel, double &P, double &LY, double &RY)
{
	double A, B, C, D, R1;

	A = 1518.0 - CG.x;
	B = CG.y - 88.0;
	C = 492.0 - CG.z;
	D = CG.y + 88.0;
	R1 = sqrt(A*A + C*C);

	if (parallel == false)
	{
		P = PITCH_BIAS - atan2(C, A);
		LY = -YAW_BIAS + atan2(D, R1);
		RY = YAW_BIAS + atan2(B, R1);
	}
	else
	{
		P = PITCH_BIAS - atan2(C, A);
		LY = -5.7*RAD + atan2(CG.y, R1);
		RY = 5.7*RAD + atan2(CG.y, R1);
	}
}

MATRIX3 MATRIX(VECTOR3 A, VECTOR3 B, VECTOR3 C)
{
	return _M(A.x, A.y, A.z, B.x, B.y, B.z, C.x, C.y, C.z);
}

SV ShuttleFDOCore::PoweredFlightProcessor(SV sv_tig, VECTOR3 DV_iner, double f_T, double v_ex, bool nonspherical)
{
	SV sv_cut;
	double t_go;

	OrbMech::poweredflight(sv_tig.R, sv_tig.V, f_T, v_ex, sv_tig.mass, DV_iner, nonspherical, sv_cut.R, sv_cut.V, sv_cut.mass, t_go);
	sv_cut.GMT = sv_tig.GMT + t_go;
	return sv_cut;
}

VECTOR3 ShuttleFDOCore::TEG2M50(VECTOR3 v_TEG)
{
	return mul(sescnst.M_TEG_TO_M50, v_TEG);
}

MATRIX3 ShuttleFDOCore::TEG_to_EF_Matrix(double gmt) const
{
	double CL, SL;

	CL = cos(gmt * OrbMech::w_Earth);
	SL = sin(gmt * OrbMech::w_Earth);

	return _M(CL, SL, 0.0, -SL, CL, 0.0, 0.0, 0.0, 1.0);
}