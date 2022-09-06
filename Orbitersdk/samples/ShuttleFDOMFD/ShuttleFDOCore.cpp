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

ShuttleFDOCore::ShuttleFDOCore(VESSEL* v)
{
	vessel = v;

	hEarth = oapiGetObjectByName("Earth");
	mu = GGRAV * oapiGetMass(hEarth);
	R_E = oapiGetSize(hEarth);
	w_E = PI2 / oapiGetPlanetPeriod(hEarth);
	
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

	InPlaneGMT = 0.0;
	DMT_MNVR = 0;
	useNonSphericalGravity = vessel->NonsphericalGravityEnabled();
	OMPErrorCode = 0;
	chaserSVOption = false;

	subThreadMode = 0;
	subThreadStatus = 0;

	for (int i = 0;i < 4;i++)
	{
		launchdate[i] = 0;
	}
	launchdateSec = 0.0;
	LaunchGMT = 0.0;
	BaseMJD = 0.0;
	M_EFTOECL_AT_EPOCH = _M(1, 0, 0, 0, 1, 0, 0, 0, 1);
	M_TEGTOECL = _M(1, 0, 0, 0, 1, 0, 0, 0, 1);

	MTTSlotData[0].SLOT = 1;
	MTTSlotData[0].thrusters = OMPDefs::THRUSTERS::OBP;
	MTTSlotData[0].guid = OMPDefs::GUID::P7;
	MTTSlotData[0].ITER = false;
	MTTSlotData[0].IMP = true;
	MTTSlotData[0].RREF = true;
	MTTSlotData[0].ROLL = 0;

	MTTSlotData[1].SLOT = 2;
	MTTSlotData[1].thrusters = OMPDefs::THRUSTERS::OBP;
	MTTSlotData[1].guid = OMPDefs::GUID::P7;
	MTTSlotData[1].ITER = false;
	MTTSlotData[1].IMP = true;
	MTTSlotData[1].RREF = true;
	MTTSlotData[1].ROLL = PI;

	MTTSlotData[2].SLOT = 3;
	MTTSlotData[2].thrusters = OMPDefs::THRUSTERS::OL;
	MTTSlotData[2].guid = OMPDefs::GUID::P7;
	MTTSlotData[2].ITER = false;
	MTTSlotData[2].IMP = true;
	MTTSlotData[2].RREF = true;
	MTTSlotData[2].ROLL = 0;

	MTTSlotData[3].SLOT = 4;
	MTTSlotData[3].thrusters = OMPDefs::THRUSTERS::OL;
	MTTSlotData[3].guid = OMPDefs::GUID::P7;
	MTTSlotData[3].ITER = false;
	MTTSlotData[3].IMP = true;
	MTTSlotData[3].RREF = true;
	MTTSlotData[3].ROLL = PI;

	MTTSlotData[4].SLOT = 5;
	MTTSlotData[4].thrusters = OMPDefs::THRUSTERS::OR;
	MTTSlotData[4].guid = OMPDefs::GUID::P7;
	MTTSlotData[4].ITER = false;
	MTTSlotData[4].IMP = true;
	MTTSlotData[4].RREF = true;
	MTTSlotData[4].ROLL = 0;

	MTTSlotData[5].SLOT = 6;
	MTTSlotData[5].thrusters = OMPDefs::THRUSTERS::OR;
	MTTSlotData[5].guid = OMPDefs::GUID::P7;
	MTTSlotData[5].ITER = false;
	MTTSlotData[5].IMP = true;
	MTTSlotData[5].RREF = true;
	MTTSlotData[5].ROLL = PI;

	MTTSlotData[6].SLOT = 7;
	MTTSlotData[6].thrusters = OMPDefs::THRUSTERS::PX2;
	MTTSlotData[6].guid = OMPDefs::GUID::P7;
	MTTSlotData[6].ITER = false;
	MTTSlotData[6].IMP = true;
	MTTSlotData[6].RREF = true;
	MTTSlotData[6].ROLL = 0;

	MTTSlotData[7].SLOT = 8;
	MTTSlotData[7].thrusters = OMPDefs::THRUSTERS::PX2;
	MTTSlotData[7].guid = OMPDefs::GUID::P7;
	MTTSlotData[7].ITER = false;
	MTTSlotData[7].IMP = true;
	MTTSlotData[7].RREF = true;
	MTTSlotData[7].ROLL = PI;

	MTTSlotData[8].SLOT = 9;
	MTTSlotData[8].thrusters = OMPDefs::THRUSTERS::PX2;
	MTTSlotData[8].guid = OMPDefs::GUID::P7;
	MTTSlotData[8].ITER = false;
	MTTSlotData[8].IMP = false;
	MTTSlotData[8].RREF = true;
	MTTSlotData[8].ROLL = 0;

	MTTSlotData[9].SLOT = 10;
	MTTSlotData[9].thrusters = OMPDefs::THRUSTERS::OL;
	MTTSlotData[9].guid = OMPDefs::GUID::P7;
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
	LWP_Settings.DTOPT = -(5.0*60.0 + 23.0);//-(5.0*60.0 + 40.0);
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
	LWP_Settings.LW = 2;
	LWP_Settings.DELNO = 0.0;
	LWP_Settings.LOT = 6;
	LWP_Settings.CWHT = 251679.0*LBM2KG;

	LWP_LaunchSite = 1;
	DTIG_ET_SEP = 12.0;
	DTIG_MPS = 1.0*60.0 + 54.0;
	DV_ET_SEP = _V(5.0, 0.0, -5.0)*0.3048;
	DV_MPS = _V(9.7, 0.2, -1.0)*0.3048;

	LWP_PlanarOpenGMT = 0.0;
	LWP_PlanarCloseGMT = 0.0;

	DOPS_GETS = DOPS_GETF = 0.0;
	DOPS_InitialRev = 1;
	DOPS_MaxXRNG = 800.0;
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
	sv.GMT = (oapiGetSimMJD() - BaseMJD)*24.0*3600.0;

	sv.R = _V(R.x, R.z, R.y);
	sv.V = _V(V.x, V.z, V.y);

	//Use TEG coordinate system
	sv.R = rhtmul(M_EFTOECL_AT_EPOCH, sv.R);
	sv.V = rhtmul(M_EFTOECL_AT_EPOCH, sv.V);

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

void ShuttleFDOCore::ApsidesMagnitudeDetermination(SV sv0, double &r_A, double &r_P)
{
	OrbMech::OELEMENTS coe;
	double r, a, u;

	coe = OrbMech::coe_from_sv(sv0.R, sv0.V, mu);
	a = OrbMech::GetSemiMajorAxis(sv0.R, sv0.V, mu);
	r = length(sv0.R);
	u = coe.TA + coe.w;

	OrbMech::PIFAAP(a, coe.e, coe.i, coe.TA, u, r, R_E, r_A, r_P);
}

void ShuttleFDOCore::ApsidesDeterminationSubroutine(SV sv0, SV &sv_a, SV &sv_p)
{
	OrbMech::OELEMENTS coe;
	bool lowecclogic;

	coe = OrbMech::coe_from_sv(sv0.R, sv0.V, mu);

	if (coe.e > 0.005)
	{
		lowecclogic = false;
	}
	else
	{
		lowecclogic = true;
	}

	if (lowecclogic == false)
	{
		sv_p = GeneralTrajectoryPropagation(sv0, 1, 0.0, 0.0, useNonSphericalGravity);
		sv_a = GeneralTrajectoryPropagation(sv0, 1, PI, 0.0, useNonSphericalGravity);
	}
	else
	{
		SV sv1, sv2;
		double u_x, u_y;

		//First guess
		ApsidesArgumentofLatitudeDetermination(sv0, u_x, u_y);

		sv1 = GeneralTrajectoryPropagation(sv0, 2, u_x, 0.0, useNonSphericalGravity);
		sv2 = GeneralTrajectoryPropagation(sv0, 2, u_y, 0.0, useNonSphericalGravity);

		if (length(sv1.R) > length(sv2.R))
		{
			sv_a = sv1;
			sv_p = sv2;
		}
		else
		{
			sv_p = sv1;
			sv_a = sv2;
		}
	}
}

void ShuttleFDOCore::ApsidesArgumentofLatitudeDetermination(SV sv0, double &u_x, double &u_y)
{
	OrbMech::OELEMENTS coe;
	SV sv[3];
	double u[3], r[3], gamma, u_0;

	sv[0] = sv0;
	sv[1] = coast(sv[0], 15.0*60.0);
	sv[2] = coast(sv[1], 15.0*60.0);

	for (int i = 0;i < 3;i++)
	{
		coe = OrbMech::coe_from_sv(sv[i].R, sv[i].V, mu);
		u[i] = fmod(coe.TA + coe.w, PI2);
		r[i] = length(sv[i].R);
	}

	gamma = (r[0] - r[1]) / (r[0] - r[2]);
	u_0 = atan2(sin(u[0]) - sin(u[1]) - gamma * (sin(u[0]) - sin(u[2])), gamma*(cos(u[2]) - cos(u[0])) - cos(u[1]) + cos(u[0]));
	if (u_0 < 0)
	{
		u_0 += PI2;
	}

	u_x = u_0 + PI05;
	u_y = u_0 - PI05;

	if (u_x < 0)
	{
		u_x += PI2;
	}
	if (u_y < 0)
	{
		u_y += PI2;
	}

}

SV ShuttleFDOCore::PositionMatch(SV sv_A, SV sv_P)
{
	SV sv_A1, sv_P1;
	VECTOR3 u, R_A1, U_L;
	double phase, n, dt, ddt;

	dt = 0.0;

	u = unit(crossp(sv_P.R, sv_P.V));
	U_L = unit(crossp(u, sv_P.R));
	sv_A1 = GeneralTrajectoryPropagation(sv_A, 0, sv_P.GMT, 0.0, useNonSphericalGravity);

	do
	{
		R_A1 = unit(sv_A1.R - u * dotp(sv_A1.R, u))*length(sv_A1.R);
		phase = acos(dotp(unit(R_A1), unit(sv_P.R)));
		if (dotp(U_L, R_A1) > 0)
		{
			phase = -phase;
		}
		n = OrbMech::GetMeanMotion(sv_A1.R, sv_A1.V, mu);
		ddt = phase / n;
		sv_A1 = coast(sv_A1, ddt);
		dt += ddt;
	} while (abs(ddt) > 0.01);

	return sv_A1;
}

VECTOR3 ShuttleFDOCore::SOIManeuver(SV sv_A, SV sv_P, double GMT1, double dt, VECTOR3 off)
{
	SV sv_A1, sv_P2;
	VECTOR3 RP2_off, VA1_apo, DV;
	double dt1, dt2;

	dt1 = GMT1 - sv_A.GMT;
	dt2 = GMT1 - sv_P.GMT + dt;

	sv_A1 = coast_auto(sv_A, dt1);
	sv_P2 = coast_auto(sv_P, dt2);

	OrbMech::REL_COMP(sv_P2.R, sv_P2.V, RP2_off, off);
	VA1_apo = LambertAuto(sv_A1.R, sv_A1.V, sv_A1.GMT, RP2_off, dt, 0, true);
	DV = VA1_apo - sv_A1.V;
	return DV;
}

VECTOR3 ShuttleFDOCore::SORManeuver(SV sv_A, SV sv_P, double GMT1, VECTOR3 off)
{
	SV sv_A1, sv_P1, sv_P2;
	VECTOR3 VA1_apo, DV, RP2_off;
	double dt, dt1, dt2;

	dt1 = GMT1 - sv_A.GMT;
	dt2 = GMT1 - sv_P.GMT;

	sv_A1 = coast_auto(sv_A, dt1);
	sv_P1 = coast_auto(sv_P, dt2);

	dt = OrbMech::time_theta(sv_P1.R, sv_P1.V, 270.0*RAD, mu);
	sv_P2 = coast_auto(sv_P1, dt);

	OrbMech::REL_COMP(sv_P2.R, sv_P2.V, RP2_off, off);
	VA1_apo = LambertAuto(sv_A1.R, sv_A1.V, sv_A1.GMT, RP2_off, dt, 0, true);
	DV = VA1_apo - sv_A1.V;
	return DV;
}

VECTOR3 ShuttleFDOCore::LambertAuto(VECTOR3 RA, VECTOR3 VA, double GMT0, VECTOR3 RP_off, double dt, int N, bool prog)
{
	if (useNonSphericalGravity)
	{
		return OrbMech::Vinti(RA, VA, RP_off, GMT0, dt, N, prog, _V(0, 0, 0));
	}
	else
	{
		return OrbMech::elegant_lambert(RA, VA, RP_off, dt, N, prog, mu);
	}
}

void ShuttleFDOCore::AddManeuver(OMPDefs::MANTYPE type, char *name, unsigned ins)
{
	ManeuverConstraints man;

	sprintf_s(man.name, name);
	man.type = type;
	man.threshold = OMPDefs::THRESHOLD::NOTHR;
	man.thresh_num = 0.0;

	if (ins == 0 || ins == ManeuverConstraintsTable.size() + 1)
	{
		ManeuverConstraintsTable.push_back(man);
	}
	else
	{
		ManeuverConstraintsTable.insert(ManeuverConstraintsTable.begin() + ins - 1, man);
	}
}

void ShuttleFDOCore::ModifyManeuver(unsigned num, OMPDefs::MANTYPE type, char *name)
{
	if (num >= 0 && num < ManeuverConstraintsTable.size())
	{
		sprintf_s(ManeuverConstraintsTable[num].name, name);
		ManeuverConstraintsTable[num].type = type;
		//ManeuverConstraintsTable[num].threshold = OMPDefs::THRESHOLD::NOTHR;
		//ManeuverConstraintsTable[num].thresh_num = 0.0;
		//ManeuverConstraintsTable[num].secondaries.clear();
	}
}

void ShuttleFDOCore::AddManeuverThreshold(unsigned num, OMPDefs::THRESHOLD type, double time)
{
	ManeuverConstraintsTable[num].threshold = type;
	ManeuverConstraintsTable[num].thresh_num = time;
}

void ShuttleFDOCore::AddManeuverSecondary(unsigned num, char *type, double value)
{
	SecData sec;

	sprintf_s(sec.type, 5, type);
	sec.value = value;
	ManeuverConstraintsTable[num].secondaries.push_back(sec);
}

int ShuttleFDOCore::CalculateOMPPlan()
{
	if (target == NULL) return 100;
	if (ManeuverConstraintsTable.size() < 1) return 1;	//Error 1: No maneuvers in constraint table
	if (ManeuverConstraintsTable[0].threshold != OMPDefs::THRESHOLD::THRES_T) return 2;	//Error 2: First maneuver needs a T as threshold

	ManeuverEvaluationTable.clear();

	std::vector<SV> sv_bef_table, sv_aft_table, sv_P_table;
	SV sv_phantom;
	VECTOR3 DV;
	std::vector<VECTOR3> dv_table;
	std::vector<VECTOR3> add_constraint;
	std::vector<TIGSecondaries> tigmodifiers;
	std::vector<SV> sv_thres;
	double dt;
	unsigned i, j, k, l, TAB;
	std::vector<ITERCONSTR> iterators;

	TAB = ManeuverConstraintsTable.size();
	l = 0;

	sv_bef_table.resize(TAB);
	sv_aft_table.resize(TAB);
	sv_P_table.resize(TAB);
	tigmodifiers.resize(TAB);
	dv_table.resize(TAB);
	add_constraint.resize(TAB);
	sv_thres.resize(TAB);

	for (i = 0;i < TAB;i++)
	{
		tigmodifiers[i].type = OMPDefs::SECONDARIES::NOSEC;
		tigmodifiers[i].value = 0.0;
		dv_table[i] = _V(0, 0, 0);
		add_constraint[i] = _V(0, 0, 0);
	}

	ITERCONSTR con;
	bool npcflag = false;
	int found;
	//SET UP ITERATORS and CHECK THAT THRESHOLDS EXIST
	for (i = 0;i < TAB;i++)
	{
		found = 0;

		if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::NOTHR) return 5;	//Error 5: Maneuver doesn't have a threshold
		
		if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NC)
		{
			for (k = i + 1;k < ManeuverConstraintsTable.size();k++)
			{
				for (j = 0;j < ManeuverConstraintsTable[k].secondaries.size();j++)
				{
					if (strcmp(ManeuverConstraintsTable[k].secondaries[j].type, "DR") == 0)
					{
						found = 1;
					}
					if (found) break;
				}
				if (found) break;
			}

			if (!found) return 7;	//Error 7: didn't find NC constraint

			con.man = i;
			con.type = 1;
			con.constr = k;
			con.value = ManeuverConstraintsTable[k].secondaries[j].value*1852.0;

			iterators.push_back(con);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NH || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NHRD)
		{
			for (k = i + 1;k < ManeuverConstraintsTable.size();k++)
			{
				for (j = 0;j < ManeuverConstraintsTable[k].secondaries.size();j++)
				{
					if (strcmp(ManeuverConstraintsTable[k].secondaries[j].type, "DH") == 0)
					{
						found = 1;
					}
					if (found) break;
				}
				if (found) break;
			}

			if (!found) return 8;	//Error 8: didn't find NH constraint

			con.man = i;
			con.type = 2;
			con.constr = k;
			con.value = ManeuverConstraintsTable[k].secondaries[j].value*1852.0;

			iterators.push_back(con);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NPC)
		{
			for (k = i + 1;k < ManeuverConstraintsTable.size();k++)
			{
				for (j = 0;j < ManeuverConstraintsTable[k].secondaries.size();j++)
				{
					if (strcmp(ManeuverConstraintsTable[k].secondaries[j].type, "WEDG") == 0)
					{
						found = 1;
					}
					if (found) break;
				}
				if (found) break;
			}

			if (npcflag) return 22;	//More than one NPC maneuver found

			//Only set up iterator if constraint was found
			if (found)
			{
				npcflag = true;

				con.man = i;
				con.type = 3;
				con.constr = k;
				con.value = ManeuverConstraintsTable[k].secondaries[j].value*1852.0;
				iterators.push_back(con);
			}			
		}
	}

	ITERSTATE *iterstate;
	iterstate = new ITERSTATE[iterators.size()];

	//CHECK THAT MANEUVER CONSTRAINTS EXIST
	for (i = 0;i < TAB;i++)
	{
		found = 0;
		if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HA || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HASH)
		{
			for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
			{
				if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "HD") == 0)
				{
					add_constraint[i].x = ManeuverConstraintsTable[i].secondaries[j].value * 1852.0;
					found++;
				}
			}
			if (found != 1) return 6;	//Didn't find HD constraints for HA maneuver
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::EXDV)
		{
			for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
			{
				if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DVLV") == 0)
				{
					if (found > 2) return 4;													//Error 4: too many DV components specified
					dv_table[i].data[found] = ManeuverConstraintsTable[i].secondaries[j].value*0.3048;
					found++;
				}
			}
			if (found < 3) return 3;													//Error 3: Not enough DV components specified
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOI)
		{
			if (i + 1 >= TAB) return 11; //Error 11: No maneuver after SOI/NCC
			if (ManeuverConstraintsTable[i + 1].type != OMPDefs::MANTYPE::SOR) return 12;	//Error 12: Wrong maneuver after SOI
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOR)
		{
			for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
			{
				if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "CXYZ") == 0)
				{
					if (found > 2) return 9; //Error 9: too many CXYZ components specified
					add_constraint[i].data[found] = ManeuverConstraintsTable[i].secondaries[j].value*1852.0;
					found++;
				}
			}
			if (found < 3) return 10; //Error 10: Not enough CXYZ components specified
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NCC)
		{
			if (i + 1 >= TAB) return 11; //Error 11: No maneuver after SOI/NCC

			for (j = 0;j < ManeuverConstraintsTable[i + 1].secondaries.size();j++)
			{
				if (strcmp(ManeuverConstraintsTable[i + 1].secondaries[j].type, "CXYZ") == 0)
				{
					if (found > 2) return 9; //Error 9: too many CXYZ components specified
					add_constraint[i + 1].data[found] = ManeuverConstraintsTable[i + 1].secondaries[j].value*1852.0;
					found++;
				}
				else if (strcmp(ManeuverConstraintsTable[i + 1].secondaries[j].type, "DH") == 0)
				{
					if (found > 2) return 9; //Error 9: too many CXYZ components specified
					add_constraint[i + 1].z = ManeuverConstraintsTable[i + 1].secondaries[j].value*1852.0;
					found++;
				}
				else if (strcmp(ManeuverConstraintsTable[i + 1].secondaries[j].type, "DR") == 0)
				{
					if (found > 2) return 9; //Error 9: too many CXYZ components specified
					add_constraint[i + 1].x = ManeuverConstraintsTable[i + 1].secondaries[j].value*1852.0;
					found++;
				}
				else if (strcmp(ManeuverConstraintsTable[i + 1].secondaries[j].type, "WEDG") == 0)
				{
					if (found > 2) return 9; //Error 9: too many CXYZ components specified
					add_constraint[i + 1].y = ManeuverConstraintsTable[i + 1].secondaries[j].value*1852.0;
					found++;
				}
			}
			if (found < 3) return 10; //Error 10: Not enough CXYZ components specified
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::DVPY || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::DVYP)
		{
			for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
			{
				if(strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DV") == 0)
				{
					add_constraint[i].x = ManeuverConstraintsTable[i].secondaries[j].value*0.3048;
				}
				else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "PIT") == 0)
				{
					add_constraint[i].y = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
				}
				else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "YAW") == 0)
				{
					add_constraint[i].z = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
				}
			}
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NOSH)
		{
			for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
			{
				if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DNOD") == 0)
				{
					add_constraint[i].x = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
					found++;
				}
			}
			if (found != 1) return 24;	//Didn't find DNOD constraint for NOSH maneuver
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::PC)
		{
		for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
		{
			if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DPC") == 0)
			{
				add_constraint[i].x = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
				found++;
			}
		}
		if (found != 1) return 27;	//Didn't find DPC constraint for PC maneuver
		}
	}

	//CHECK AND LOAD TIG MODIFIERS
	for (i = 0;i < TAB;i++)
	{
		for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
		{
			//Maneuver at apogee
			if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "APO") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::APO;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value;
			}
			//Maneuver at apogee
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "PER") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::PER;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value;
			}
			//Initial guess
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DV") == 0)
			{
				dv_table[i] = _V(ManeuverConstraintsTable[i].secondaries[j].value*0.3048, 0, 0);
			}
			//Common Node
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "CN") == 0)
			{
				if (ManeuverConstraintsTable[i].type != OMPDefs::MANTYPE::NPC) return 14;	//Error 14: CN secondary only applies to NPC
				tigmodifiers[i].type = OMPDefs::SECONDARIES::CN;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value;
			}
			//Maneuver at nth upcoming apsis
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "APS") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::SEC_APS;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value;
			}
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "LITI") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::LITI;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*60.0;
			}
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "LITM") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::LITM;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*60.0;
			}
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "LITO") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::LITO;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*60.0;
			}
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "NITI") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::NITI;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*60.0;
			}
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "NITM") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::NITM;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*60.0;
			}
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "NITO") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::NITO;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*60.0;
			}
			//Optimum node shift
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "OPT") == 0)
			{
				if (ManeuverConstraintsTable[i].type != OMPDefs::MANTYPE::NOSH) return 25;	//Error 25: OPT secondary only applies to NOSH
				tigmodifiers[i].type = OMPDefs::SECONDARIES::OPT;
			}
			//Angle from apogee
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "A") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::A;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
			}
			//Angle from perigee
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "P") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::P;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
			}
			//Argument of latitude
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "U") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::U;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
			}
			//Ascending Node
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "ASC") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::ASC;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value;
			}
			//Descending Node
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DSC") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::DSC;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value;
			}
			//Latitude
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "LAT") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::LAT;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
			}
			//Longitude
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "LON") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::LON;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
			}
			//Declination
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DEC") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::DEC;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*RAD;
			}
			//Latitude
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "ALT") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::ALT;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value*1852.0;
			}
		}
	}

	SV sv_A0, sv_P0, sv_cur, sv_P_cur;
	if (chaserSVOption)
	{
		sv_A0 = sv_chaser;
	}
	else
	{
		sv_A0 = StateVectorCalc(shuttle);
	}
	sv_P0 = StateVectorCalc(target);

	//Save for later use
	sv_chaser = sv_A0;
	sv_target = sv_P0;

	//For Testing
	VECTOR3 u;
	/*u = unit(crossp(sv_P0.R, sv_P0.V));
	sv_A0.R = unit(sv_A0.R - u * dotp(sv_A0.R, u))*length(sv_A0.R);
	sv_A0.V = unit(sv_A0.V - u * dotp(sv_A0.V, u))*length(sv_A0.V);*/

	//Set up loop
	i = 0;
	bool recycle;

	do
	{
		recycle = false;
		if (i == 0)
		{
			sv_cur = sv_A0;
			sv_P_cur = sv_P0;
		}
		else
		{
			sv_cur = sv_aft_table[i - 1];
			sv_P_cur = sv_P_table[i - 1];
		}

		//THRESHOLD
		if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_T)
		{
			dt = GMTfromGET(ManeuverConstraintsTable[i].thresh_num) - sv_cur.GMT;
			sv_bef_table[i] = coast_auto(sv_cur, dt);
		}
		else if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_M || ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_REV ||
			ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_APS || ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_N)
		{
			double mult;

			if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_M || ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_REV)
			{
				mult = 1.0;
			}
			else
			{
				mult = 0.5;
			}

			if (i > 0 && ManeuverConstraintsTable[i - 1].type == OMPDefs::MANTYPE::NPC)
			{
				//Special NPC logic. Propagate threshold state vector for desired orbits, then propagate current state vector to same time. Assumes plane change doesn't affect orbital period
				sv_bef_table[i] = DeltaOrbitsAuto(sv_thres[i - 1], mult*ManeuverConstraintsTable[i].thresh_num);
				sv_bef_table[i] = coast_auto(sv_cur, sv_bef_table[i].GMT - sv_cur.GMT);
			}
			else
			{
				sv_bef_table[i] = DeltaOrbitsAuto(sv_cur, mult*ManeuverConstraintsTable[i].thresh_num);
			}
		}
		else if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_DT)
		{
			//Special NPC logic. DT from threshold time, not current time
			double ddt;
			if (i > 0 && ManeuverConstraintsTable[i - 1].type == OMPDefs::MANTYPE::NPC)
			{
				ddt = ManeuverConstraintsTable[i].thresh_num - (sv_cur.GMT - sv_thres[i - 1].GMT);
			}
			else
			{
				ddt = ManeuverConstraintsTable[i].thresh_num;
			}
			sv_bef_table[i] = coast_auto(sv_cur, ddt);
		}
		else if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_CAN || ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_WT)
		{
			dt = OrbMech::time_theta(sv_bef_table[i].R, sv_bef_table[i].V, ManeuverConstraintsTable[i].thresh_num, mu, true);
			sv_bef_table[i] = DeltaOrbitsAuto(sv_cur, dt);
		}

		//Save state vector at threshold
		sv_thres[i] = sv_bef_table[i];

		//TIG MODIFICATION
		if (tigmodifiers[i].type != OMPDefs::SECONDARIES::NOSEC)
		{
			if (tigmodifiers[i].type == OMPDefs::SECONDARIES::APO)
			{
				sv_bef_table[i] = timetoapo_auto(sv_bef_table[i], tigmodifiers[i].value);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::PER)
			{
				double dt_P = 0.0;
				if (tigmodifiers[i].value > 1.0)
				{
					double P = OrbMech::period(sv_bef_table[i].R, sv_bef_table[i].V, mu);
					dt_P = P * tigmodifiers[i].value - 1.0;
					sv_bef_table[i] = coast_auto(sv_bef_table[i], dt_P);
				}
				dt = OrbMech::timetoperi(sv_bef_table[i].R, sv_bef_table[i].V, mu, 1);
				sv_bef_table[i] = coast_auto(sv_bef_table[i], dt);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::CN)
			{
				bool err;

				//First iteration
				if (sv_phantom.GMT == 0.0)
				{
					SV sv_P1 = coast_auto(sv_P_cur, sv_bef_table[i].GMT - sv_P_cur.GMT);
					err = FindCommonNode(sv_bef_table[i], sv_P1, add_constraint[i], dt);
				}
				else
				{
					err = FindCommonNode(sv_bef_table[i], sv_phantom, add_constraint[i], dt);
				}
				if (err) return 31;

				sv_bef_table[i] = coast_auto(sv_bef_table[i], dt);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::SEC_APS)
			{
				sv_bef_table[i] = FindNthApsidalCrossingAuto(sv_bef_table[i], tigmodifiers[i].value);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::ARG)
			{
				OrbMech::OELEMENTS coe;
				double u_b, DN, u_d;

				coe = OrbMech::coe_from_sv(sv_bef_table[i].R, sv_bef_table[i].V, mu);
				u_b = fmod(coe.TA + coe.w, PI2);
				u_d = tigmodifiers[i].value*RAD;
				if (u_b > u_d)
				{
					DN = 1.0;
				}
				else
				{
					DN = 0.0;
				}

				sv_bef_table[i] = GeneralTrajectoryPropagation(sv_bef_table[i], 2, u_d, DN, useNonSphericalGravity);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::LITI || tigmodifiers[i].type == OMPDefs::SECONDARIES::NITO)
			{
				sv_bef_table[i] = FindOrbitalSunriseRelativeTime(sv_bef_table[i], true, tigmodifiers[i].value);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::LITM)
			{
				sv_bef_table[i] = FindOrbitalMidnightRelativeTime(sv_bef_table[i], false, tigmodifiers[i].value);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::LITO || tigmodifiers[i].type == OMPDefs::SECONDARIES::NITI)
			{
				sv_bef_table[i] = FindOrbitalSunriseRelativeTime(sv_bef_table[i], false, tigmodifiers[i].value);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::NITM)
			{
				sv_bef_table[i] = FindOrbitalMidnightRelativeTime(sv_bef_table[i], true, tigmodifiers[i].value);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::OPT)
			{
				sv_bef_table[i] = FindOptimumNodeShiftPoint(sv_bef_table[i], add_constraint[i].x);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::LAT || tigmodifiers[i].type == OMPDefs::SECONDARIES::LON || tigmodifiers[i].type == OMPDefs::SECONDARIES::DEC || tigmodifiers[i].type == OMPDefs::SECONDARIES::ALT)
			{
				SV sv_temp;
				bool err;

				err = SEARMT(sv_bef_table[i], tigmodifiers[i].type, tigmodifiers[i].value, sv_temp);
				if (err) return 26; //Failure to find maneuver time
				sv_bef_table[i] = sv_temp;
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::ASC || tigmodifiers[i].type == OMPDefs::SECONDARIES::DSC || tigmodifiers[i].type == OMPDefs::SECONDARIES::U)
			{
				VECTOR3 N, H;
				double U, crossings, U_D;

				//Calculate current argument of latitude
				H = unit(crossp(sv_bef_table[i].R, sv_bef_table[i].V));
				N = unit(crossp(_V(0, 0, 1), H));
				U = PHSANG(sv_bef_table[i].R, sv_bef_table[i].V, N);
				if (U < 0)
				{
					U += PI2;
				}

				if (tigmodifiers[i].type == OMPDefs::SECONDARIES::ASC)
				{
					U_D = 0.0;
					crossings = round(tigmodifiers[i].value) - 1.0;
				}
				else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::DSC)
				{
					U_D = PI;
					crossings = round(tigmodifiers[i].value) - 1.0;
				}
				else
				{
					U_D = tigmodifiers[i].value;
					crossings = 0.0;
				}

				if (U > U_D)
				{
					crossings = crossings + 1.0;
				}

				sv_bef_table[i] = GeneralTrajectoryPropagation(sv_bef_table[i], 2, U_D, crossings, useNonSphericalGravity);
			}
		}

		//TIG has been calculated, now get target SV at TIG
		sv_P_table[i] = coast_auto(sv_P_cur, sv_bef_table[i].GMT - sv_P_cur.GMT);

		//ITERATORS
		for (unsigned l = 0;l < iterators.size();l++)
		{
			if (iterators[l].constr == i)
			{
				//NC Manever Iterator
				if (iterators[l].type == 1)
				{
					VECTOR3 R_REL, V_REL;
					SV SV_ACON = sv_bef_table[i];
					SV SV_PCON = sv_P_table[i];
					OrbMech::REL_COMP(true, SV_PCON.R, SV_PCON.V, SV_ACON.R, SV_ACON.V, R_REL, V_REL);

					iterstate[l].err = iterators[l].value - R_REL.x;

					if (abs(iterstate[l].err) > 10.0)
					{
						iterstate[l].converged = false;
						iterstate[l].dv = dv_table[iterators[l].man].x;
						OrbMech::ITER(iterstate[l].c_I, iterstate[l].s_F, iterstate[l].err, iterstate[l].p_H, iterstate[l].dv, iterstate[l].erro, iterstate[l].dvo);

						if (iterstate[l].s_F) return 20;	//Error 20: Too many iterations

						dv_table[iterators[l].man].x = iterstate[l].dv;

						//return to maneuver
						i = iterators[l].man;
						recycle = true;
						break;
					}
					else
					{
						iterstate[l].converged = true;
						iterstate[l].c_I = 0;
					}
				}
				//NH Maneuver Iterator
				if (iterators[l].type == 2)
				{
					//Calculate error
					VECTOR3 Rtemp, Vtemp;
					SV SV_ACON = sv_bef_table[i];
					SV SV_PCON = sv_P_table[i];

					u = unit(crossp(SV_PCON.R, SV_PCON.V));
					SV_ACON.R = unit(SV_ACON.R - u * dotp(SV_ACON.R, u))*length(SV_ACON.R);
					OrbMech::RADUP(SV_PCON.R, SV_PCON.V, SV_ACON.R, mu, Rtemp, Vtemp);

					iterstate[l].err = length(Rtemp) - length(SV_ACON.R) - iterators[l].value;

					if (abs(iterstate[l].err) > 10.0)
					{
						iterstate[l].converged = false;
						iterstate[l].dv = dv_table[iterators[l].man].x;
						OrbMech::ITER(iterstate[l].c_I, iterstate[l].s_F, iterstate[l].err, iterstate[l].p_H, iterstate[l].dv, iterstate[l].erro, iterstate[l].dvo);

						if (iterstate[l].s_F) return 20;	//Error 20: Too many iterations

						dv_table[iterators[l].man].x = iterstate[l].dv;

						//return to maneuver
						i = iterators[l].man;
						recycle = true;
						break;
					}
					else
					{
						iterstate[l].converged = true;
						iterstate[l].c_I = 0;
					}
				}
				//NPC Iterator
				if (iterators[l].type == 3)
				{
					SV SV_ACON = sv_bef_table[i];
					SV SV_PCON = sv_P_table[i];
					VECTOR3 H_A = crossp(SV_ACON.R, SV_ACON.V);
					VECTOR3 H_P = crossp(SV_PCON.R, SV_PCON.V);

					iterstate[l].err = OrbMech::acos2(dotp(H_A, H_P) / length(H_A) / length(H_P)) - iterators[l].value;

					if (abs(iterstate[l].err) > 0.0005*RAD)
					{
						SV sv_PH;
						iterstate[l].converged = false;

						//Generate phantom plane
						u = unit(H_P);
						sv_PH = SV_ACON;
						sv_PH.R = unit(SV_ACON.R - u * dotp(SV_ACON.R, u))*length(SV_ACON.R);
						sv_PH.V = unit(SV_ACON.V - u * dotp(SV_ACON.V, u))*length(SV_ACON.V);

						//Iterate backwards to NPC TIG
						for (unsigned m = iterators[l].constr - 1;m >= iterators[l].man;m--)
						{
							sv_PH = coast_auto(sv_PH, sv_bef_table[m].GMT - sv_PH.GMT);
							if (m > iterators[l].man)
							{
								DV = tmul(OrbMech::LVLH_Matrix(sv_PH.R, sv_PH.V), dv_table[m]);
								sv_PH.V -= DV;
							}
							else break;
						}

						sv_phantom = sv_PH;

						iterstate[l].c_I += 1.0;
						if (iterstate[l].c_I > 15) return 20; //Error 20: Too many iterations

						//return to maneuver
						/*i = iterators[l].man;
						recycle = true;
						break;*/
					}
					else
					{
						iterstate[l].converged = true;
						iterstate[l].c_I = 0;
					}
				}
			}
		}

		if (recycle) continue;

		//MANEUVER
		if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HA)
		{
			if (HeightManeuverAuto(sv_bef_table[i], R_E + add_constraint[i].x, true, DV))
			{
				return 28;	//HA maneuver failed to converge
			}
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HASH)
		{
			if (HeightManeuverAuto(sv_bef_table[i], R_E + add_constraint[i].x, false, DV))
			{
				return 29;	//HASH maneuver failed to converge
			}
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::EXDV || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NC || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NH)
		{
			DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NHRD)
		{
			double r_dot = dotp(sv_bef_table[i].R, sv_bef_table[i].V) / length(sv_bef_table[i].R);
			dv_table[i].z = -r_dot;
			DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOI)
		{
			double ddt;
			if (ManeuverConstraintsTable[i + 1].threshold == OMPDefs::THRESHOLD::THRES_T)
			{
				ddt = GMTfromGET(ManeuverConstraintsTable[i + 1].thresh_num) - sv_bef_table[i].GMT;
			}
			else if (ManeuverConstraintsTable[i + 1].threshold == OMPDefs::THRESHOLD::THRES_DT)
			{
				ddt = ManeuverConstraintsTable[i + 1].thresh_num;
			}
			else
			{
				return 23;	//No valid threshold for SOI/NCC
			}

			DV = SOIManeuver(sv_bef_table[i], sv_P_cur, sv_bef_table[i].GMT, ddt, add_constraint[i + 1]);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOR)
		{
			DV = SORManeuver(sv_bef_table[i], sv_P_cur, sv_bef_table[i].GMT, add_constraint[i]);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NPC)
		{
			VECTOR3 H_P;
			//First iteration
			if (add_constraint[i].x == 0.0)
			{
				SV sv_P1 = coast_auto(sv_P_cur, sv_bef_table[i].GMT - sv_P_cur.GMT);
				H_P = crossp(sv_P1.R, sv_P1.V);
			}
			else
			{
				H_P = add_constraint[i];
			}

			DV = NPCManeuver(sv_bef_table[i], H_P);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NCC)
		{
			double ddt;
			if (ManeuverConstraintsTable[i + 1].threshold == OMPDefs::THRESHOLD::THRES_T)
			{
				ddt = GMTfromGET(ManeuverConstraintsTable[i + 1].thresh_num) - sv_bef_table[i].GMT;
			}
			else if (ManeuverConstraintsTable[i + 1].threshold == OMPDefs::THRESHOLD::THRES_DT)
			{
				ddt = ManeuverConstraintsTable[i + 1].thresh_num;
			}
			else
			{
				return 23;	//No valid threshold for SOI/NCC
			}

			DV = SOIManeuver(sv_bef_table[i], sv_P_cur, sv_bef_table[i].GMT, ddt, add_constraint[i + 1]);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		//Apsidal Shift
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::APSO)
		{
			double r_dot = dotp(sv_bef_table[i].R, sv_bef_table[i].V) / length(sv_bef_table[i].R);
			dv_table[i] = _V(0, 0, -2.0*r_dot);
			DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
		}
		//Circularization
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::CIRC)
		{
			if (HeightManeuverAuto(sv_bef_table[i], length(sv_bef_table[i].R), false, DV))
			{
				return 30;	//CIRC maneuver failed to converge
			}
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::DVPY || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::DVYP)
		{
			double dv = add_constraint[i].x;
			double pit = add_constraint[i].y;
			double yaw = add_constraint[i].z;
			dv_table[i] = _V(dv*cos(pit)*cos(yaw), dv*sin(yaw), -dv * sin(pit)*cos(yaw));
			DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
		}
		//Node Shift setting up common node 90° later
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NS)
		{
			double Y_A_dot = CalculateYDot(sv_bef_table[i].V, sv_P_table[i].R, sv_P_table[i].V);
			dv_table[i] = _V(0, -Y_A_dot, 0);
			DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NSR)
		{
			dv_table[i] = NSRManeuver(sv_bef_table[i], sv_P_table[i]);
			DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NOSH)
		{
			DV = NodeShiftManeuver(sv_bef_table[i], add_constraint[i].x);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::PC)
		{
			DV = PlaneChangeManeuver(sv_bef_table[i], add_constraint[i].x);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}

		//Additional secondary maneuver constraints
		for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
		{
			if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "NULL") == 0)
			{
				double Y_A_dot = CalculateYDot(sv_bef_table[i].V, sv_P_table[i].R, sv_P_table[i].V);
				dv_table[i].y = -Y_A_dot;
				DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
			}
		}

		//Calculate SV after the maneuver
		sv_aft_table[i] = sv_bef_table[i];
		sv_aft_table[i].V += DV;

		
		i++;
		if (i == TAB && IsOMPConverged(iterstate, iterators.size()) == false)
		{
			i = 0;
		}
	} while (i < TAB || IsOMPConverged(iterstate, iterators.size()) == false);

	MANEUVER man;

	ManeuverTable.clear();

	for (i = 0;i < TAB;i++)
	{
		man.dV_LVLH = dv_table[i];
		sprintf_s(man.name, 10, ManeuverConstraintsTable[i].name);
		man.TIG_GMT = sv_bef_table[i].GMT;
		man.type = ManeuverConstraintsTable[i].type;

		ManeuverTable.push_back(man);
	}

	CalculateManeuverEvalTable(sv_A0, sv_P0);

	return 0;
}

void ShuttleFDOCore::CalculateManeuverEvalTable(SV sv_A0, SV sv_P0)
{
	SV sv_cur, sv_Pcur;
	VECTOR3 DV, u, R, Rtemp, Vtemp, R_REL, V_REL;
	double apo, peri, dt1, dt2;
	OBJHANDLE hSun;
	MANEVALDATA man;

	sv_cur = sv_A0;
	hSun = oapiGetObjectByName("hSun");

	for (unsigned i = 0;i < ManeuverTable.size();i++)
	{
		sv_cur = coast_auto(sv_cur, ManeuverTable[i].TIG_GMT - sv_cur.GMT);
		DV = tmul(OrbMech::LVLH_Matrix(sv_cur.R, sv_cur.V), ManeuverTable[i].dV_LVLH);
		sv_cur.V += DV;

		GetOPMManeuverType(man.type, ManeuverTable[i].type);
		sprintf_s(man.name, 10, ManeuverTable[i].name);
		man.DVMag = length(ManeuverTable[i].dV_LVLH) / 0.3048;
		man.GMTIG = ManeuverTable[i].TIG_GMT;
		man.METIG = GETfromGMT(ManeuverTable[i].TIG_GMT);
		if (i < ManeuverTable.size() - 1)
		{
			man.DT = ManeuverTable[i + 1].TIG_GMT - ManeuverTable[i].TIG_GMT;
		}
		else
		{
			man.DT = 0.0;
		}
		man.DV = ManeuverTable[i].dV_LVLH / 0.3048;

		if (useNonSphericalGravity)
		{
			ApsidesMagnitudeDetermination(sv_cur, apo, peri);
		}
		else
		{
			OrbMech::periapo(sv_cur.R, sv_cur.V, mu, apo, peri);
		}
		man.HA = (apo - R_E) / 1852.0;
		man.HP = (peri - R_E) / 1852.0;

		sv_Pcur = coast_auto(sv_P0, sv_cur.GMT - sv_P0.GMT);
		u = unit(crossp(sv_Pcur.R, sv_Pcur.V));
		R = unit(sv_cur.R - u * dotp(sv_cur.R, u))*length(sv_cur.R);
		OrbMech::RADUP(sv_Pcur.R, sv_Pcur.V, R, mu, Rtemp, Vtemp);
		man.DH = (length(Rtemp) - length(R)) / 1852.0;
		man.RANGE = length(sv_Pcur.R - sv_cur.R) / 1852.0;
		OrbMech::REL_COMP(true, sv_Pcur.R, sv_Pcur.V, sv_cur.R, sv_cur.V, R_REL, V_REL);
		man.PHASE = -R_REL.x / length(sv_Pcur.R)*DEG;
		man.Y = R_REL.y / 0.3048;
		man.Ydot = V_REL.y / 0.3048;

		dt1 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.GMT, BaseMJD, M_EFTOECL_AT_EPOCH, hEarth, hSun, true, true, true);
		dt2 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.GMT, BaseMJD, M_EFTOECL_AT_EPOCH, hEarth, hSun, false, true, true);

		if (dt1 < dt2)
		{
			man.noon = false;
			man.TTN = dt1;
		}
		else
		{
			man.noon = true;
			man.TTN = dt2;
		}

		dt1 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.GMT, BaseMJD, M_EFTOECL_AT_EPOCH, hEarth, hSun, true, false, true);
		dt2 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.GMT, BaseMJD, M_EFTOECL_AT_EPOCH, hEarth, hSun, false, false, true);

		if (dt1 < dt2)
		{
			man.sunrise = true;
			man.TTS = dt1;
		}
		else
		{
			man.sunrise = false;
			man.TTS = dt2;
		}

		ManeuverEvaluationTable.push_back(man);
	}
}

OMPDefs::MANTYPE ShuttleFDOCore::GetOPMManeuverType(char *buf)
{
	OMPDefs::MANTYPE man = OMPDefs::MANTYPE::NOMAN;

	if (strcmp(buf, "HA") == 0)
	{
		man = OMPDefs::HA;
	}
	else if (strcmp(buf, "HASH") == 0)
	{
		man = OMPDefs::HASH;
	}
	else if (strcmp(buf, "NC") == 0)
	{
		man = OMPDefs::NC;
	}
	else if (strcmp(buf, "EXDV") == 0)
	{
		man = OMPDefs::EXDV;
	}
	else if (strcmp(buf, "NH") == 0)
	{
		man = OMPDefs::NH;
	}
	else if (strcmp(buf, "SOI") == 0)
	{
		man = OMPDefs::SOI;
	}
	else if (strcmp(buf, "SOR") == 0)
	{
		man = OMPDefs::SOR;
	}
	else if (strcmp(buf, "NPC") == 0)
	{
		man = OMPDefs::NPC;
	}
	else if (strcmp(buf, "NCC") == 0)
	{
		man = OMPDefs::NCC;
	}
	else if (strcmp(buf, "APSO") == 0)
	{
		man = OMPDefs::APSO;
	}
	else if (strcmp(buf, "CIRC") == 0)
	{
		man = OMPDefs::CIRC;
	}
	else if (strcmp(buf, "NHRD") == 0)
	{
		man = OMPDefs::NHRD;
	}
	else if (strcmp(buf, "NSR") == 0)
	{
		man = OMPDefs::NSR;
	}
	else if (strcmp(buf, "NOSH") == 0)
	{
		man = OMPDefs::NOSH;
	}
	else if (strcmp(buf, "PC") == 0)
	{
		man = OMPDefs::PC;
	}

	return man;
}

void ShuttleFDOCore::GetOPMManeuverType(char *buf, OMPDefs::MANTYPE type)
{
	if (type == OMPDefs::MANTYPE::HA)
	{
		sprintf_s(buf, 100, "HA");
	}
	else if (type == OMPDefs::MANTYPE::HASH)
	{
		sprintf_s(buf, 100, "HASH");
	}
	else if (type == OMPDefs::MANTYPE::NC)
	{
		sprintf_s(buf, 100, "NC");
	}
	else if (type == OMPDefs::MANTYPE::EXDV)
	{
		sprintf_s(buf, 100, "EXDV");
	}
	else if (type == OMPDefs::MANTYPE::NH)
	{
		sprintf_s(buf, 100, "NH");
	}
	else if (type == OMPDefs::MANTYPE::SOI)
	{
		sprintf_s(buf, 100, "SOI");
	}
	else if (type == OMPDefs::MANTYPE::SOR)
	{
		sprintf_s(buf, 100, "SOR");
	}
	else if (type == OMPDefs::MANTYPE::NPC)
	{
		sprintf_s(buf, 100, "NPC");
	}
	else if (type == OMPDefs::MANTYPE::NCC)
	{
		sprintf_s(buf, 100, "NCC");
	}
	else if (type == OMPDefs::MANTYPE::APSO)
	{
		sprintf_s(buf, 100, "APSO");
	}
	else if (type == OMPDefs::MANTYPE::CIRC)
	{
		sprintf_s(buf, 100, "CIRC");
	}
	else if (type == OMPDefs::MANTYPE::NHRD)
	{
		sprintf_s(buf, 100, "NHRD");
	}
	else if (type == OMPDefs::MANTYPE::NSR)
	{
		sprintf_s(buf, 100, "NSR");
	}
	else if (type == OMPDefs::MANTYPE::NOSH)
	{
		sprintf_s(buf, 100, "NOSH");
	}
	else if (type == OMPDefs::MANTYPE::PC)
	{
		sprintf_s(buf, 100, "PC");
	}
	else
	{
		sprintf_s(buf, 100, "N/A");
	}

}

VECTOR3 ShuttleFDOCore::NPCManeuver(SV sv_A, VECTOR3 H_P)
{
	VECTOR3 u1, u2, DV_PC_LV, V2;
	double Y_C_dot;

	u1 = unit(crossp(sv_A.V, sv_A.R));
	u2 = -unit(H_P);

	Y_C_dot = dotp(sv_A.V, u2);
	DV_PC_LV = _V(0, -Y_C_dot, 0);

	//return tmul(OrbMech::LVLH_Matrix(sv_A.R, sv_A.V), DV_PC_LV);

	V2 = sv_A.V + tmul(OrbMech::LVLH_Matrix(sv_A.R, sv_A.V), DV_PC_LV);
	V2 = unit(V2)*length(sv_A.V);
	return V2 - sv_A.V;
}

bool ShuttleFDOCore::FindCommonNode(SV sv_A, SV sv_P, VECTOR3 &u_d, double &dt)
{
	SV sv_A1, sv_P1;
	VECTOR3 H_A, H_P, C_N;
	double dtheta, ddt, eps_C;
	int C, CMAX;

	sv_A1 = sv_A;
	sv_P1 = GeneralTrajectoryPropagation(sv_P, 0, sv_A1.GMT, 0.0, useNonSphericalGravity);
	C = 0;
	dt = 0.0;

	CMAX = 20;
	eps_C = 0.0115*RAD;

	do
	{
		sv_P1 = PositionMatch(sv_P1, sv_A1);

		H_A = unit(crossp(sv_A1.R, sv_A1.V));
		H_P = unit(crossp(sv_P1.R, sv_P1.V));
		C_N = crossp(H_A, H_P);

		if (C == 0)
		{
			VECTOR3 T_EST = crossp(sv_A1.R, C_N);
			if (dotp(T_EST, H_A) < 0)
			{
				C_N = -C_N;
			}
			dtheta = -OrbMech::PHSANG(sv_A1.R, sv_A1.V, C_N);
		}
		else
		{
			double dtheta1, dtheta2;

			dtheta1 = -OrbMech::PHSANG(sv_A1.R, sv_A1.V, C_N);
			dtheta2 = -OrbMech::PHSANG(sv_A1.R, sv_A1.V, -C_N);
			if (abs(dtheta1) < abs(dtheta2))
			{
				dtheta = dtheta1;
			}
			else
			{
				dtheta = dtheta2;
			}
		}
		ddt = OrbMech::time_theta(sv_A1.R, sv_A1.V, dtheta, mu, false);
		sv_A1 = coast_auto(sv_A1, ddt);
		dt += ddt;
		C++;
		if (useNonSphericalGravity == false) break;
	} while (abs(dtheta) >= eps_C && C < CMAX);

	u_d = H_P;
	if (C >= CMAX)
	{
		return true;
	}
	return false;
}

VECTOR3 ShuttleFDOCore::NSRManeuver(SV sv_A, SV sv_P)
{
	//Assume both SVs are at TIG
	VECTOR3 R_P2, V_P2, R_A2, V_A2, R_PC, V_PC, u, V_A2_apo, DV2, X, Y, Z;
	double DH_CDH, r_PC, r_A2, v_PV, a_A, a_P, v_A2, v_AV, v_AH;

	R_A2 = sv_A.R;
	r_A2 = length(R_A2);
	V_A2 = sv_A.V;
	v_A2 = length(V_A2);
	R_P2 = sv_P.R;
	V_P2 = sv_P.V;
	u = unit(crossp(R_P2, V_P2));
	R_A2 = unit(R_A2 - u * dotp(R_A2, u))*r_A2;
	V_A2 = unit(V_A2 - u * dotp(V_A2, u))*v_A2;

	OrbMech::RADUP(R_P2, V_P2, R_A2, mu, R_PC, V_PC);
	a_P = OrbMech::GetSemiMajorAxis(R_PC, V_PC, mu);

	r_PC = length(R_PC);
	DH_CDH = r_PC - r_A2;
	v_PV = dotp(V_PC, R_A2) / r_A2;
	a_A = a_P - DH_CDH;
	v_AV = v_PV * pow(a_P / a_A, 1.5);
	v_AH = sqrt(mu*(2.0 / r_A2 - 1.0 / a_A) - v_AV * v_AV);
	V_A2_apo = unit(crossp(u, R_A2))*v_AH + unit(R_A2)*v_AV;
	DV2 = V_A2_apo - V_A2;
	Z = unit(-R_A2);
	Y = -u;
	X = unit(crossp(Y, Z));
	return tmul(_M(X.x, Y.x, Z.x, X.y, Y.y, Z.y, X.z, Y.z, Z.z), DV2);
}

SV ShuttleFDOCore::coast_auto(SV sv0, double dt)
{
	if (useNonSphericalGravity)
	{
		return coast(sv0, dt);
	}
	else
	{
		return coast_osc(sv0, dt, mu);
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

int ShuttleFDOCore::subThread()
{
	//Do nothing if mission not initialized
	if (BaseMJD == 0.0)
	{
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
		OMPErrorCode = 0;
		OMPErrorCode = CalculateOMPPlan();

		Result = 0;
	}
	break;
	case 2: //Launch Window/Targeting Processor
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
		LWP_Settings.lwp_param_table = &LWP_Parameters;

		LWP.Init(LWP_Settings);
		LWP.LWP();
		LWP.GetOutput(LPW_Summary);

		//No error
		if (LPW_Summary.LWPERROR == 0)
		{
			if (LWP_Settings.LW == 0)
			{
				InPlaneGMT = LPW_Summary.TPLANE;
			}
			else
			{
				InPlaneGMT = LPW_Summary.GMTLO;
			}

			LWP_PlanarOpenGMT = LWP_Parameters.GMTLO[0];
			LWP_PlanarCloseGMT = LWP_Parameters.GMTLO[1];

			//Post insertion state vector processing
			SV sv_P1, sv_P2;

			sv_P1 = coast_auto(LWP.LWPSV.sv_P, DTIG_ET_SEP);
			sv_P1.V += tmul(LVLH_Matrix(sv_P1.R, sv_P1.V), DV_ET_SEP);

			sv_P2 = coast_auto(sv_P1, DTIG_MPS - DTIG_ET_SEP);
			sv_P2.V += tmul(LVLH_Matrix(sv_P2.R, sv_P2.V), DV_MPS);
			sv_P2.mass = LWP_Settings.CWHT;

			//Store SV
			sv_chaser = sv_P2;
			chaserSVOption = true;

			//Save launch time
			int hh, mm;
			double ss;

			OrbMech::days2hms(InPlaneGMT / 24.0 / 3600.0, hh, mm, ss);
			SetLaunchTime(hh, mm, ss);
		}


		Result = 0;
	}
	break;
	case 3: //Deorbit Opportunities
	{
		LandingOpportunitiesProcessor lop;
		LOPTInput opt;

		std::ifstream myfile;
		myfile.open(".\\Config\\MFD\\ShuttleFDOMFD\\LandingSites.txt");
		if (myfile.is_open())
		{
			char Buffer[128];
			LOPTSite temp;
			opt.sites.clear();

			std::string line;
			while (std::getline(myfile, line))
			{
				if (sscanf(line.c_str(), "%s %lf %lf %lf %d", Buffer, &temp.lat, &temp.lng, &temp.rad, &temp.timezone) == 5)
				{
					temp.name.assign(Buffer);
					temp.lat *= RAD;
					temp.lng *= RAD;
					opt.sites.push_back(temp);
				}
			}

			if (opt.sites.size() > 0)
			{
				opt.sv_in = StateVectorCalc(vessel);
				opt.GETS = DOPS_GETS;
				opt.GETF = DOPS_GETF;
				opt.INORB = DOPS_InitialRev;
				opt.SVPROP = useNonSphericalGravity;
				opt.GMTR = LaunchGMT;
				opt.XRNG = DOPS_MaxXRNG;
				opt.BaseMJD = BaseMJD;
				opt.RM = M_EFTOECL_AT_EPOCH;

				lop.LOPT(opt, DODS_Output);
			}
		}
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

bool ShuttleFDOCore::MET2MTT()
{
	if (ManeuverEvaluationTable.size() < 1) return false;

	ManeuverTransferTable.clear();

	MANTRANSDATA man;

	for (unsigned i = 0;i < ManeuverEvaluationTable.size();i++)
	{
		man.MNVR = i + 1;
		sprintf_s(man.NAME, ManeuverEvaluationTable[i].type, 4);
		sprintf_s(man.COMMENT, ManeuverEvaluationTable[i].name, 10);
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

void ShuttleFDOCore::ExecuteMTT()
{
	//Sanity checks
	if (ManeuverTransferTable.size() < 1) return;
	if (ManeuverEvaluationTable.size() < 1) return;
	if (ManeuverTransferTable.size() != ManeuverEvaluationTable.size()) return;
	if (sv_chaser.GMT == 0.0) return;

	DMTInputTable.clear();

	DMTINPUT man;
	SV sv_cur, sv_tig, sv_cut;
	VECTOR3 DV_iner;
	double dt, F, isp, dv, dt_burn, W_dot;

	sv_cur = sv_chaser;

	for (unsigned i = 0;i < ManeuverTransferTable.size();i++)
	{
		dt = ManeuverEvaluationTable[i].GMTIG - sv_cur.GMT;
		sv_cur = coast_auto(sv_cur, dt);

		GetThrusterData(ManeuverTransferTable[i].thrusters, F, isp);
		W_dot = F / isp;

		dv = ManeuverEvaluationTable[i].DVMag*0.3048;
		dt_burn = sv_cur.mass / W_dot * (1.0 - exp(-dv * W_dot / F));

		DV_iner = tmul(OrbMech::LVLH_Matrix(sv_cur.R, sv_cur.V), ManeuverEvaluationTable[i].DV*0.3048);
		man.DV_iner = DV_iner;

		sv_cut = sv_cur;
		sv_cut.mass -= W_dot * dt_burn;
		//TBD: ITER support
		sv_cut.V += DV_iner;

		//Very small burn, bypass logic
		if (ManeuverEvaluationTable[i].DVMag < 0.1)
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
				A = S * log(sv_cur.mass / sv_cut.mass);
				B = (sv_cur.mass / W_dot)*A - S * dt_burn;
				dt_tig = -B / A;

				sv_tig = coast_auto(sv_cur, dt_tig);
			}
			else
			{
				sv_tig = sv_cur;
			}
		}

		man.sv_tig = sv_tig;
		sv_cur = sv_cut;

		man.TV_ROLL = ManeuverTransferTable[i].ROLL;
		man.thrusters = ManeuverTransferTable[i].thrusters;
		sprintf_s(man.comment, ManeuverTransferTable[i].COMMENT);

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
	GetDMTManeuverID(Buffer2, input.comment);

	sprintf_s(DMT.CODE, "%sE%02d%s", Buffer, DMT_MNVR, Buffer2);
	DMT.TV_ROLL = input.TV_ROLL*DEG;
	if (input.thrusters == OMPDefs::THRUSTERS::OBP)
	{
		OMSTVC(_V(1071.75429, 0.0, 364.71665), true, P, LY, RY);

		DMT.TRIMS_P = P * DEG;
		DMT.TRIMS_LY = LY * DEG;
		DMT.TRIMS_RY = RY * DEG;

		p_T = P - 15.82*RAD;
		y_T = 0.0;
	}
	else if (input.thrusters == OMPDefs::THRUSTERS::OL || input.thrusters == OMPDefs::THRUSTERS::OR)
	{
		OMSTVC(_V(1071.75429, 0.0, 364.71665), false, P, LY, RY);

		DMT.TRIMS_P = P * DEG;
		DMT.TRIMS_LY = LY * DEG;
		DMT.TRIMS_RY = RY * DEG;

		if (input.thrusters == OMPDefs::THRUSTERS::OL)
		{
			p_T = P - 15.82*RAD;
			y_T = LY + 6.5*RAD;
		}
		else
		{
			p_T = P - 15.82*RAD;
			y_T = RY - 6.5*RAD;
		}
	}
	else
	{
		DMT.TRIMS_P = 0.0;
		DMT.TRIMS_LY = 0.0;
		DMT.TRIMS_RY = 0.0;

		p_T = 0.0;
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
	DV_LVLH = mul(Rot, input.DV_iner)*MPS2FPS;

	// Round this, so that it agrees with the input for the Shuttle computer
	DV_LVLH.x = round(DV_LVLH.x*10.0) / 10.0;
	DV_LVLH.y = round(DV_LVLH.y*10.0) / 10.0;
	DV_LVLH.z = round(DV_LVLH.z*10.0) / 10.0;
	//Actual inertial DV vector
	DV_iner_act = tmul(Rot, DV_LVLH) / MPS2FPS;

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
	DMT.DVTOT = DMT.DV_M = length(DV_iner_act)*MPS2FPS;

	GetThrusterData(input.thrusters, F, isp);
	DMT.TGO = isp / F * input.sv_tig.mass*(1.0 - exp(-length(DV_iner_act) / isp));
	DMT.VGO = u_A*DMT.DVTOT;

	sv_cut = PoweredFlightProcessor(input.sv_tig, DV_iner_act, F, isp, useNonSphericalGravity);

	double apo, peri;
	OrbMech::periapo(sv_cut.R, sv_cut.V, mu, apo, peri);

	DMT.TGT_HA = (apo - R_E) / 1852.0;
	DMT.TGT_HP = (peri - R_E) / 1852.0;
}

void ShuttleFDOCore::GetThrusterData(OMPDefs::THRUSTERS type, double &F, double &isp)
{
	if (type == OMPDefs::THRUSTERS::OBP)
	{
		F = 2.0*OMS_THRUST;
		isp = OMS_ISP0;
	}
	else if (type == OMPDefs::THRUSTERS::OL || type == OMPDefs::THRUSTERS::OR)
	{
		F = OMS_THRUST;
		isp = OMS_ISP0;
	}
	else
	{
		isp = RCS_ISP0;
		if (type == OMPDefs::THRUSTERS::PX2) F = 2.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::PX3) F = 3.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::PX3) F = 4.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::MXL) F = 2.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::YL) F = 2.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::MYL) F = 2.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::ZH) F = 3.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::ZL) F = 4.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::MZL) F = 6.0*RCS_THRUST;
		else if (type == OMPDefs::THRUSTERS::MZH) F = 2.0*RCS_THRUST;
		else F = 2.0*RCS_THRUST;
	}
}

void ShuttleFDOCore::GetMTTThrusterType(char *buf, OMPDefs::THRUSTERS type)
{
	if (type == OMPDefs::THRUSTERS::PX4)
	{
		sprintf_s(buf, 100, "PX4");
	}
	else if (type == OMPDefs::THRUSTERS::PX3)
	{
		sprintf_s(buf, 100, "PX3");
	}
	else if (type == OMPDefs::THRUSTERS::PX2)
	{
		sprintf_s(buf, 100, "PX2");
	}
	else if (type == OMPDefs::THRUSTERS::MXL)
	{
		sprintf_s(buf, 100, "MXL");
	}
	else if (type == OMPDefs::THRUSTERS::YL)
	{
		sprintf_s(buf, 100, "YL");
	}
	else if (type == OMPDefs::THRUSTERS::MYL)
	{
		sprintf_s(buf, 100, "MYL");
	}
	else if (type == OMPDefs::THRUSTERS::ZH)
	{
		sprintf_s(buf, 100, "ZH");
	}
	else if (type == OMPDefs::THRUSTERS::ZL)
	{
		sprintf_s(buf, 100, "ZL");
	}
	else if (type == OMPDefs::THRUSTERS::MZH)
	{
		sprintf_s(buf, 100, "MZH");
	}
	else if (type == OMPDefs::THRUSTERS::M1)
	{
		sprintf_s(buf, 100, "M1");
	}
	else if (type == OMPDefs::THRUSTERS::M2)
	{
		sprintf_s(buf, 100, "M2");
	}
	else if (type == OMPDefs::THRUSTERS::OL)
	{
		sprintf_s(buf, 100, "OL");
	}
	else if (type == OMPDefs::THRUSTERS::OR)
	{
		sprintf_s(buf, 100, "OR");
	}
	else if (type == OMPDefs::THRUSTERS::OBP)
	{
		sprintf_s(buf, 100, "OBP");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOCore::GetDMTThrusterType(char *buf, OMPDefs::THRUSTERS type)
{
	if (type == OMPDefs::THRUSTERS::PX4 || type == OMPDefs::THRUSTERS::PX3)
	{
		sprintf_s(buf, 100, "XH");
	}
	else if (type == OMPDefs::THRUSTERS::PX2)
	{
		sprintf_s(buf, 100, "XL");
	}
	else  if (type == OMPDefs::THRUSTERS::ZL)
	{
		sprintf_s(buf, 100, "ZL");
	}
	else if (type == OMPDefs::THRUSTERS::MXL)
	{
		sprintf_s(buf, 100, "MX");
	}
	else if (type == OMPDefs::THRUSTERS::YL)
	{
		sprintf_s(buf, 100, "YL");
	}
	else if (type == OMPDefs::THRUSTERS::MYL)
	{
		sprintf_s(buf, 100, "MY");
	}
	else if (type == OMPDefs::THRUSTERS::ZH)
	{
		sprintf_s(buf, 100, "ZH");
	}
	else if (type == OMPDefs::THRUSTERS::MZH)
	{
		sprintf_s(buf, 100, "ZM");
	}
	else if (type == OMPDefs::THRUSTERS::M1)
	{
		sprintf_s(buf, 100, "M1");
	}
	else if (type == OMPDefs::THRUSTERS::M2)
	{
		sprintf_s(buf, 100, "M2");
	}
	else if (type == OMPDefs::THRUSTERS::OL)
	{
		sprintf_s(buf, 100, "OL");
	}
	else if (type == OMPDefs::THRUSTERS::OR)
	{
		sprintf_s(buf, 100, "OR");
	}
	else if (type == OMPDefs::THRUSTERS::OBP)
	{
		sprintf_s(buf, 100, "BP");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOCore::GetDMTManeuverID(char *buf, char *name)
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
	launchdate[0] = Y;
	launchdate[1] = D;

	//Calculate base MJD
	BaseMJD = OrbMech::Date2MJD(Y, D, 0, 0, 0.0);
	//Calculate rotation matrix
	M_EFTOECL_AT_EPOCH = OrbMech::GetRotationMatrix(BaseMJD);
	//Calculate matrix from TEG to M50
	MATRIX3 M_M50TOECL = OrbMech::GetObliquityMatrix(33281.923357);
	M_TEGTOECL = mul(OrbMech::tmat(M_M50TOECL), M_EFTOECL_AT_EPOCH);
	//sprintf(oapiDebugString(), "%f", BaseMJD);
}

void ShuttleFDOCore::SetLaunchTime(int H, int M, double S)
{
	launchdate[2] = H;
	launchdate[3] = M;
	launchdateSec = S;

	LaunchGMT = 3600.0*(double)H + 60.0*(double)M + S;
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
		P = 15.82*RAD - atan2(C, A);
		LY = -6.5*RAD + atan2(D, R1);
		RY = 6.5*RAD + atan2(B, R1);
	}
	else
	{
		P = 15.82*RAD - atan2(C, A);
		LY = -5.7*RAD + atan2(CG.y, R1);
		RY = 5.7*RAD + atan2(CG.y, R1);
	}
}

SV ShuttleFDOCore::timetoapo_auto(SV sv_A, double revs)
{
	SV sv_out;
	if (useNonSphericalGravity)
	{
		double v_r = dotp(sv_A.R, sv_A.V) / length(sv_A.R);
		if (v_r > 0)
		{
			sv_out = GeneralTrajectoryPropagation(sv_A, 1, PI, revs - 1.0, useNonSphericalGravity);
		}
		else
		{
			sv_out = GeneralTrajectoryPropagation(sv_A, 1, PI, revs, useNonSphericalGravity);
		}
	}
	else
	{
		double dt1 = 0.0;
		if (revs > 1.0)
		{
			double T_P = OrbMech::period(sv_A.R, sv_A.V, mu);
			dt1 = T_P * (revs - 1.0);

		}
		double dt2 = OrbMech::timetoapo(sv_A.R, sv_A.V, mu, 1);
		sv_out = coast_auto(sv_A, dt1 + dt2);
	}

	return sv_out;
}

bool ShuttleFDOCore::IsOMPConverged(ITERSTATE *iters, int size)
{
	if (size > 0)
	{
		for (int i = 0;i < size;i++)
		{
			if (iters[i].converged == false) return false;
		}
		return true;
	}

	return true;
}

SV ShuttleFDOCore::AEG(SV sv0, int opt, double dval, double DN)
{
	SV sv1 = sv0;
	OrbMech::AEGServiceRoutine(sv0.R, sv0.V, sv0.GMT, opt, dval, DN, sv1.R, sv1.V, sv1.GMT);
	return sv1;
}

SV ShuttleFDOCore::DeltaOrbitsAuto(SV sv0, double M)
{
	if (useNonSphericalGravity)
	{
		return GeneralTrajectoryPropagation(sv0, 3, 0.0, M, useNonSphericalGravity);
	}
	else
	{
		double P = OrbMech::period(sv0.R, sv0.V, mu);
		double dt = P * M;
		return coast_osc(sv0, dt, mu);
	}
}

SV ShuttleFDOCore::FindNthApsidalCrossingAuto(SV sv0, double N)
{
	int M = (int)N;
	bool even = (M % 2) == 0;
	double fact;
	if (even)
	{
		fact = -1.0;
	}
	else
	{
		fact = 1.0;
	}
	double v_r = dotp(sv0.R, sv0.V) / length(sv0.R);
	
	if (useNonSphericalGravity)
	{
		double DN;

		if (v_r > 0)
		{
			DN = (double)((M - 1) / 2);
		}
		else
		{
			DN = (double)(M / 2);
		}

		if (fact*v_r > 0)
		{
			//Apoapsis
			return GeneralTrajectoryPropagation(sv0, 1, PI, DN, useNonSphericalGravity);
		}
		else
		{
			//Periapsis
			return GeneralTrajectoryPropagation(sv0, 1, 0, DN, useNonSphericalGravity);
		}
	}
	else
	{
		double dt;
		if (v_r > 0)
		{
			dt = OrbMech::timetoapo(sv0.R, sv0.V, mu, 1);
		}
		else
		{
			dt = OrbMech::timetoperi(sv0.R, sv0.V, mu, 1);
		}
		double P = OrbMech::period(sv0.R, sv0.V, mu);
		return coast_osc(sv0, dt + 0.5*P * (N - 1.0), mu);
	}
}

double ShuttleFDOCore::CalculateYDot(VECTOR3 V_A, VECTOR3 R_P, VECTOR3 V_P)
{
	VECTOR3 u2 = unit(crossp(V_P, R_P));
	double Y_A_dot = dotp(V_A, u2);
	return Y_A_dot;
}

SV ShuttleFDOCore::PoweredFlightProcessor(SV sv_tig, VECTOR3 DV_iner, double f_T, double v_ex, bool nonspherical)
{
	SV sv_cut;
	double t_go;

	OrbMech::poweredflight(sv_tig.R, sv_tig.V, f_T, v_ex, sv_tig.mass, DV_iner, nonspherical, sv_cut.R, sv_cut.V, sv_cut.mass, t_go);
	sv_cut.GMT = sv_tig.GMT + t_go;
	return sv_cut;
}

SV ShuttleFDOCore::FindOrbitalSunriseRelativeTime(SV sv0, bool sunrise, double dt1)
{
	SV sv1;
	OBJHANDLE hSun;
	double dt2;

	hSun = oapiGetObjectByName("hSun");

	dt2 = OrbMech::sunrise(sv0.R, sv0.V, sv0.GMT, BaseMJD, M_EFTOECL_AT_EPOCH, hEarth, hSun, sunrise, false, true);
	sv1 = coast_auto(sv0, dt1 + dt2);
	return sv1;
}

SV ShuttleFDOCore::FindOrbitalMidnightRelativeTime(SV sv0, bool midnight, double dt1)
{
	SV sv1;
	OBJHANDLE hSun;
	double dt2;

	hSun = oapiGetObjectByName("hSun");

	dt2 = OrbMech::sunrise(sv0.R, sv0.V, sv0.GMT, BaseMJD, M_EFTOECL_AT_EPOCH, hEarth, hSun, midnight, true, true);
	sv1 = coast_auto(sv0, dt1 + dt2);
	return sv1;
}

bool ShuttleFDOCore::FindSVAtElevation(SV sv_A, SV sv_P, double t_guess, double elev_D, SV &sv_A2)
{
	SV sv_A1, sv_P1;
	double c_I, p_H, dt, elev, e_H, e_Ho, eps2, dto;
	int s_F;

	eps2 = 0.01*RAD;
	p_H = c_I = 0.0;
	s_F = 0;

	dt = t_guess - GETfromGMT(sv_A.GMT);
	sv_A1 = coast_auto(sv_A, dt);
	dt = t_guess - GETfromGMT(sv_P.GMT);
	sv_P1 = coast_auto(sv_P, dt);
	dt = 0.0;
	dto = 0.0;

	do
	{
		sv_A1 = coast_auto(sv_A1, dt - dto);
		sv_P1 = coast_auto(sv_P1, dt - dto);

		elev = OrbMech::COMELE(sv_A1.R, sv_A1.V, sv_P1.R);
		e_H = elev_D - elev;

		if (abs(e_H) >= eps2)
		{
			OrbMech::ITER(c_I, s_F, e_H, p_H, dt, e_Ho, dto);
			if (s_F == 1)
			{
				return false;
			}
		}
	} while (abs(e_H) >= eps2);

	sv_A2 = sv_A1;
	return true;
}

bool ShuttleFDOCore::HeightManeuverAuto(SV sv_A, double r_D, bool horizontal, VECTOR3 &DV)
{
	SV sv_A_apo, sv_D;
	OrbMech::OELEMENTS coe;
	VECTOR3 am, U_hor;
	double eps, p_H, c_I, u_b, u_d, DN, v_H, e_H, e_Ho, v_Ho;
	int s_F;

	sv_A_apo = sv_A;

	//Some helping vectors
	am = unit(crossp(sv_A.R, sv_A.V));
	U_hor = unit(crossp(am, unit(sv_A.R)));

	//Tolerance
	eps = 250.0*0.3048;
	//Set up iterator
	p_H = c_I = 0.0;
	s_F = 0;

	//Set up desired argument of latitude (only used with non-spherical gravity)
	coe = OrbMech::coe_from_sv(sv_A.R, sv_A.V, mu);
	u_b = fmod(coe.TA + coe.w, PI2);
	u_d = u_b + PI;
	if (u_d < PI2)
	{
		DN = 0.0;
	}
	else
	{
		u_d = u_d - PI2;
		DN = 1.0;
	}

	//Initial guess
	if (horizontal)
	{
		v_H = 0.0;
	}
	else
	{
		v_H = sqrt(2.0*mu / (length(sv_A.R)*(1.0 + length(sv_A.R) / r_D)));
	}

	do
	{
		if (horizontal)
		{
			sv_A_apo.V = sv_A.V + U_hor * v_H;
		}
		else
		{
			sv_A_apo.V = U_hor * v_H;
		}

		if (useNonSphericalGravity)
		{
			sv_D = GeneralTrajectoryPropagation(sv_A_apo, 2, u_d, DN, useNonSphericalGravity);
		}
		else
		{
			double dt_temp;
			OrbMech::REVUP(sv_A_apo.R, sv_A_apo.V, 0.5, mu, sv_D.R, sv_D.V, dt_temp);
		}

		e_H = length(sv_D.R) - r_D;
		if (abs(e_H) >= eps)
		{
			OrbMech::ITER(c_I, s_F, e_H, p_H, v_H, e_Ho, v_Ho);
			if (s_F == 1)
			{
				//Error
				return true;
			}
		}
	} while (abs(e_H) >= eps);

	DV = sv_A_apo.V - sv_A.V;
	return false;
}

SV ShuttleFDOCore::FindOptimumNodeShiftPoint(SV sv0, double dh)
{
	SV sv1;
	OrbMech::InvariantElements coe;
	double U_D, U_B, DN;

	coe = OrbMech::CalculateInvariantElementsBlock(sv0, mu, 0.0, useNonSphericalGravity);
	U_B = fmod(OrbMech::MeanToTrueAnomaly(coe.l, coe.e) + coe.g, PI2);

	U_D = atan2(1.0 + cos(dh), -sin(dh)*cos(coe.i));
	if (U_D < 0)
	{
		U_D += PI2;
	}

	if (U_B < U_D)
	{
		DN = 0.0;
	}
	else
	{
		if (U_B < U_D + PI)
		{
			U_D = U_D + PI;
			DN = 0.0;
		}
		else
		{
			DN = 1.0;
		}
	}
	if (useNonSphericalGravity)
	{
		sv1 = GeneralTrajectoryPropagation(sv0, 2, U_D, DN, useNonSphericalGravity);
	}
	else
	{
		double dt = OrbMech::time_theta(sv0.R, sv0.V, U_D + DN * PI2 - U_B, mu);
		sv1 = sv0;
		OrbMech::rv_from_r0v0(sv0.R, sv0.V, dt, sv1.R, sv1.V, mu);
		sv1.GMT = sv0.GMT + dt;
	}
	return sv1;
}

VECTOR3 ShuttleFDOCore::NodeShiftManeuver(SV sv0, double dh_D)
{
	OrbMech::CELEMENTS coe_b, coe_a;
	VECTOR3 Rtemp, Vtemp;
	double cos_u_a, sin_u_a, u_b, u_a, cos_dw, sin_dw, dw, f_a, f_b;

	coe_b = OrbMech::CartesianToKeplerian(sv0.R, sv0.V, mu);
	f_b = OrbMech::MeanToTrueAnomaly(coe_b.l, coe_b.e);
	u_b = fmod(f_b + coe_b.g, PI2);

	coe_a = coe_b;
	f_a = f_b;

	cos_u_a = cos(dh_D)*cos(u_b) + sin(dh_D)*sin(u_b)*cos(coe_b.i);
	sin_u_a = sqrt(1.0 - cos_u_a * cos_u_a);
	if (u_b > PI)
	{
		sin_u_a = -sin_u_a;
	}
	u_a = atan2(sin_u_a, cos_u_a);
	if (u_a < 0)
	{
		u_a = PI2;
	}
	cos_dw = (cos(dh_D) - cos(u_b)*cos(u_a)) / (sin(u_b)*sin(u_a));
	sin_dw = sin(dh_D)*sin(coe_b.i) / sin_u_a;
	dw = atan2(sin_dw, cos_dw);
	coe_a.i = acos(cos(coe_b.i)*cos_dw - sin(coe_b.i)*sin_dw*cos(u_b));
	coe_a.h = coe_b.h + dh_D;
	coe_a.h = fmod(coe_a.h, PI2);
	coe_a.g = u_a - f_a;
	if (coe_a.h < 0)
	{
		coe_a.h += PI2;
	}
	if (coe_a.g < 0)
	{
		coe_a.g += PI2;
	}
	OrbMech::KeplerianToCartesian(coe_a, mu, Rtemp, Vtemp);

	return Vtemp - sv0.V;
}

VECTOR3 ShuttleFDOCore::PlaneChangeManeuver(SV sv0, double dw_D)
{
	OrbMech::CELEMENTS coe_b, coe_a;
	VECTOR3 Rtemp, Vtemp;
	double f_b, u_b, sin_dh, cos_dh, dh, sin_u_a, cos_u_a, u_a, f_a;

	coe_b = OrbMech::CartesianToKeplerian(sv0.R, sv0.V, mu);
	f_b = OrbMech::MeanToTrueAnomaly(coe_b.l, coe_b.e);
	u_b = fmod(f_b + coe_b.g, PI2);

	coe_a = coe_b;
	f_a = f_b;

	coe_a.i = acos(cos(coe_b.i)*cos(dw_D) - sin(coe_b.i)*sin(dw_D)*cos(u_b));
	sin_dh = sin(dw_D)*sin(u_b) / sin(coe_a.i);
	cos_dh = (cos(dw_D) - cos(coe_b.i)*cos(coe_a.i)) / (sin(coe_b.i)*sin(coe_a.i));
	dh = atan2(sin_dh, cos_dh);
	coe_a.h = coe_b.h + dh;
	if (coe_a.h < 0)
	{
		coe_a.h += PI2;
	}
	else if (coe_a.h >= PI2)
	{
		coe_a.h -= PI2;
	}
	sin_u_a = sin(u_b)*sin(coe_b.i) / sin(coe_a.i);
	cos_u_a = cos(u_b)*cos(dh) + sin(u_b)*sin(dh)*cos(coe_b.i);
	u_a = atan2(sin_u_a, cos_u_a);
	if (u_a < 0)
	{
		u_a += PI2;
	}
	coe_a.g = u_a - f_a;
	if (coe_a.g < 0)
	{
		coe_a.g += PI2;
	}

	OrbMech::KeplerianToCartesian(coe_a, mu, Rtemp, Vtemp);
	return Vtemp - sv0.V;
}

bool TLAT(VECTOR3 R, VECTOR3 V, double lat, int C, double & K_AD, double &dtheta)
{
	VECTOR3 H, N;
	double i, U, Ulat;
	
	H = unit(crossp(R, V));
	N = unit(crossp(_V(0, 0, 1), H));
	i = acos(H.z);
	if (abs(lat) > i) return true;
	U = OrbMech::PHSANG(R, V, N);
	if (U < 0)
	{
		U += PI2;
	}
	Ulat = asin(abs(sin(lat)) / sin(i));
	if (C == 0)
	{
		K_AD = -1.0;
	}
	if (lat < 0)
	{
		U = U - PI;
	}
	U = atan2(sin(U), cos(U));
	if (U < 0)
	{
		U += PI2;
	}
	if (K_AD > 0)
	{
		Ulat = PI - Ulat;
	}
	if (C == 0)
	{
		double Ulat_pi = PI - Ulat;

		if (U >= Ulat)
		{
			if (U < Ulat_pi)
			{
				Ulat = Ulat_pi;
				K_AD = 1.0;
			}
			else
			{
				Ulat = Ulat + PI2;
			}
		}
	}
	dtheta = Ulat - U;
	if (C != 0)
	{
		if (abs(dtheta) > PI)
		{
			dtheta = dtheta - PI2 * sign(dtheta);
		}
	}
	return false;
}

double TLON(VECTOR3 R, VECTOR3 V, double t, double lng, int C)
{
	VECTOR3 N, H;
	double alpha1, alpha2, I, NODE, U, pro, U_lng, dtheta;

	H = unit(crossp(R, V));
	N = unit(crossp(_V(0, 0, 1), H));
	I = acos(H.z);
	NODE = atan2(N.y, N.x);
	U = OrbMech::PHSANG(R, V, N);
	if (U < 0)
	{
		U += PI2;
	}
	pro = 1.0;
	if (PI05 - I < 0)
	{
		pro = -1.0;
	}

	alpha1 = pro*(lng - NODE);
	alpha2 = alpha1 + OrbMech::w_Earth*t;
	U_lng = atan2(sin(alpha2), pro*cos(alpha2)*cos(I));
	if (U_lng < 0)
	{
		U_lng += PI2;
	}
	if (C == 0)
	{
		if (U_lng < U)
		{
			U_lng = U_lng + PI2;
		}
		dtheta = U_lng - U;
	}
	else
	{
		dtheta = U_lng - U;
		if (abs(dtheta) >= PI)
		{
			dtheta = dtheta - sign(dtheta)*PI2;
		}
	}
	return dtheta;
}

bool TALT(VECTOR3 R, VECTOR3 V, double rad, int C, double mu, double &dtheta)
{
	double r, v, r_dot, P, a, ecosE, esinE, e, r_MIN, r_MAX, theta_D, theta;

	r = length(R);
	v = length(V);
	r_dot = dotp(V, R) / r;
	P = pow(length(crossp(R, V)), 2) / mu;
	a = r / (2.0 - v * v*r / mu);
	ecosE = (a - r) / a;
	esinE = r * r_dot / sqrt(mu*a);
	e = sqrt(esinE*esinE + ecosE * ecosE);
	r_MIN = P / (1.0 + e);
	r_MAX = 2.0*a - r_MIN;

	//Error condition
	if (rad < r_MIN || r > r_MAX) return true;

	theta_D = acos((P / rad - 1.0) / e);
	theta = acos((P / r - 1.0) / e);
	
	if (C != 0)
	{
		dtheta = theta_D - theta;
		if (r_dot < 0)
		{
			dtheta = -dtheta;
		}
	}
	else
	{
		dtheta = 0.0;

		if (r_dot > 0)
		{
			if (theta_D > theta)
			{
				dtheta = theta_D - theta;
			}
			else if (theta_D < theta)
			{
				dtheta = PI2 - (theta_D + theta);
			}
		}
		else
		{
			if (theta_D > theta)
			{
				dtheta = theta + theta_D;
			}
			else if (theta_D < theta)
			{
				dtheta = theta - theta_D;
			}
		}
	}
	return false;
}

bool ShuttleFDOCore::SEARMT(SV sv0, int opt, double val, SV &sv1)
{
	double K_AD, dtheta, dt;
	int C;
	bool CEND;

	const int CMAX = 20;
	const double eps4 = 0.011*RAD;

	C = 0;
	CEND = false;
	sv1 = sv0;

	do
	{
		if (opt == OMPDefs::SECONDARIES::LAT || opt == OMPDefs::SECONDARIES::DEC)
		{
			if (TLAT(sv1.R, sv1.V, val, C, K_AD, dtheta))
			{
				//Error
				return true;
			 }
		}
		else if (opt == OMPDefs::SECONDARIES::ALT)
		{
			if (TALT(sv1.R, sv1.V, val + R_E, C, mu, dtheta))
			{
				//Error
				return true;
			}
		}
		else
		{
			dtheta = TLON(sv1.R, sv1.V, sv1.GMT, val, C);
		}

		dt = OrbMech::time_theta(sv1.R, sv1.V, dtheta, mu, C == 0);
		sv1 = coast_auto(sv1, dt);
		//If we don't use nonspherical gravity, we might be already done
		if (useNonSphericalGravity == false)
		{
			if (opt == OMPDefs::SECONDARIES::LAT || opt == OMPDefs::SECONDARIES::DEC || opt == OMPDefs::SECONDARIES::ALT)
			{
				break;
			}
		}
		C++;
	} while (abs(dtheta) > eps4 && C < CMAX);

	if (C >= CMAX)
	{
		return true;
	}
	return false;
}

VECTOR3 ShuttleFDOCore::TEG2M50(VECTOR3 v_TEG)
{
	return rhmul(M_TEGTOECL, v_TEG);
}