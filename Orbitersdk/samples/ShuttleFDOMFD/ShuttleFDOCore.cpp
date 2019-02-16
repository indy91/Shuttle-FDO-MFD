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
#include "OrbMech.h"
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

	//STS-126
	launchdate[0] = 2008;
	launchdate[1] = 320;
	launchdate[2] = 0;
	launchdate[3] = 55;
	launchdateSec = 18.0;
	SetLaunchMJD(launchdate[0], launchdate[1], launchdate[2], launchdate[3], launchdateSec);
	
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
	useNonSphericalGravity = false;
	OMPErrorCode = 0;

	subThreadMode = 0;
	subThreadStatus = 0;

	/*for (int i = 0;i < 4;i++)
	{
		launchdate[i] = 0;
	}
	launchdateSec = 0.0;*/

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

void ShuttleFDOCore::LoadPlanC()
{
	ManeuverConstraintsTable.clear();

	//Example plan
	AddManeuver(OMPDefs::MANTYPE::HA, "OMS-2");
	AddManeuverThreshold(0, OMPDefs::THRESHOLD::THRES_T, 30.0*60.0);
	AddManeuverSecondary(0, "APO", 1.0);
	AddManeuverSecondary(0, "HD", 85.0);

	AddManeuver(OMPDefs::MANTYPE::NC, "NC-1");
	AddManeuverThreshold(1, OMPDefs::THRESHOLD::THRES_M, 2.0);
	AddManeuverSecondary(1, "DV", 100.0);

	AddManeuver(OMPDefs::MANTYPE::EXDV, "NC-2");
	AddManeuverThreshold(2, OMPDefs::THRESHOLD::THRES_M, 9.0);
	AddManeuverSecondary(2, "DVLV", 8.0);
	AddManeuverSecondary(2, "DVLV", 0.0);
	AddManeuverSecondary(2, "DVLV", 0.0);

	AddManeuver(OMPDefs::MANTYPE::NPC, "NPC");
	AddManeuverThreshold(3, OMPDefs::THRESHOLD::THRES_DT, 1.0);
	AddManeuverSecondary(3, "CN", 1.0);

	AddManeuver(OMPDefs::MANTYPE::NH, "NC-3");
	AddManeuverThreshold(4, OMPDefs::THRESHOLD::THRES_M, 15.5);

	AddManeuver(OMPDefs::MANTYPE::NC, "NC-4");
	AddManeuverThreshold(5, OMPDefs::THRESHOLD::THRES_M, 0.5);
	AddManeuverSecondary(5, "DR", -40.0);

	AddManeuver(OMPDefs::MANTYPE::SOI, "TI");
	AddManeuverThreshold(6, OMPDefs::THRESHOLD::THRES_M, 1.0);
	AddManeuverSecondary(6, "DR", -8.0);
	AddManeuverSecondary(6, "DH", 0.2);
	AddManeuverSecondary(6, "WEDG", 0.0);

	AddManeuver(OMPDefs::MANTYPE::SOR, "MC-4");
	AddManeuverThreshold(7, OMPDefs::THRESHOLD::THRES_DT, 3600.0 + 16.0*60.0 + 54.0);
	AddManeuverSecondary(7, "CXYZ", -0.1481);
	AddManeuverSecondary(7, "CXYZ", 0.0);
	AddManeuverSecondary(7, "CXYZ", 0.2962);
}

void ShuttleFDOCore::LoadPlanCNoNPC()
{
	ManeuverConstraintsTable.clear();

	//Example plan
	AddManeuver(OMPDefs::MANTYPE::HA, "OMS-2");
	AddManeuverThreshold(0, OMPDefs::THRESHOLD::THRES_T, 30.0*60.0);
	AddManeuverSecondary(0, "APO", 1.0);
	AddManeuverSecondary(0, "HD", 85.0);

	AddManeuver(OMPDefs::MANTYPE::NC, "NC-1");
	AddManeuverThreshold(1, OMPDefs::THRESHOLD::THRES_M, 2.0);
	AddManeuverSecondary(1, "DV", 100.0);

	AddManeuver(OMPDefs::MANTYPE::EXDV, "NC-2");
	AddManeuverThreshold(2, OMPDefs::THRESHOLD::THRES_M, 9.0);
	AddManeuverSecondary(2, "DVLV", 8.0);
	AddManeuverSecondary(2, "DVLV", 0.0);
	AddManeuverSecondary(2, "DVLV", 0.0);

	AddManeuver(OMPDefs::MANTYPE::NH, "NC-3");
	AddManeuverThreshold(3, OMPDefs::THRESHOLD::THRES_M, 15.5);

	AddManeuver(OMPDefs::MANTYPE::NC, "NC-4");
	AddManeuverThreshold(4, OMPDefs::THRESHOLD::THRES_M, 0.5);
	AddManeuverSecondary(4, "DR", -40.0);

	AddManeuver(OMPDefs::MANTYPE::SOI, "TI");
	AddManeuverThreshold(5, OMPDefs::THRESHOLD::THRES_M, 1.0);
	AddManeuverSecondary(5, "DR", -8.0);
	AddManeuverSecondary(5, "DH", 0.2);

	AddManeuver(OMPDefs::MANTYPE::SOR, "MC-4");
	AddManeuverThreshold(6, OMPDefs::THRESHOLD::THRES_DT, 3600.0 + 16.0*60.0 + 54.0);
	AddManeuverSecondary(6, "CXYZ", -0.1481);
	AddManeuverSecondary(6, "CXYZ", 0.0);
	AddManeuverSecondary(6, "CXYZ", 0.2962);
}

SV ShuttleFDOCore::StateVectorCalc(VESSEL *vessel, double SVMJD)
{
	VECTOR3 R, V;
	double dt;
	SV sv, sv1;

	vessel->GetRelativePos(hEarth, R);
	vessel->GetRelativeVel(hEarth, V);
	sv.MJD = oapiGetSimMJD();

	sv.R = _V(R.x, R.z, R.y);
	sv.V = _V(V.x, V.z, V.y);

	sv.mass = vessel->GetMass();

	if (SVMJD != 0.0)
	{
		dt = (SVMJD - sv.MJD)*24.0*3600.0;
		sv1 = coast(sv, dt);
	}
	else
	{
		sv1 = sv;
	}

	return sv1;
}

SV ShuttleFDOCore::coast(SV sv0, double dt)
{
	SV sv1;

	OrbMech::oneclickcoast(sv0.R, sv0.V, sv0.MJD, dt, sv1.R, sv1.V);
	sv1.mass = sv0.mass;
	sv1.MJD = sv0.MJD + dt / 24.0 / 3600.0;

	return sv1;
}

SV ShuttleFDOCore::coast_osc(SV sv0, double dt)
{
	SV sv1;

	OrbMech::rv_from_r0v0(sv0.R, sv0.V, dt, sv1.R, sv1.V, mu);
	sv1.mass = sv0.mass;
	sv1.MJD = sv0.MJD + dt / 24.0 / 3600.0;

	return sv1;
}

SV ShuttleFDOCore::coast_auto(SV sv0, double dt)
{
	if (useNonSphericalGravity)
	{
		return coast(sv0, dt);
	}
	else
	{
		return coast_osc(sv0, dt);
	}
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
		sv_p = GeneralTrajectoryPropagation(sv0, 1, 0.0);
		sv_a = GeneralTrajectoryPropagation(sv0, 1, PI);
	}
	else
	{
		SV sv1, sv2;
		double u_x, u_y;

		//First guess
		ApsidesArgumentofLatitudeDetermination(sv0, u_x, u_y);

		sv1 = GeneralTrajectoryPropagation(sv0, 2, u_x);
		sv2 = GeneralTrajectoryPropagation(sv0, 2, u_y);

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

SV ShuttleFDOCore::GeneralTrajectoryPropagation(SV sv0, int opt, double param, double DN)
{
	//Update to the given time
	if (opt == 0)
	{
		double MJD1, dt;

		MJD1 = param;
		dt = (MJD1 - sv0.MJD)*24.0*3600.0;
		return coast(sv0, dt);
	}
	//Update to the given mean anomaly
	else
	{
		SV sv1;
		OrbMech::CELEMENTS osc0, osc1;
		VECTOR3 R_equ, V_equ;
		double DX_L, X_L, X_L_dot, dt, ddt, L_D, ll_dot, n0, g_dot, J20;
		int LINE, COUNT;
		bool DH;

		J20 = 1082.6269e-6;

		sv1 = sv0;
		OrbMech::EclipticToECI(sv0.R, sv0.V, sv0.MJD, R_equ, V_equ);
		osc0 = OrbMech::CartesianToKeplerian(R_equ, V_equ, mu);

		n0 = sqrt(mu / (osc0.a*osc0.a*osc0.a));
		ll_dot = n0;
		g_dot = n0 * (-(3.0 / 4.0)*(J20*R_E*(5.0*cos(osc0.i)*cos(osc0.i) - 1.0)) / (osc0.a*osc0.a*pow(1.0 - osc0.e*osc0.e, 2.0)));

		osc1 = osc0;
		if (opt != 3)
		{
			L_D = param;
		}
		else
		{
			double u = OrbMech::MeanToTrueAnomaly(osc1.l, osc1.e) + osc1.g;
			u = fmod(u, PI2);
			if (u < 0)
				u += PI2;
			L_D = u;
		}
		DX_L = 1.0;
		DH = DN > 0.0;
		dt = 0.0;
		LINE = 0;
		COUNT = 24;

		do
		{
			//Mean anomaly
			if (opt == 1)
			{
				X_L = osc1.l;
				X_L_dot = ll_dot;
			}
			//Argument of latitude
			else if (opt == 2)
			{
				double u = OrbMech::MeanToTrueAnomaly(osc1.l, osc1.e) + osc1.g;
				u = fmod(u, PI2);
				if (u < 0)
					u += PI2;

				X_L = u;
				X_L_dot = ll_dot + g_dot;
			}
			//Maneuver line
			else
			{
				double u = OrbMech::MeanToTrueAnomaly(osc1.l, osc1.e) + osc1.g;
				u = fmod(u, PI2);
				if (u < 0)
					u += PI2;

				X_L = u;
				X_L_dot = ll_dot + g_dot;
				LINE = 2;
			}

			if (DH)
			{
				double DN_apo = DN * PI2;
				ddt = DN_apo / ll_dot;
				DH = false;

				if (LINE != 0)
				{
					L_D = L_D + g_dot * ddt + DN_apo;
					while (L_D < 0) L_D += PI2;
					while (L_D >= PI2) L_D -= PI2;
				}
				else
				{
					ddt += (L_D - X_L) / X_L_dot;
				}
			}
			else
			{
				DX_L = L_D - X_L;
				if (abs(DX_L) - PI >= 0)
				{
					if (DX_L > 0)
					{
						DX_L -= PI2;
					}
					else
					{
						DX_L += PI2;
					}
				}
				ddt = DX_L / X_L_dot;
				if (LINE != 0)
				{
					L_D = L_D + ddt * g_dot;
				}
			}


			dt += ddt;
			sv1 = coast(sv1, ddt);
			OrbMech::EclipticToECI(sv1.R, sv1.V, sv1.MJD, R_equ, V_equ);
			osc1 = OrbMech::CartesianToKeplerian(R_equ, V_equ, mu);

			COUNT--;

		} while (abs(DX_L) > 0.0005 && COUNT > 0);

		return sv1;
	}

	return sv0;
}

void ShuttleFDOCore::ApsidesArgumentofLatitudeDetermination(SV sv0, double &u_x, double &u_y)
{
	OrbMech::OELEMENTS coe;
	SV sv[3];
	MATRIX3 obl;
	VECTOR3 R1_equ, V1_equ;
	double u[3], r[3], gamma, u_0;

	sv[0] = sv0;
	sv[1] = coast(sv[0], 15.0*60.0);
	sv[2] = coast(sv[1], 15.0*60.0);

	for (int i = 0;i < 3;i++)
	{
		obl = OrbMech::GetObliquityMatrix(hEarth, sv[i].MJD);
		R1_equ = OrbMech::rhtmul(obl, sv[i].R);
		V1_equ = OrbMech::rhtmul(obl, sv[i].V);
		coe = OrbMech::coe_from_sv(R1_equ, V1_equ, mu);
		u[i] = fmod(coe.TA + coe.w, PI2);
		r[i] = length(R1_equ);
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

VECTOR3 ShuttleFDOCore::SOIManeuver(SV sv_A, SV sv_P, double MJD1, double dt, VECTOR3 off)
{
	SV sv_A1, sv_P2;
	VECTOR3 RP2_off, VA1_apo, DV;
	double dt1, dt2;

	dt1 = (MJD1 - sv_A.MJD)*24.0*3600.0;
	dt2 = (MJD1 - sv_P.MJD)*24.0*3600.0 + dt;

	sv_A1 = coast_auto(sv_A, dt1);
	sv_P2 = coast_auto(sv_P, dt2);

	OrbMech::REL_COMP(sv_P2.R, sv_P2.V, RP2_off, off);
	VA1_apo = LambertAuto(sv_A1.R, sv_A1.V, sv_A1.MJD, RP2_off, dt, 0, true);
	DV = VA1_apo - sv_A1.V;
	return DV;
}

VECTOR3 ShuttleFDOCore::SORManeuver(SV sv_A, SV sv_P, double MJD1, VECTOR3 off)
{
	SV sv_A1, sv_P1, sv_P2;
	VECTOR3 VA1_apo, DV, RP2_off;
	double dt, dt1, dt2;

	dt1 = (MJD1 - sv_A.MJD)*24.0*3600.0;
	dt2 = (MJD1 - sv_P.MJD)*24.0*3600.0;

	sv_A1 = coast_auto(sv_A, dt1);
	sv_P1 = coast_auto(sv_P, dt2);

	dt = OrbMech::time_theta(sv_P1.R, sv_P1.V, 270.0*RAD, mu);
	sv_P2 = coast_auto(sv_P1, dt);

	OrbMech::REL_COMP(sv_P2.R, sv_P2.V, RP2_off, off);
	VA1_apo = LambertAuto(sv_A1.R, sv_A1.V, sv_A1.MJD, RP2_off, dt, 0, true);
	DV = VA1_apo - sv_A1.V;
	return DV;
}

VECTOR3 ShuttleFDOCore::LambertAuto(VECTOR3 RA, VECTOR3 VA, double MJD0, VECTOR3 RP_off, double dt, int N, bool prog)
{
	if (useNonSphericalGravity)
	{
		return OrbMech::Vinti(RA, VA, RP_off, MJD0, dt, N, prog, _V(0, 0, 0));
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
	if (ManeuverConstraintsTable.size() < 1) return 1;	//Error 1: No maneuvers in constraint table
	if (ManeuverConstraintsTable[0].threshold != OMPDefs::THRESHOLD::THRES_T) return 2;	//Error 2: First maneuver needs a T as threshold

	SV *sv_bef_table, *sv_aft_table, *sv_P_table;
	VECTOR3 DV;
	VECTOR3 *dv_table;
	VECTOR3 *add_constraint;
	TIGSecondaries *tigmodifiers;
	double *thresholdtime;
	double dt;
	unsigned i, j, k, l, TAB;
	std::vector<ITERCONSTR> iterators;

	sv_bef_table = sv_aft_table = sv_P_table = NULL;
	tigmodifiers = NULL;

	TAB = ManeuverConstraintsTable.size();
	l = 0;

	sv_bef_table = new SV[TAB];
	sv_aft_table = new SV[TAB];
	sv_P_table = new SV[TAB];
	tigmodifiers = new TIGSecondaries[TAB];
	dv_table = new VECTOR3[TAB];
	add_constraint = new VECTOR3[TAB];
	thresholdtime = new double[TAB];

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
		if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::NOTHR) return 5;	//Error 5: Maneuver doesn't have a threshold
		
		//Look through all secondary constraints
		for (j = 0;j < ManeuverConstraintsTable[i].secondaries.size();j++)
		{
			found = 0;

			//DR constraint found
			if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DR") == 0)
			{
				for (k = i - 1;k >= 0;k--)
				{
					if (ManeuverConstraintsTable[k].type == OMPDefs::MANTYPE::NC)
					{
						found = 1;
					}
					if (found) break;
				}

				if (!found) return 7;	//Error 7: didn't find NC constraint

				con.man = k;
				con.type = 1;
				con.constr = i;
				con.value = ManeuverConstraintsTable[i].secondaries[j].value*1852.0;

				iterators.push_back(con);
			}
			//DH constraint found
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "DH") == 0)
			{
				for (k = i - 1;k >= 0;k--)
				{
					if (ManeuverConstraintsTable[k].type == OMPDefs::MANTYPE::NH)
					{
						found = 1;
					}
					if (found) break;
				}

				if (!found) return 8;	//Error 8: didn't find NH constraint

				con.man = k;
				con.type = 2;
				con.constr = i;
				con.value = ManeuverConstraintsTable[i].secondaries[j].value*1852.0;

				iterators.push_back(con);
			}
			//WEDG constraint found
			else if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "WEDG") == 0)
			{
				for (k = i - 1;k >= 0;k--)
				{
					if (ManeuverConstraintsTable[k].type == OMPDefs::MANTYPE::NPC)
					{
						//if (npcflag) return 22;	//Error 22: More than one NPC maneuver specified
						//Allow only one NPC maneuver
						//npcflag = true;
						found = 1;
					}
					if (found) break;
				}

				if (!found) return 8;	//Error 8: didn't find NH constraint

				con.man = k;
				con.type = 3;
				con.constr = i;
				con.value = ManeuverConstraintsTable[i].secondaries[j].value*RAD;

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
		if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HA)
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
			if (strcmp(ManeuverConstraintsTable[i].secondaries[j].type, "APS") == 0)
			{
				tigmodifiers[i].type = OMPDefs::SECONDARIES::SEC_APS;
				tigmodifiers[i].value = ManeuverConstraintsTable[i].secondaries[j].value;
			}
		}
	}

	SV sv_A0, sv_P0, sv_cur, sv_P_cur;
	sv_A0 = StateVectorCalc(vessel);
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
			dt = ManeuverConstraintsTable[i].thresh_num - OrbMech::GETfromMJD(sv_cur.MJD, LaunchMJD);
			sv_bef_table[i] = coast_auto(sv_cur, dt);
		}
		else if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_M)
		{
			sv_bef_table[i] = DeltaOrbitsAuto(sv_cur, ManeuverConstraintsTable[i].thresh_num);
		}
		else if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::THRES_DT)
		{
			sv_bef_table[i] = coast_auto(sv_cur, ManeuverConstraintsTable[i].thresh_num);
		}

		thresholdtime[i] = sv_bef_table[i].MJD;

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
				//First iteration
				if (add_constraint[i].x == 0.0)
				{
					SV sv_P1 = coast_auto(sv_P_cur, (sv_bef_table[i].MJD - sv_P_cur.MJD)*24.0*3600.0);
					VECTOR3 H_P = crossp(sv_P1.R, sv_P1.V);
					dt = FindCommonNode(sv_bef_table[i], H_P);
				}
				else
				{
					dt = FindCommonNode(sv_bef_table[i], add_constraint[i]);
				}


				sv_bef_table[i] = coast_auto(sv_bef_table[i], dt);
			}
			else if (tigmodifiers[i].type == OMPDefs::SECONDARIES::SEC_APS)
			{
				sv_bef_table[i] = FindNthApsidalCrossingAuto(sv_bef_table[i], tigmodifiers[i].value);
			}
		}

		//TIG has been calculated, now get target SV at TIG
		sv_P_table[i] = coast_auto(sv_P_cur, (sv_bef_table[i].MJD - sv_P_cur.MJD)*24.0*3600.0);

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

					if (abs(iterstate[l].err) > 5.0)
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

					if (abs(iterstate[l].err) > 5.0)
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

					if (abs(iterstate[l].err) > 0.005*RAD)
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
							sv_PH = coast_auto(sv_PH, (sv_bef_table[m].MJD - sv_PH.MJD)*24.0*3600.0);
							if (m > iterators[l].man)
							{
								DV = tmul(OrbMech::LVLH_Matrix(sv_PH.R, sv_PH.V), dv_table[m]);
								sv_PH.V -= DV;
							}
						}

						add_constraint[iterators[l].man] = crossp(sv_PH.R, sv_PH.V);

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
					}
				}
			}
		}

		if (recycle) continue;

		//MANEUVER
		if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HA)
		{
			VECTOR3 Rtemp, Vtemp;
			double dt_temp, dh;

			OrbMech::REVUP(sv_bef_table[i].R, sv_bef_table[i].V, 0.5, mu, Rtemp, Vtemp, dt_temp);
			dh = oapiGetSize(hEarth) + add_constraint[i].x - length(Rtemp);
			DV = OrbMech::HeightManeuver(sv_bef_table[i].R, sv_bef_table[i].V, dh, mu);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::EXDV || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NC || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NH)
		{

			DV = tmul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), dv_table[i]);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOI)
		{
			double ddt;
			if (ManeuverConstraintsTable[i + 1].threshold == OMPDefs::THRESHOLD::THRES_T)
			{
				ddt = ManeuverConstraintsTable[i + 1].thresh_num - OrbMech::GETfromMJD(sv_bef_table[i].MJD, LaunchMJD);
			}
			else if (ManeuverConstraintsTable[i + 1].threshold == OMPDefs::THRESHOLD::THRES_DT)
			{
				ddt = ManeuverConstraintsTable[i + 1].thresh_num;
			}
			else
			{
				return 23;	//No valid threshold for SOI/NCC
			}

			DV = SOIManeuver(sv_bef_table[i], sv_P_cur, sv_bef_table[i].MJD, ddt, add_constraint[i + 1]);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOR)
		{
			DV = SORManeuver(sv_bef_table[i], sv_P_cur, sv_bef_table[i].MJD, add_constraint[i]);
			dv_table[i] = mul(OrbMech::LVLH_Matrix(sv_bef_table[i].R, sv_bef_table[i].V), DV);
		}
		else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NPC)
		{
			VECTOR3 H_P;
			//First iteration
			if (add_constraint[i].x == 0.0)
			{
				SV sv_P1 = coast_auto(sv_P_cur, (sv_bef_table[i].MJD - sv_P_cur.MJD)*24.0*3600.0);
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
				ddt = ManeuverConstraintsTable[i + 1].thresh_num - OrbMech::GETfromMJD(sv_bef_table[i].MJD, LaunchMJD);
			}
			else if (ManeuverConstraintsTable[i + 1].threshold == OMPDefs::THRESHOLD::THRES_DT)
			{
				ddt = ManeuverConstraintsTable[i + 1].thresh_num;
			}
			else
			{
				return 23;	//No valid threshold for SOI/NCC
			}

			DV = SOIManeuver(sv_bef_table[i], sv_P_cur, sv_bef_table[i].MJD, ddt, add_constraint[i + 1]);
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
			DV = CircManeuver(sv_bef_table[i]);
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
		man.TIG_MJD = sv_bef_table[i].MJD;
		man.type = ManeuverConstraintsTable[i].type;

		ManeuverTable.push_back(man);
	}

	CalculateManeuverEvalTable(sv_A0, sv_P0);

	delete sv_bef_table; delete sv_aft_table; delete tigmodifiers; delete dv_table;

	return 0;
}

void ShuttleFDOCore::CalculateManeuverEvalTable(SV sv_A0, SV sv_P0)
{
	SV sv_cur, sv_Pcur;
	VECTOR3 DV, u, R, Rtemp, Vtemp, R_REL, V_REL;
	double apo, peri, dt1, dt2;
	OBJHANDLE hSun;
	MANEVALDATA man;
	ManeuverEvaluationTable.clear();

	sv_cur = sv_A0;
	hSun = oapiGetObjectByName("hSun");

	for (unsigned i = 0;i < ManeuverTable.size();i++)
	{
		sv_cur = coast_auto(sv_cur, (ManeuverTable[i].TIG_MJD - sv_cur.MJD)*24.0*3600.0);
		DV = tmul(OrbMech::LVLH_Matrix(sv_cur.R, sv_cur.V), ManeuverTable[i].dV_LVLH);
		sv_cur.V += DV;

		GetOPMManeuverType(man.type, ManeuverTable[i].type);
		sprintf_s(man.name, 10, ManeuverTable[i].name);
		man.DVMag = length(ManeuverTable[i].dV_LVLH) / 0.3048;
		man.GMTIG = OrbMech::MJDToDate(ManeuverTable[i].TIG_MJD);
		man.METIG = OrbMech::GETfromMJD(ManeuverTable[i].TIG_MJD, LaunchMJD);
		if (i < ManeuverTable.size() - 1)
		{
			man.DT = (ManeuverTable[i + 1].TIG_MJD - ManeuverTable[i].TIG_MJD)*24.0*3600.0;
		}
		else
		{
			man.DT = 0.0;
		}
		man.DV = ManeuverTable[i].dV_LVLH / 0.3048;
		OrbMech::periapo(sv_cur.R, sv_cur.V, mu, apo, peri);
		man.HA = (apo - R_E) / 1852.0;
		man.HP = (peri - R_E) / 1852.0;

		sv_Pcur = coast_auto(sv_P0, (sv_cur.MJD - sv_P0.MJD)*24.0*3600.0);
		u = unit(crossp(sv_Pcur.R, sv_Pcur.V));
		R = unit(sv_cur.R - u * dotp(sv_cur.R, u))*length(sv_cur.R);
		OrbMech::RADUP(sv_Pcur.R, sv_Pcur.V, R, mu, Rtemp, Vtemp);
		man.DH = (length(Rtemp) - length(R)) / 1852.0;
		man.RANGE = length(sv_Pcur.R - sv_cur.R) / 1852.0;
		OrbMech::REL_COMP(true, sv_Pcur.R, sv_Pcur.V, sv_cur.R, sv_cur.V, R_REL, V_REL);
		man.PHASE = -R_REL.x / length(sv_Pcur.R)*DEG;
		man.Y = R_REL.y / 0.3048;
		man.Ydot = V_REL.y / 0.3048;

		dt1 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.MJD, hEarth, hSun, true, true, true);
		dt2 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.MJD, hEarth, hSun, false, true, true);

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

		dt1 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.MJD, hEarth, hSun, true, false, true);
		dt2 = OrbMech::sunrise(sv_cur.R, sv_cur.V, sv_cur.MJD, hEarth, hSun, false, false, true);

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

void ShuttleFDOCore::GetOPMManeuverType(char *buf, OMPDefs::MANTYPE type)
{
	if (type == OMPDefs::MANTYPE::HA)
	{
		sprintf_s(buf, 100, "HA");
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

VECTOR3 ShuttleFDOCore::CircManeuver(SV sv_A)
{
	//TBD: nonspherical logic
	VECTOR3 U_H, U_hor, V_apo;
	double v_circ;

	U_H = unit(crossp(sv_A.R, sv_A.V));
	U_hor = unit(crossp(U_H, unit(sv_A.R)));
	v_circ = sqrt(mu / length(sv_A.R));
	V_apo = U_hor * v_circ;
	return V_apo - sv_A.V;
}

double ShuttleFDOCore::FindCommonNode(SV sv_A, VECTOR3 H_P)
{
	SV sv_A1;
	VECTOR3 u1, u2, u_node[2];
	double theta, dt[2], ddt, dt_abs;
	int i = 0;
	dt_abs = 0.0;

	sv_A1 = sv_A;

	do
	{
		u1 = unit(crossp(sv_A1.V, sv_A1.R));
		u2 = -unit(H_P);

		u_node[0] = unit(crossp(u1, u2));
		u_node[1] = -u_node[0];

		for (int j = 0;j < 2;j++)
		{
			theta = OrbMech::sign(dotp(crossp(sv_A1.R, u_node[j]), crossp(sv_A1.R, sv_A1.V)))*OrbMech::acos2(dotp(sv_A1.R / length(sv_A1.R), u_node[j]));
			dt[j] = OrbMech::time_theta(sv_A1.R, sv_A1.V, theta, mu, i == 0);
		}

		if (abs(dt[0]) > abs(dt[1]))
		{
			ddt = dt[1];
		}
		else
		{
			ddt = dt[0];
		}

		sv_A1 = coast_auto(sv_A1, ddt);
		dt_abs += ddt;
		i++;

		//If wedge angle is below 0.01° or 10 iterations have been done, go back
		if (acos(dotp(u1, u2)) < 0.01*RAD || i >= 10) break;

	} while (abs(ddt) > 0.1);

	return dt_abs;
}

double ShuttleFDOCore::CalculateInPlaneTime()
{
	SV sv_P;
	VECTOR3 u_A, u_P, R_equ, R_ecl, V_A, n_A, n_P;
	double lng, lat, rad, dt, MJD, dt_bias, LAN_A, LAN_P, dLAN;

	dt = 0.0;
	dt_bias = -300.0;

	vessel->GetEquPos(lng, lat, rad);
	sv_P = StateVectorCalc(target);
	MJD = sv_P.MJD;

	R_equ = unit(_V(cos(lng)*cos(lat), sin(lng)*cos(lat), sin(lat)));

	do
	{
		R_ecl = OrbMech::rhmul(OrbMech::GetRotationMatrix(hEarth, MJD), R_equ);
		u_P = unit(crossp(sv_P.R, sv_P.V));
		V_A = crossp(u_P, R_ecl);
		u_A = unit(crossp(R_ecl, V_A));

		n_A = _V(-u_A.y, u_A.x, 0.0);
		n_P = _V(-u_P.y, u_P.x, 0.0);
		LAN_A = OrbMech::acos2(n_A.x / length(n_A));
		if (n_A.y < 0)
		{
			LAN_A = PI2 - LAN_A;
		}
		LAN_P = OrbMech::acos2(n_P.x / length(n_P));
		if (n_P.y < 0)
		{
			LAN_P = PI2 - LAN_P;
		}
		dLAN = LAN_P - LAN_A;
		dt = dLAN / w_E;

		MJD += dt / 24.0 / 3600.0;
	} while (abs(dt) > 0.1);

	return MJD + dt_bias / 24.0 / 3600.0;
}

void ShuttleFDOCore::CalcLaunchTime()
{
	double intpart;
	double MJD = CalculateInPlaneTime();

	InPlaneGMT = modf(MJD, &intpart)*24.0*3600.0;
}

int ShuttleFDOCore::subThread()
{
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

void ShuttleFDOCore::MET2MTT()
{
	if (ManeuverEvaluationTable.size() < 1) return;

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
	if (sv_chaser.MJD == 0.0) return;

	DMTInputTable.clear();

	DMTINPUT man;
	SV sv_cur, sv_tig;
	VECTOR3 DV_iner;
	double MJD, dt, F, isp, dv, dt_burn;

	sv_cur = sv_chaser;

	for (unsigned i = 0;i < ManeuverTransferTable.size();i++)
	{
		MJD = OrbMech::MJDfromGET(ManeuverEvaluationTable[i].METIG, LaunchMJD);
		dt = (MJD - sv_cur.MJD)*24.0*3600.0;
		sv_cur = coast_auto(sv_cur, dt);

		GetThrusterData(ManeuverTransferTable[i].thrusters, F, isp);

		dv = ManeuverEvaluationTable[i].DVMag*0.3048;
		dt_burn = isp / F * sv_cur.mass*(1.0 - exp(-dv / isp));

		DV_iner = tmul(OrbMech::LVLH_Matrix(sv_cur.R, sv_cur.V), ManeuverEvaluationTable[i].DV*0.3048);
		man.DV_iner = DV_iner;

		if (ManeuverTransferTable[i].IMP)
		{
			sv_tig = coast_auto(sv_cur, -dt_burn / 2.0);
		}
		else
		{
			sv_tig = sv_cur;
		}

		man.sv_tig = sv_tig;

		//TBD: ITER support

		sv_tig.mass -= F / isp * dt_burn;
		sv_tig.V += DV_iner;
		sv_cur = sv_tig;

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
	MATRIX3 Rot;
	VECTOR3 u_A;
	double F, isp, P, LY, RY, p_T, y_T;
	char Buffer[100], Buffer2[100];

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
	DMT.TIG = OrbMech::GETfromMJD(input.sv_tig.MJD, LaunchMJD);
	
	DMT.PEG4_C1 = 0.0;
	DMT.PEG4_C2 = 0.0;
	DMT.PEG4_HT = 0.0;
	DMT.PEG4_PRPLT = 0.0;
	DMT.PEG4_THETAT = 0.0;

	Rot = OrbMech::LVLH_Matrix(input.sv_tig.R, input.sv_tig.V);
	DMT.PEG7_DV = mul(Rot, input.DV_iner)*MPS2FPS;

	u_A = _V(cos(y_T)*cos(p_T), sin(y_T), -cos(y_T)*sin(p_T));

	MATRIX3 MTP;
	VECTOR3 u_D, VEC_BOD, RR_BOD, VEC_M50, RR_M50, YN, YT, vec_A, vec_B, RORB, VORB, Att;
	double ROLL;

	u_D = unit(input.DV_iner);
	VEC_BOD = u_A;
	if (abs(sin(y_T)) <= 0.999848)
	{
		RR_BOD = _V(0, 1, 0);
	}
	else
	{
		RR_BOD = _V(0, 0, -1);
	}
	RORB = OrbMech::Ecl2M50(hEarth, input.sv_tig.R);
	VORB = OrbMech::Ecl2M50(hEarth, input.sv_tig.V);
	VEC_M50 = OrbMech::Ecl2M50(hEarth, u_D);
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
	DMT.DVTOT = length(input.DV_iner)*MPS2FPS;

	GetThrusterData(input.thrusters, F, isp);
	DMT.TGO = isp / F * input.sv_tig.mass*(1.0 - exp(-length(input.DV_iner) / isp));
	DMT.VGO = u_A*DMT.DVTOT;

	double apo, peri;
	OrbMech::periapo(input.sv_tig.R, input.sv_tig.V + input.DV_iner, mu, apo, peri);


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

void ShuttleFDOCore::SetLaunchMJD(int Y, int D, int H, int M, double S)
{
	launchdate[0] = Y;
	launchdate[1] = D;
	launchdate[2] = H;
	launchdate[3] = M;
	launchdateSec = S;

	LaunchMJD = OrbMech::Date2MJD(Y, D, H, M, S);
	//sprintf(oapiDebugString(), "%f", LaunchMJD);
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
			sv_out = GeneralTrajectoryPropagation(sv_A, 1, PI, revs - 1.0);
		}
		else
		{
			sv_out = GeneralTrajectoryPropagation(sv_A, 1, PI, revs);
		}

		return sv_out;

		/*SV sv_out2;
		double dt1 = 0.0;
		if (revs > 1.0)
		{
			double T_P = OrbMech::period(sv_A.R, sv_A.V, mu);
			dt1 = T_P * (revs - 1.0);
			sv_out2 = coast_auto(sv_A, dt1);
		}
		else
		{
			sv_out2 = sv_A;
		}
		sv_out = sv_out2;
		double dt2 = OrbMech::timetoapo_integ(sv_out2.R, sv_out2.V, sv_out2.MJD, sv_out.R, sv_out.V);
		sv_out.MJD += dt2 / 24.0 / 3600.0;*/
		
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
	OrbMech::AEGServiceRoutine(sv0.R, sv0.V, sv0.MJD, opt, dval, DN, sv1.R, sv1.V, sv1.MJD);
	return sv1;
}

SV ShuttleFDOCore::DeltaOrbitsAuto(SV sv0, double M)
{
	if (useNonSphericalGravity)
	{
		return GeneralTrajectoryPropagation(sv0, 3, 0.0, M);
	}
	else
	{
		double P = OrbMech::period(sv0.R, sv0.V, mu);
		double dt = P * M;
		return coast_osc(sv0, dt);
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
			return GeneralTrajectoryPropagation(sv0, 1, PI, DN);
		}
		else
		{
			//Periapsis
			return GeneralTrajectoryPropagation(sv0, 1, 0, DN);
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
		return coast_osc(sv0, dt + 0.5*P * (N - 1.0));
	}
}