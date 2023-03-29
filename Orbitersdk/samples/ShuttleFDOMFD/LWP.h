/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Launch Window Processor

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

#include "OrbMech.h"

struct LWPOutput
{
	double GMTOPT = 0.0;
	double PA_GMTOPT = 0.0;
	double PFT = 0.0;
	double PFA = 0.0;
	double DTO = 0.0;
	double DTC = 0.0;
	double OPT = 0.0;
	int PHASE = 0;
	int WRAP = 0;
	//Planar open
	double GMTPO = 0.0;
	double PA_GMTPO = 0.0;
	//Planar close
	double GMTPC = 0.0;
	double PA_GMTPC = 0.0;
	//Phase open
	double PA_OPEN = 0.0;
	double GMTLO_OPEN = 0.0;
	//Phase close
	double PA_CLOSE = 0.0;
	double GMTLO_CLOSE = 0.0;
	int LWPERROR = 0;
};

struct LTPOutput
{
	double GMTLO = 0.0;
	//MECO
	double MET_MECO = 0.0;
	double V_MECO = 0.0;
	double R_MECO = 0.0;
	double G_MECO = 0.0;
	double I_MECO = 0.0;
	double PHASE_MECO = 0.0;
	double HA_MECO = 0.0;
	double HP_MECO = 0.0;
	double LONG_MECO = 0.0;
	//MPS Dump
	double TIG_MPS = 0.0;
	double DV_MPS = 0.0;
	double HA_MPS = 0.0;
	double HP_MPS = 0.0;
	//OMS-2
	double TIG_OMS2 = 0.0;
	double DV_OMS2 = 0.0;
	double HA_OMS2 = 0.0;
	double HP_OMS2 = 0.0;
	double NODE_OMS2 = 0.0;
	double PHASE_OMS2 = 0.0;
	double PERIOD_OMS2 = 0.0;
	//TGT
	double HA_TGT = 0.0;
	double HP_TGT = 0.0;
	double LONG_TGT = 0.0;
	double DELN = 0.0;
	double PERIOD_TGT = 0.0;

	//IYD vectors
	VECTOR3 IY_MECO = _V(0, 0, 1);
	VECTOR3 IY_OMS1 = _V(0, 0, 1);
	VECTOR3 IY_OMS2 = _V(0, 0, 1);

	int LWPERROR = 0;
};

struct LWPStateVectorTable
{
	//Target state vectors
	OrbMech::SV sv_T0, sv_T_MECO;
	//Chaser state vectors
	OrbMech::SV sv_P_MECO, sv_P_ET_Sep, sv_P_MPS_Dump, sv_P_OMS2_before, sv_P_OMS2_after;
};

struct OMSTargetSet
{
	double DTIG;
	double C1;
	double C2;
	double HTGT;
	double THETA;
};

struct LWPSettings
{
	LWPSettings();

	//Inplane launch window opening and closing times option
	//0 = inplane opening (ascending node), 1 = inplane closing (descending node), 2 = opening and closing (both)
	int NS;
	//0 = conic, 1 = integrated
	int SVPROP;
	//Delta time to be subtracted from analytical inplane launch time to obtain empirical inplane launch time
	double DTOPT;
	//Flag to wrap initial phase angle (add 2NPI to phase angle)
	int WRAP;
	//Initial phase angle control flag
	//0 = 0 to 2PI, 1 = -2PI to 0, 2 = -PI to PI
	int NEGTIV;
	//Flightpath angle at insertion
	double GAMINS;
	//Geocentric latitude of launch site
	double LATLS;
	//Geocentric longitude of launch site
	double LONGLS;
	//Powered flight time
	double PFT;
	//Powered flight arc
	double PFA;
	//Radius of insertion
	double RINS;
	//Velocity magnitude of insertion
	double VINS;
	//Yaw steering limit
	double YSMAX;
	//GMT of inplane lift-off time (normally computed internally)
	double TPLANE;
	//Launch window/launch targeting options
	//0 = LW (should not be used), 1 = LT, 2 = LW and LT
	int LW;
	//Delta time prior to in-plane time to start parameter table
	double TSTART;
	//Delta time after in-plane time to stop parameter table
	double TEND;
	//Target state vector
	OrbMech::SV TRGVEC;
	//Flag for option to compute differential nodal regression from insertion to rendezvous
	//false = input DELNO, true = compute DELNO
	bool DELNOF;
	//Angle that is added to the target descending node to account for differential nodal regression
	double DELNO;
	//Lift-off time options for launch targeting
	//1 = input time, 2 = phase angle offset, 3 = biased phase zero (GMTLOR threshold), 4 = biased phase zero (TPLANE threshold), 5 = in-plane, 6 = in-plane with nodal regression
	int LOT;
	//Chaser vehicle weight at OMS-2
	double CWHT;
	//Bias that is added to GMTLO* to produce lift-off time
	double BIAS = 0.0;
	//Launch azimuth coefficients
	double LAZCOE[4];
	//Phase angle desired at insertion
	double OFFSET;
	//DT from lift-off, which defines the time of guidance reference release
	double DTGRR;
	//Delta time added to inplane time to obtain lift-off time
	double TRANS;
	//Insertion cutoff conditions option flag
	//1 = Input VINS, GAMINS, RINS; 2 = Input GAMINS, RINS and height difference, 3 = Input GAMINS RINS, altitude at input angle from insertion
	int INSCO;
	//Recommended or threshold lift-off GMT
	double GMTLOR;
	//DTIG between MECO and ET SEP
	double DTIG_ET_SEP;
	//DTIG between MECO and MPS Dump
	double DTIG_MPS;
	//ET SEP DV
	VECTOR3 DV_ET_SEP;
	//MPS Dump DV
	VECTOR3 DV_MPS;
	//false = Standard Insertion, true = Direct Insertion
	bool DirectInsertion;
	OMSTargetSet OMS1, OMS2;
	LWPOutput *lwp_table;
	LTPOutput *ltp_table;
};

class LaunchWindowProcessor
{
public:
	LaunchWindowProcessor();
	void Init(LWPSettings &set);
	void LWP();

	LWPStateVectorTable LWPSV;
protected:
	void LWT();
	void UPDAT(VECTOR3 &R, VECTOR3 &V, double &T, double TF);
	void NPLAN(double &TIP);
	void LENSR(double GMTLO);
	double PHANG();
	void GMTLS(double TI, double TF);
	void RLOT();
	void TARGT();
	void NSERT(double GMTLO, double &UINS, double &DH);
	void LWPOut();
	void LTPOut();
	void OMS2();

	//Nominal semimajor axis at insertion
	double ANOM;
	//Time iteration tolerance
	double DET;
	//Radius iteration tolerance
	double DELH;
	//Flag for option to compute differential nodal regression from insertion to rendezvous
	//false = input DELNO, true = compute DELNO
	bool DELNOF;
	//Differential nodal precession
	double DELNOD;
	//Time step in parameter table
	double TSTEP;
	//First guess change in value of independent variable (used in subroutine ITER)
	double DX1;
	//Tolerance on plane change DV for calculating launch window inplane launch points
	double DVTOL;
	//Phase angle at window closing (descending node)
	double PAC;
	//Phase angle at window opening (ascending node)
	double PAO;
	//GMT of lift-off for launch window opening (ascending node)
	double OPEN;
	//GMT of lift-off for launch window closing (descending node)
	double CLOSE;
	//GMT of insertion
	double GMTINS;
	//Wedge angle between planes
	double WEDGE;
	//Launch window parameter table options
	//0 = do not generate table, 1 = window opening, 2 = window closing, 3 = both, 4 = entire window
	int LPT;
	//GMT of lift-off
	double GMTLO;
	//Inclination of chaser at insertion
	double IIGM;
	//Angle measured from launch site meridian to chaser descending node
	double TIGM;
	//Optimum launch azimuth
	double AZL;
	//Target vector Earth-fixed descending node
	double DN;
	//Phase angle at insertion
	double PA;
	//Target position vector
	VECTOR3 RT;
	//Target velocity vector
	VECTOR3 VT;
	//Target vector time
	double TT;
	//Chaser position vector
	VECTOR3 RP;
	//Chaser velocity vector
	VECTOR3 VP;
	//Chaser vector time
	double TP;
	//Number of GMTLO* times computed
	int K25;
	//GMTLO* time array
	double STAR[10];
	//Last GMTLO* time computed
	double GSTAR;
	//GMTLO* table flag. 1 = compute table, 2 = don't compute table
	int STABLE;
	//Time of minimum yaw steering
	double TYAW;
	//Unit launch site position in TEG coordinates
	VECTOR3 URLS;
	//Velocity-to-go of OMS-2
	VECTOR3 VGO_OMS2;

	//Constants
	//Maximum iterations limit
	int CMAX;

	LWPSettings inp;

	LWPOutput *lwp_table;
	LTPOutput *ltp_table;
	int error;
};