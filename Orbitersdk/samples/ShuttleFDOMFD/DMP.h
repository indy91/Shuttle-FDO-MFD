/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Deorbit Maneuver Processor (Header)

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

#include "Orbitersdk.h"

struct DMPOptions
{
	//TIG mode selected flag. 0 = TIG fixed, 1 = TIG free
	int ITIGFR = 1;
	//Deorbit ignition time (for TIG fixed)
	double TIG = 0.0;
	//Deorbit threshold time (for TIG free)
	double TTHRSH = 0.0;
	//Propellant use mode (1 = fuel wasting, 2 = in-plane)
	int IFUEL = 2;
	//Weight of OMS to be burned
	double WCGOMS = 0.0;
	//Primary and backup engine config (16 = 2OMS, 14 = 1OMS, 12 = RCS)
	int INGPR = 16, INGBU = 14;
	//Ordinate intercept on VV-VH target line
	double C1IP = 0.0;
	//Slope of the VV-VH target line
	double C2IP = 0.0;
	//Minimum allowed free fall time
	double TFFMIN = 9.0*60.0;
	//Range target from EI to landing site
	double RNGIP = 0.0;
	//PEG guidance option (3 = PEG4, 7 = PEG7)
	int KDGUID = 3;

	//DTMVEC
	//Input state vector
	double TIMEC;
	VECTOR3 XYZI;
	VECTOR3 XYZID;
	double CD;
	double AREA;
	//Weight
	double WT;

	//false = conic, true = integrated
	bool INTEGF;
	//Print options. 0 = final print only, 1 = final print with flag settings, 2 = interim printing with flags
	int IPOUT = 0;
	//Landing site geodetic latitude
	double TLATD;
	//Landing site geodetic longitude
	double TLONG;
	//Landing site geodetic altitude
	double TALTD;
	//Landing site azimuth
	double RAZ;
	//OMS one engine thrust
	double FOMS = 6000.0 * 4.44822;
	//RCS four engine thrust
	double FRCS = 3614.74*4.44822;
	//OMS one engine flow rate
	double XMDOMS = 19.157088*0.45359237;
	//RCS four engine flow rate
	double XMDRCS = 13.623*0.45359237;
};

class DMP
{
public:
	DMP();

	void Executive(const DMPOptions &opt);
protected:
	//DTM2 executive routine (TIG free)
	void DTM2();
	//DTM4 executive routine (TIG fixed)
	void DTM3();
	//DTM4 executive routine (impulsive minimum DV solution)
	void DTM4();
	//DTM5 executive routine (basic targeting)
	void DTM5();
	//DTM6 executive routine (equal yaw angle)
	void DTM6();
	//DTM7 executive routine (prime and backup solution)
	void DTM7();
	//DTM8 executive routine (equal OMS)
	void DTM8();
	//DTM14 executive routine (offset targeting)
	void DTM14();

	//Finite burn
	void DTT3();
	//Out of plane direction
	void DTT4();
	//Equal yaw angle or OMS usage between primary and backup system
	void DTT5();

	//Segment 1
	//Initialization and units conversion task
	void DTT1();
	void GEOD(double GHD, double GPHID, double &GR, double &GPHIC);

	//Segment 2
	//Minimum DV
	void DTT2();
	//Minimization routine
	void GLPRP();

	//Segment 6
	//Error messages
	void DTMER();

	//Segment 7
	//Propagate state vector to ignition time or to entry interface
	void DTT7();
	void UPDTV(VECTOR3 RS, VECTOR3 VS, double TS, double TF, VECTOR3 &RF, VECTOR3 &VF);

	//Segment 8
	//LVLH reference frame
	void DTT8();

	//In-plane position at EI
	void DTT10();
	double HLIP(VECTOR3 U);

	//Segment 11
	//Impulsive burn deorbit solution
	void DTT11();
	//Conic intercept
	void LTVCN(double C1, double C2, VECTOR3 R0, VECTOR3 R1, VECTOR3 XNUNIT, double &DELTAT, int &KFLAG, VECTOR3 &V0, VECTOR3 &V1);

	//PEG
	void DTT12();
	void PGSUP();
	void H2M50(double THETA, double &DTHETA, VECTOR3 &RT);
	void PGOP3();
	void INI1();
	void PRDT6();
	void CORT7();
	void LTVCN2(double C1, double C2, VECTOR3 R0, VECTOR3 R1, VECTOR3 XNUNIT, double &THETA, int &KFLAG, VECTOR3 &V0);
	double HELIP(VECTOR3 R);
	void SUPRG(double DT, VECTOR3 DVS, double J2FLAG, VECTOR3 &RF, double TF, double TI, VECTOR3 &VF);

	//Segment 13
	//Entry range
	void DTT13();
	//Geodetic latitude, longitude, altitude to Earth fixed position vector
	VECTOR3 GD2EF(double GLAT, double GLON, double ALT);
	//Earth fixed to topodetic
	MATRIX3 EF2TD(double GLAT, double XLON);
	//Rotation matrix
	MATRIX3 ROTMX(double ANGLE, int IAXIS);
	//Earth-relative velocity
	VECTOR3 VREL(VECTOR3 R, VECTOR3 V);
	//Earth-fixed to M50
	MATRIX3 EF2MF(double TIME);
	//Earth-fixed to geodetic
	void EF2GD(VECTOR3 R, double &GLAT, double &XLON, double &XALT);
	void EGTR();

	//Segment 14
	//Entry range
	void DTT14();
	void FVE(double VE, double &DTAE, double &FPAE);
	void UpdateC1C2(double VE, double &C1, double &C2);

	//Null range error
	void DTT15();
	//Null range error, too
	void DTT16();
	//End task for DTM5
	void DTT17();
	//Advance TIG by one period
	void DTT21();
	//Initialization routine offset targeting
	void DTT22();
	//Correction factor offset target
	void DTT23();
	//State extrapolation with J2
	void SUPRJ();
	//Gravity routine
	VECTOR3 GRAVJ(VECTOR3 RF, double J2);
	//Crossrange
	void DTT24();
	//Output
	void DTMOT();
	//User display
	void DTMPR();

	DMPOptions opt;

	//Flags

	//Abort selection flag, always 0
	bool IAME;
	//Range adjustment pass flag (false = not first pass, true = first pass)
	bool IBTR;
	//ETG call select flag (false = no ETG call, true = call ETG)
	bool ICALL;
	//DTM main routine pass flag. 1 = first pass, >1 TIG & threshold pass
	int IDTM;
	//ETG final call flag (false = ETG approximate mode, true = ETG precise mode)
	bool IFINAL;
	//Free flight predictor route flag (false = EI, true = TIG)
	bool IFFPTM;
	//Impulsive solution flag (0 = finite burn, 1 = impulsive)
	bool IMPULS;
	//DTM error message flag
	int IMSG;
	//ETG precision mode flag (false = approximate range solution, true = precise range solution)
	bool IPRETG;
	//ETG range selection flag (false = use ETG computed range, true = use input range, C1, C2)
	bool IPRNG;
	//GOLPAR status flag
	int JJ;
	//DTM iteration counter
	int NCNT;
	//DTT21 iteration counter
	int NCNT1;
	//DTT5 iteration counter
	int NCNT2;
	//DTM8 iteration counter
	int NCNT3;
	//DTM4 iteration counter
	int NCNT4;
	//System selection flag (1 = primary, 2 = backup)
	int NSYS;
	//Max iterations diagnostics flag
	int MITER;
	//Iterations required in DTT7 to converge on TEI for offset targeting
	int XOFCN;
	//Steering mode flag (false = nominal PEG4 turning rate, true = zero turning rate for inertial hold) - Same as KSTEER
	bool KDSTER;
	//Shallow target line flag, always 0
	bool INCR;
	//Free fall time constraint flag (1 = normal solution, 2 = TFF forced to constraint, 3 = DV exceeds DV available, 4 = TFF constraint exceeded)
	int ITFF;
	//DTT5 pass flag (0 = first pass set ITIGFR = 2, 1 = first pass computations, >1 = second or greater pass)
	int ITIGG;
	//Compute C2 flag
	bool IC2FLG;
	//PEG initial pass flag. 0 = no initialization, 1 = first pass initialization
	bool KINIT;
	//Weight of fuel to be burned x SBD
	double WCG;
	//PEG entry interface altitude
	double HTGT;
	//Desired transfer angle TIG to EI
	double THETAT;

	//GOLPAR historical data array (0 = lower bound angle, 1 = upper bound angle, 2 = , 6 = DV, 7 = DV)
	double AAA[10];
	//Past value of THETEI in minimum DV search
	double THEPEI;
	//Orbital rate
	double THETD;
	//TIG to EI transfer angle
	double THETEI;
	//Position vector at TIG
	VECTOR3 RAAA;
	//Velocity vector at TIG
	VECTOR3 VA;
	//Burnout vector
	VECTOR3 RDA, VDA;
	//Time of burnout
	double TP;
	//Time of entry interface
	double TT;
	//EI position and velocity vector (post offset targeting)
	VECTOR3 RENK, VENK;
	//LVLH downrange, out of plane, and radial unit vectors
	VECTOR3 UXDTM, UYDTM, UZDTM;
	//Unit vector at entry interface
	VECTOR3 UEI;
	//EI position vector
	VECTOR3 RTA;
	//Velocity vector at EI
	VECTOR3 VTAA;
	//Intercept and slope of the V(V)-V(H) target line
	double CA[2];
	//Coast time from burnout to EI
	double DTCOST;
	//Impulsive DV
	double DVIMP;
	//Unit vector in impulsive thrust direction
	VECTOR3 UFIMP;
	//Angle from burnout to EI
	double THEBO;
	//Time at EI (pre offset targeting)
	double TEIS;
	//EI position vector (pre offset targeting)
	VECTOR3 REIS;
	//EI velocity vector (pre offset targeting)
	VECTOR3 VEIS;
	//Magnitude of REIS vector
	double REISM;
	//Time at EI (post offset targeting)
	double TEINK;
	//Earth fixed position vector
	VECTOR3 RLS;
	//Earth rotation angle at initial input vector time
	double ERAI;
	//Earth fixed entry interface position vector
	VECTOR3 XYZEN;
	//Earth fixed to runway matrix
	MATRIX3 REC;
	//Earth radius at landing site
	double RCHMAG;
	//EGTR computed range, nautical miles
	double TRANG;
	//V-rho topodetic
	VECTOR3 VRHOTD;
	//Variable to compute sign of THETLS
	double DELAZ;
	//Actual range angle EI to landing site
	double THETLS;
	//Required entry range from DTT14, nautical miles
	double ETGRNG;
	//Desired range angle EI to landing site
	double THELSD;
	//Relative closing rate between the vehicle and landing site
	double THETDR;
	//Primary system thrust
	double FPR;
	//Backup system thrust
	double FBU;
	//Primary system flow rate
	double WDPR;
	//Backup system flow rate
	double WDBU;
	//Convergence criteria for VMISS
	double SMISS;
	//Primary and backup DV available
	double DVOBPR, DVOBBU;
	//DT between TIG and TTHRSH
	double DTTHRS;
	//Sign of out of plane burn direction
	double SBD;
	//TIG adjust parameter = DWBU-DWPR
	double DY2;
	//Previous value of DY2
	double DY1;
	//Minimum value of denominator in equal OMS logic
	double TGDMN;
	//Initial slope for equal OMS yaw logic
	double TGSLPP;
	//Previous value of TIG
	double TIG1;
	//Primary and backup systems weight of propellant expended
	double DWPR, DWBU;
	//Backup system free fall time
	double TFFBV;
	//Burnout weight
	double WBO;
	//Required entry flight path angle
	double GAMETG;
	//Angle from EI to landing site
	double DTHELS;
	//Flight path angle error used in offset targeting
	double DGAM;
	//C1 offset value
	double DC11;
	//Conic horizontal and vertical velocity components at entry interface
	double VHS, VVS;
	//Burn DT
	double TGO;
	//Step size for gravity prediction
	double DTAVG;
	//Geocentric latitude
	double TLATC;
	//Time of closest approach
	double TC;
	//Crossrange in NM
	double CRSRNG;
	//Downrange in NM
	double DNRNG;
	double THETA_DOT;

	//COMMON

	//Unit vector in thrust direction computed in PEG. COM(16)
	VECTOR3 ATP;
	//Unit vector in thrust direction for backup system. COM(19)
	VECTOR3 ATPBU;
	//Unit vector in thrust direction for primary system. COM(22)
	VECTOR3 ATPPR;
	//Engine configuration flag in PEG (1 = RCS, 14 = one OMS, 16 = two OMS)
	int KDTHR;
	//PEG TIG state initialization flag ICOM(794)
	bool KITER = true;

	//Backup system deorbit Delta-V. COM(111)
	double DVBU;
	//Prime system deorbit Delta-V. COM(112)
	double DVPR;
	//Thrust of selected system in PEG COM(135)
	double FT;
	//Out of plane fuel wasting angle COM(151)
	double FWYAW;
	//Backup system out of plane yaw angle. COM(405)
	double PSIBU;
	//Primary system out of plane yaw angle. COM(406)
	double PSIPR;
	//Backup system free fall time. COM(492)
	double TFFBU;
	//Primary system free fall time. COM(494)
	double TFFPR;
	//Total effective exhaust veloicty COM(550)
	double VEXX;
	//Inertial Delta-V components COM(553)
	VECTOR3 VG;
	//Impulsive Delta-V magnitude COM(556)
	double VGIMP;
	//Magnitude of VG from PEG COM(557)
	double VGMAG;

	//PEG COMMON

	//Estimated thrust acceleration. PEGCOM(4)
	double ATR;
	//Fractional multiplier of orbit rate PEGCOM(5)
	double CLMDXZ;
	//Change in accumulated sensed velocity. PEGCOM(8)
	VECTOR3 DVSS;
	//Saved thrust value for selected system PEGCOM(11)
	double FTS;
	//Scalar damping factor applied to VMISS to correct VGO PEGCOM(19)
	double RHOO;
	//Delta-t of RCS during parallel burn. PEGCOM(20)
	double TBRCS = 0.0;
	//Previous value of guidance time PEGCOM(21)
	double TGDP;
	//Reference time of XLAM and XLDA PEGCOM(23)
	double TLAM;
	//Reference time of XLAMC and XLAMDC PEGCOM(24)
	double TLAMC;
	//RCS cutoff time during AOA. PEGCOM(26)
	double TRCSOF;
	//Current guidance time PEGCOM(27)
	double TTT;
	//Unit vector normal to desired trajectory plane PEGCOM(29)
	VECTOR3 UYY;
	//Saved value of total effective exhaust velocity PEGCOM(33)
	double VEXS;
	//Inertial velocity to be gained PEGCOM(37)
	VECTOR3 VGDP;
	//Predicted thrust cutoff velocity PEGCOM(40)
	VECTOR3 VP;
	//Previous pass sensed velocity PEGCOM(43)
	VECTOR3 VSP;
	//Unit position vector in VGO direction. PEGCOM(46)
	VECTOR3 XLAM;
	//Unit position vector in VGO direction. PEGCOM(49)
	VECTOR3 XLAMC;
	//Unit velocity vector in VGO direction. PEGCOM(52)
	VECTOR3 XLAMDC;
	//Unit velocity vector in VGO direction PEGCOM(55)
	VECTOR3 XLDA;
	//Fractional multiplier of orbit rate PEGCOM(58)
	double XLDXZ;
	//Current mass of vehicle PEGCOM(59)
	double XMASSE;
	//Desired mass of vehicle at thrust cutoff PEGCOM(60)
	double XMBO;
	//Current mass flow rate PEGCOM(61)
	double XMDOT;
	//Unit vector normal to trajectory plane PEGCOM(68)
	VECTOR3 UYD;

	//Abort guidance flag
	bool KABORT = false;
	//Active guidance termination flag IPGCOM(28)
	bool KCUTOF;
	//PEG error indicator flag IPGCOM(29)
	int KFLAG;
	//Out of plane fuel depletion flag IPGCOM(30)
	bool KFUELD;
	//PEG solution convergence flag IPGCOM(31)
	bool KSTOP;
	//Number of PEG calls per guidance cycles. IPGCOM(32)
	int NCYC;
	//Number of OMS used in two phase maneuver IPGCOM(35)
	int NOMS;
	//Number of active guidance thrust phases used. IPGCOM(36)
	int NPHASE;

	//Flag for FWYAW computation. PO3CM(1)
	bool IYAW;
	//Intermediate thrust integral variable. PO3COM(2)
	double QPRIME;
	//Final position vector computed in precise predict. PO3COM(3)
	VECTOR3 RC2;
	//Position to go vector including range bias. PO3COM(6)
	VECTOR3 RGO;
	//Predicted thrust cutoff position. PO3COM(9)
	VECTOR3 RP;
	//Burn centroid delta-t. PO3COM(12)
	double TIJOL;
	//Twice the thrust integral from 0 to T plus 0 to TGO. PO3COM(13)
	double TIS;
	//Ratio of mass to mass flow rate. PO3COM(14)
	double TAU;
	//Previous predicted burnout time value. PO3COM(15)
	double TPREV;
	//Predicted cutoff velocity from precise predict. PO3COM(21)
	VECTOR3 VC2;

	//Constants
	//Distance from runway to TAEM alinement circle
	const double DBARR = 31993.0*0.3048;
	//Alinement circle radius
	const double RTURNN = 20000.0*0.3048;

	//Range convergence tolerance (0.0167°)
	const double RNGTOL = 0.00029147;

	//Initial value of TIG adjustment
	const double DTIGO = 100.0;
	//Maximum value of TIG adjustment
	const double DTGMAX = 200.0;

	//Weight tolerance on equal OMS check
	const double DWTOL = 25.0*0.45359237;

	//Tolerance on gamma convergence (0.01°)
	const double GAMTOL = 0.0001745329;

	//GOLPAR constants array (1°, 1°, DVMAX, 0°, 360°)
	const double SSS[5] = { 0.01745329, 0.01745329, 5000.0*0.3048, 0.0, 6.283185 };
	//Maximum iterations during initialization
	const int NMAX1 = 10;
	//Maximum PEG iterations
	const int NMAX2 = 1;
	//Number of steps used in gravity prediction
	const int NSEG = 20;
	//Factor for computing the damping factor applied to VMISS for correcting VGO
	const double RHOI = 0.75;
	//PEG convergence factor
	const double SMISSI = 0.001;
	//Guidance delta time before thrust termination
	const double TGOMIN = 6.0;
	//Circular orbir rate multiplier for PEG
	const double XLD = 0.0;
	//Maximum time step for gravity prediction
	const double DTMAXX = 25.0;
	//Minimum time step for gravity prediction
	const double DTMINN = 2.0;
	//GOLPAR constant
	const double GGOL = 0.381966;
	//Initial theta EI for TIG fixed mode (73°)
	const double THEIIP = 1.27409;

	//Crossrange computation tolerance
	const double CRTOL = 0.0001;
	//Minimum denominator in crossrange logic
	const double TDENMN = 0.0001;
	//Initial delta time in crossrange logic
	const double DTCR0 = 10.0;
	//Minimum denominator in equal OMS logic
	const double TGDOMS = 1.0;

	//Bias time for minimum DV TIG free solution
	const double TTHRB = 60.0;
	//Initial slope for equal OMS prediction
	const double TGSLPV = 10.0;
	//Initial slope for equal yaw angle logic
	const double TGSLPY = 572.96;
	//DTM iteration limit
	const int NCMAX = 20;
	//Unit vector in direction of Earth pole
	const VECTOR3 POLE = _V(0, 0, 1);
	const MATRIX3 TM502I = _M(1, 0, 0, 0, 1, 0, 0, 0, 1);

	const double FVECON_VEL[5] = { 25600.0*0.3048, 25700.0*0.3048, 25727.87*0.3048, 25800.0*0.3048, 25900.0*0.3048 };
	const double FVECON_FPA[5] = { -0.01782556, -0.02034792, -0.02096274, -0.02253098, -0.02447867 };
	const double FVECON_RNGG[5] = { 24926650.0*0.3048, 24847060.0*0.3048, 24864070.0*0.3048, 24873190.0*0.3048, 24984800.0*0.3048 };

	//Global constants
	const double POLRAD = 6.37101e6;
	const double EQRAD = 6.37101e6;
	const double EIALT = 400000.0*0.3048;
	const double XMU = 398600439968871.2;
	const double ELLIPT = 1.0;
	const double EARTHR = PI2 / 86164.10132;
	const double XJ2 = 1082.6269e-6;
	const double AMILE = 1852.0;
};
