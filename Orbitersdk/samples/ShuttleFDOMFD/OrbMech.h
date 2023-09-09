/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD Orbital Mechanics Calculations (Header)

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

namespace OrbMech
{
	const double LAUNCHSITE_LATITUDE[3] = { 28.60833333, 28.627, 34.580847 };
	const double LAUNCHSITE_LONGITUDE[3] = { -80.60416667, -80.621, -120.62595 };

	const double EARTH_RADIUS_EQUATOR = 6378166.0;
	const double EARTH_RADIUS_GRAV = 6.37101e6;
	const double mu_Earth = 398600439968871.2;
	const double J2_Earth = 1082.6269e-6;
	const double J3_Earth = -2.51e-6;
	const double J4_Earth = -1.60e-6;
	const double w_Earth = PI2 / 86164.10132;

	const double LBM2KG = 0.45359237;
	const double FPS2MPS = 0.3048;		//Feet to meters
	const double NM2M = 1852.0;			//Nautical miles to meters

	struct SV
	{
		VECTOR3 R = _V(0, 0, 0);
		VECTOR3 V = _V(0, 0, 0);
		double GMT = 0.0;
		double mass = 0.0;
	};

	struct InvariantElements
	{
		double TIMV = 0.0;
		double a = 0.0;
		double e = 0.0;
		double i = 0.0;
		double g = 0.0;
		double h = 0.0;
		double l = 0.0;
		double l_dot = 0.0;
		double g_dot = 0.0;
		double h_dot = 0.0;
		double CD = 0.0;
		double Area = 0.0;
		double Mass = 0.0;
		std::string VectorName;
	};

	struct OELEMENTS
	{
		double h = 0.0;
		double e = 0.0;
		double i = 0.0;
		double RA = 0.0;
		double w = 0.0;
		double TA = 0.0;
	};

	//Classical elements
	struct CELEMENTS
	{
		//Semi-major axis
		double a = 0.0;
		//Eccentricity
		double e = 0.0;
		//Inclination
		double i = 0.0;
		//Longitude of the ascending node
		double h = 0.0;
		//Argument of pericenter
		double g = 0.0;
		//Mean Anomaly
		double l = 0.0;

		CELEMENTS operator+(const CELEMENTS&) const;
		CELEMENTS operator-(const CELEMENTS&) const;
	};

	struct ITERSTATE
	{
		int s_F = 0;
		double p_H = 0.0;
		double c_I = 0.0;
		double dv = 0.0;
		double dvo = 0.0;
		double err = 0.0;
		double erro = 0.0;
		bool converged = false;
	};

	//Trajectory computations
	void oneclickcoast(VECTOR3 R0, VECTOR3 V0, double dt, VECTOR3 &R1, VECTOR3 &V1);
	void rv_from_r0v0(VECTOR3 R0, VECTOR3 V0, double t, VECTOR3 &R1, VECTOR3 &V1, double mu, double x = 0.0);
	void rv_from_r0v0_obla(VECTOR3 R1, VECTOR3 V1, double dt, VECTOR3 &R2, VECTOR3 &V2, OBJHANDLE gravref);
	double kepler_U(double dt, double ro, double vro, double a, double mu, double x0);
	void f_and_g(double x, double t, double ro, double a, double &f, double &g, double mu);
	void fDot_and_gDot(double x, double r, double ro, double a, double &fdot, double &gdot, double mu);
	double kepler_E(double e, double M, double error2 = 1.e-8);
	double time_theta(VECTOR3 R, VECTOR3 V, double dtheta, double mu, bool future = true);
	void f_and_g_ta(VECTOR3 R0, VECTOR3 V0, double dt, double &f, double &g, double mu);
	void fDot_and_gDot_ta(VECTOR3 R0, VECTOR3 V0, double dt, double &fdot, double &gdot, double mu);
	double period(VECTOR3 R, VECTOR3 V, double mu);
	double timetoapo(VECTOR3 R, VECTOR3 V, double mu, int s = 0);
	double timetoapo_integ(VECTOR3 R, VECTOR3 V, double GMT);
	double timetoapo_integ(VECTOR3 R, VECTOR3 V, double GMT, VECTOR3 &R2, VECTOR3 &V2);
	double timetoperi(VECTOR3 R, VECTOR3 V, double mu, int s = 0);
	double kepler_U_equation(double x, double ro, double vro, double a, double mu);
	void REVUP(VECTOR3 R, VECTOR3 V, double n, double mu, VECTOR3 &R1, VECTOR3 &V1, double &t);
	void RADUP(VECTOR3 R_W, VECTOR3 V_W, VECTOR3 R_C, double mu, VECTOR3 &R_W1, VECTOR3 &V_W1);
	bool CSIToDH(VECTOR3 R_A1, VECTOR3 V_A1, VECTOR3 R_P2, VECTOR3 V_P2, double DH, double mu, double &dv);
	void ITER(double &c, int &s, double e, double &p, double &x, double &eo, double &xo, double dx0 = 1.0);
	VECTOR3 elegant_lambert(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double dt, int N, bool prog, double mu);
	VECTOR3 Vinti(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double gmt0, double dt, int N, bool prog, VECTOR3 V_guess, double tol = 0.1);
	void periapo(VECTOR3 R, VECTOR3 V, double mu, double &apo, double &peri);
	void umbra(VECTOR3 R, VECTOR3 V, VECTOR3 sun, OBJHANDLE planet, bool rise, double &v1);
	double sunrise(VECTOR3 R, VECTOR3 V, double GMT, double BaseMJD, MATRIX3 Rot, OBJHANDLE planet, OBJHANDLE planet2, bool rise, bool midnight, bool future = false);
	void orbitmidnight(VECTOR3 R, VECTOR3 V, VECTOR3 sun, OBJHANDLE planet, bool night, double &v1);
	//Analytical sun ephemeris
	VECTOR3 SUN(double MJD, const MATRIX3 &RM);
	void BrouwerSecularRates(CELEMENTS coe_osc, CELEMENTS coe_mean, double &l_dot, double &g_dot, double &h_dot);
	CELEMENTS AnalyticEphemerisGenerator(CELEMENTS osc0, int opt, double dval, double DN, double mu, double &DeltaTime);
	void AEGServiceRoutine(VECTOR3 R, VECTOR3 V, double GMT, int opt, double dval, double DN, VECTOR3 &R2, VECTOR3 &V2, double &GMT_out);
	void poweredflight(VECTOR3 R, VECTOR3 V, double f_T, double v_ex, double m, VECTOR3 V_G, bool nonspherical, VECTOR3 &R_cutoff, VECTOR3 &V_cutoff, double &m_cutoff, double &t_go);
	VECTOR3 gravityroutine(VECTOR3 R, bool nonspherical);
	double GetSemiMajorAxis(VECTOR3 R, VECTOR3 V, double mu);
	double GetMeanMotion(VECTOR3 R, VECTOR3 V, double mu);
	double findlatitude(VECTOR3 R, VECTOR3 V, OBJHANDLE gravref, double lat, bool up, VECTOR3 &Rlat, VECTOR3 &Vlat);
	double findlatitude_integ(VECTOR3 R, VECTOR3 V, OBJHANDLE gravref, double lat, bool up, VECTOR3 &Rlat, VECTOR3 &Vlat);
	bool impulsive(VECTOR3 R, VECTOR3 V, double GMT, double f_T, double f_av, double isp, double m, VECTOR3 DV, bool nonspherical, VECTOR3 &Llambda, double &t_slip, VECTOR3 &R_cutoff, VECTOR3 &V_cutoff, double &GMT_cutoff, double &m_cutoff);
	double COMELE(VECTOR3 RS_COM, VECTOR3 VS_COM, VECTOR3 RT_COM);
	//Apogee and Perigee Radius Magnitude
	void PCHAPE(double R1, double R2, double R3, double U1, double U2, double U3, double &RAP, double RPE);
	//Apogee/Perigee Magnitude Determination
	void ApsidesMagnitudeDetermination(SV sv0, double &r_A, double &r_P);
	void PIFAAP(double a, double e, double i, double f, double u, double r, double R_E, double &r_A, double &r_P);
	SV coast_auto(SV sv0, double dt, bool precision);
	SV coast(SV sv0, double dt);
	SV coast_osc(SV sv0, double dt, double mu);
	SV GeneralTrajectoryPropagation(SV sv0, int opt, double param, double DN, bool precision);
	InvariantElements CalculateInvariantElementsBlock(SV sv, double mu, double Area, bool precision);
	VECTOR3 PROJCT(VECTOR3 U1, VECTOR3 U2, VECTOR3 X);
	double PHSANG(VECTOR3 R, VECTOR3 V, VECTOR3 RD);
	double REVTIM(VECTOR3 R, VECTOR3 V, bool SPERT);

	//Conversions
	OELEMENTS coe_from_sv(VECTOR3 R, VECTOR3 V, double mu);
	void sv_from_coe(OELEMENTS el, double mu, VECTOR3 &R, VECTOR3 &V);
	CELEMENTS CartesianToKeplerian(VECTOR3 R, VECTOR3 V, double mu);
	void KeplerianToCartesian(CELEMENTS coe, double mu, VECTOR3 &R, VECTOR3 &V);
	MATRIX3 GetRotationMatrix(double t, bool earth = true);
	MATRIX3 GetObliquityMatrix(double t, bool earth = true);
	VECTOR3 Polar2Cartesian(double r, double lat, double lng);
	VECTOR3 Polar2CartesianVel(double r, double lat, double lng, double r_dot, double lat_dot, double lng_dot);
	VECTOR3 rhmul(const MATRIX3 &A, const VECTOR3 &b);
	VECTOR3 rhtmul(const MATRIX3 &A, const VECTOR3 &b);
	MATRIX3 MatrixRH_LH(MATRIX3 A);
	void REL_COMP(VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3 &R_S_INER, VECTOR3 &R_REL);
	void REL_COMP(bool INER_TO_LVC, VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3 &R_S_INER, VECTOR3 &V_S_INER, VECTOR3 &R_REL, VECTOR3 &V_REL);
	double MJDToDate(double MJD);
	//Rotation matrix from inertial to LVLH
	MATRIX3 LVLH_Matrix(VECTOR3 R, VECTOR3 V);
	int Date2JD(int Y, int M, int D);
	//Convert date to MJD
	double Date2MJD(int Y, int D, int H, int M, double S);
	void mjd2date(double mjd, int &year, int &month, int &day, int &hour, int &minute, double &second);
	void mjd2ydoy(double mjd, int &Y, int &D, int &H, int &M, double &S);
	double jd2mjd(double jd);
	double mjd2jd(double mjd);
	void jd2date(double jd, int &year, int &month, int &day);
	void days2hms(double days, int &hour, int &minute, double &second);
	int dayofyear(int year, int month, int day);
	bool isleapyear(int a);
	CELEMENTS BrouwerMeanLongToOsculatingElements(CELEMENTS mean);
	CELEMENTS CartesianToBrouwerMeanLong(VECTOR3 R, VECTOR3 V, double mu);
	CELEMENTS OsculatingToBrouwerMeanLong(CELEMENTS osc, double mu);
	double MeanToTrueAnomaly(double meanAnom, double eccdp, double error2 = 1e-12);
	double TrueToMeanAnomaly(double ta, double eccdp);
	double TrueToEccentricAnomaly(double ta, double ecc);
	void latlong_from_r(VECTOR3 R, double &lat, double &lng);

	//Math
	double stumpS(double z);
	double stumpC(double z);
	double power(double b, double e);
	template <typename T> int sign(T val);
	double cot(double a);
	double sec(double a);
	double acos2(double _X);
	double asin2(double _X);
	double fraction_an(int n);
	double fraction_ad(int n);
	double fraction_a(int n, double x);
	double fraction_b(int n, double x);
	double fraction_delta(int n, double x);
	double fraction_u(int n, double x);
	double fraction_pq(double x);
	double fraction_xi(double x);
	double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle);
	double normalize_angle(const double value, const double start, const double end);
	MATRIX3 inverse(MATRIX3 a);
	double determinant(MATRIX3 a);
	MATRIX3 tmat(MATRIX3 a);

	class CoastIntegrator
	{
	public:
		CoastIntegrator(VECTOR3 R0, VECTOR3 V0, double dt);
		~CoastIntegrator();
		bool iteration();

		VECTOR3 R2, V2;
	private:
		VECTOR3 f(VECTOR3 alpha, VECTOR3 R, VECTOR3 a_d);
		double fq(double q);
		VECTOR3 adfunc(VECTOR3 R);

		double R_E, mu;
		double K, dt_lim;
		int jcount;
		double *JCoeff;
		VECTOR3 R00, V00, R0, V0, R_CON, V_CON;
		double t_0, t, tau, t_F, x;
		VECTOR3 delta, nu;
		OBJHANDLE hEarth;
		double rect1, rect2;
		VECTOR3 U_Z;
	};

}