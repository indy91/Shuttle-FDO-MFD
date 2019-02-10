/****************************************************************************
  This file is part of OMP MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  OMP MFD Orbital Mechanics Calculations (Header)

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

namespace OrbMech
{
	struct OELEMENTS
	{
		double h = 0.0;
		double e = 0.0;
		double i = 0.0;
		double RA = 0.0;
		double w = 0.0;
		double TA = 0.0;
	};

	//Trajectory computations
	void oneclickcoast(VECTOR3 R0, VECTOR3 V0, double mjd0, double dt, VECTOR3 &R1, VECTOR3 &V1);
	void rv_from_r0v0(VECTOR3 R0, VECTOR3 V0, double t, VECTOR3 &R1, VECTOR3 &V1, double mu, double x = 0.0);
	void rv_from_r0v0_obla(VECTOR3 R1, VECTOR3 V1, double MJD, double dt, VECTOR3 &R2, VECTOR3 &V2, OBJHANDLE gravref);
	double kepler_U(double dt, double ro, double vro, double a, double mu, double x0);
	void f_and_g(double x, double t, double ro, double a, double &f, double &g, double mu);
	void fDot_and_gDot(double x, double r, double ro, double a, double &fdot, double &gdot, double mu);
	double kepler_E(double e, double M);
	double time_theta(VECTOR3 R, VECTOR3 V, double dtheta, double mu, bool future = true);
	void f_and_g_ta(VECTOR3 R0, VECTOR3 V0, double dt, double &f, double &g, double mu);
	void fDot_and_gDot_ta(VECTOR3 R0, VECTOR3 V0, double dt, double &fdot, double &gdot, double mu);
	double period(VECTOR3 R, VECTOR3 V, double mu);
	double timetoapo(VECTOR3 R, VECTOR3 V, double mu, int s = 0);
	double kepler_U_equation(double x, double ro, double vro, double a, double mu);
	void REVUP(VECTOR3 R, VECTOR3 V, double n, double mu, VECTOR3 &R1, VECTOR3 &V1, double &t);
	void RADUP(VECTOR3 R_W, VECTOR3 V_W, VECTOR3 R_C, double mu, VECTOR3 &R_W1, VECTOR3 &V_W1);
	bool CSIToDH(VECTOR3 R_A1, VECTOR3 V_A1, VECTOR3 R_P2, VECTOR3 V_P2, double DH, double mu, double &dv);
	void ITER(double &c, int &s, double e, double &p, double &x, double &eo, double &xo, double dx0 = 1.0);
	VECTOR3 HeightManeuver(VECTOR3 R, VECTOR3 V, double dh, double mu);
	VECTOR3 elegant_lambert(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double dt, int N, bool prog, double mu);
	VECTOR3 Vinti(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double mjd0, double dt, int N, bool prog, VECTOR3 V_guess, double tol = 0.1);
	void periapo(VECTOR3 R, VECTOR3 V, double mu, double &apo, double &peri);
	void umbra(VECTOR3 R, VECTOR3 V, VECTOR3 sun, OBJHANDLE planet, bool rise, double &v1);
	double sunrise(VECTOR3 R, VECTOR3 V, double MJD, OBJHANDLE planet, OBJHANDLE planet2, bool rise, bool midnight, bool future = false);
	void orbitmidnight(VECTOR3 R, VECTOR3 V, VECTOR3 sun, OBJHANDLE planet, bool night, double &v1);

	//Conversions
	OELEMENTS coe_from_sv(VECTOR3 R, VECTOR3 V, double mu);
	void sv_from_coe(OELEMENTS el, double mu, VECTOR3 &R, VECTOR3 &V);
	MATRIX3 GetObliquityMatrix(OBJHANDLE plan, double t);
	MATRIX3 GetRotationMatrix(OBJHANDLE plan, double t);
	VECTOR3 Polar2Cartesian(double r, double lat, double lng);
	VECTOR3 Polar2CartesianVel(double r, double lat, double lng, double r_dot, double lat_dot, double lng_dot);
	VECTOR3 rhmul(const MATRIX3 &A, const VECTOR3 &b);
	VECTOR3 rhtmul(const MATRIX3 &A, const VECTOR3 &b);
	void REL_COMP(VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3 &R_S_INER, VECTOR3 &R_REL);
	void REL_COMP(bool INER_TO_LVC, VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3 &R_S_INER, VECTOR3 &V_S_INER, VECTOR3 &R_REL, VECTOR3 &V_REL);
	double MJDToDate(double MJD);
	//Rotation matrix from inertial to LVLH
	MATRIX3 LVLH_Matrix(VECTOR3 R, VECTOR3 V);
	double GETfromMJD(double MJD, double GETBase);
	double MJDfromGET(double GET, double GETBase);
	int Date2JD(int Y, int M, int D);
	double Date2MJD(int Y, int D, int H, int M, double S);

	//Math
	double stumpS(double z);
	double stumpC(double z);
	double power(double b, double e);
	template <typename T> int sign(T val);
	double cot(double a);
	double sec(double a);
	double acos2(double _X);
	double fraction_an(int n);
	double fraction_ad(int n);
	double fraction_a(int n, double x);
	double fraction_b(int n, double x);
	double fraction_delta(int n, double x);
	double fraction_u(int n, double x);
	double fraction_pq(double x);
	double fraction_xi(double x);
	double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle);
	MATRIX3 inverse(MATRIX3 a);
	double determinant(MATRIX3 a);

	class CoastIntegrator
	{
	public:
		CoastIntegrator(VECTOR3 R0, VECTOR3 V0, double mjd0, double dt);
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
		double mjd0;
		double rect1, rect2;
		VECTOR3 U_Z;
	};

}