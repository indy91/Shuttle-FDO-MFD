/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD Orbital Mechanics Calculations

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

namespace OrbMech
{
	double power(double b, double e)
	{
		double res;

		res = 0;
		if (b < 0 && e != ceil(e)) {
			res = -pow(-b, e);
		}
		else {
			res = pow(b, e);
		}
		return res;
	}

	double cot(double a)
	{
		return cos(a) / sin(a);
	}

	double sec(double a)
	{
		return 1.0 / cos(a);
	}

	double acos2(double _X)
	{
		return acos(min(1.0, max(-1.0, _X)));
	}

	double stumpS(double z)
	{
		double s;

		s = 0;
		if (z > 0) {
			s = (sqrt(z) - sin(sqrt(z))) / OrbMech::power(sqrt(z), 3);
		}
		else if (z < 0) {
			s = (sinh(sqrt(-z)) - sqrt(-z)) / OrbMech::power(sqrt(-z), 3);
		}
		else {
			s = 1.0 / 6.0;
		}
		return s;
	}

	double stumpC(double z)
	{
		double c;
		c = 0;
		if (z > 0) {
			c = (1.0 - cos(sqrt(z))) / z;
		}
		else if (z < 0) {
			c = (cosh(sqrt(-z)) - 1.0) / (-z);
		}
		else {
			c = 0.5;
		}
		return c;
	}

	double fraction_an(int n)
	{
		if (n == 0)
		{
			return 4.0;
		}
		else
		{
			return fraction_an(n - 1) + (2.0*n + 1.0);
		}
	}

	double fraction_ad(int n)
	{
		if (n == 0)
		{
			return 15.0;
		}
		else
		{
			return fraction_ad(n - 1) + 4.0*(2.0*n + 1.0);
		}
	}

	double fraction_a(int n, double x)
	{
		if (n == 0)
		{
			return 1.0 / 5.0;
		}
		else
		{
			return -fraction_an(n) / fraction_ad(n)*x;
		}
	}

	double fraction_b(int n, double x)
	{
		return 1.0;
	}

	double fraction_delta(int n, double x)
	{
		if (n == 0)
		{
			return 1.0;
		}
		else
		{
			return 1.0 / (1.0 - fraction_a(n, x)*fraction_delta(n - 1, x));
		}
	}

	double fraction_u(int n, double x)
	{
		if (n == 0)
		{
			return 1.0 / 5.0;
		}
		else
		{
			return fraction_u(n - 1, x)*(fraction_delta(n, x) - 1.0);
		}
	}

	double fraction_pq(double x)
	{
		int n = 0;
		double u = 100.0;
		double pq = 0.0;

		while (abs(u) > 1e-9)
		{
			u = fraction_u(n, x);
			pq += u;
			n++;
		}
		return pq;
	}

	double fraction_xi(double x)
	{
		double eta, xi_eta, xi_x;

		eta = (sqrt(1.0 + x) - 1.0) / (sqrt(1.0 + x) + 1.0);
		xi_eta = fraction_pq(eta);
		xi_x = 1.0 / (8.0*(sqrt(1.0 + x) + 1.0))*(3.0 + xi_eta / (1.0 + eta * xi_eta));
		return xi_x;
	}

	MATRIX3 GetObliquityMatrix(OBJHANDLE plan, double t)
	{
		double t0, T_p, L_0, e_rel, phi_0, T_s, e_ref, L_ref, L_rel, phi, e_ecl, L_ecl;
		MATRIX3 Rot1, Rot2, Rot3, Rot4, Rot5, Rot6, R_ref, R_rel, R_rot, Rot;
		VECTOR3 s;

		if (plan == oapiGetObjectByName("Earth"))
		{
			t0 = 51544.5;								//LAN_MJD, MJD of the LAN in the "beginning"
			T_p = -9413040.4;							//Precession Period
			L_0 = 0.0;									//LAN in the "beginning"
			e_rel = 0.4090928023;						//Obliquity / axial tilt of the Earth in radians
			phi_0 = 4.88948754;							//Sidereal Rotational Offset
			T_s = 86164.10132 / 24.0 / 60.0 / 60.0;	//Sidereal Rotational Period
			e_ref = 0;									//Precession Obliquity
			L_ref = 0;									//Precession LAN
		}
		else if (plan == oapiGetObjectByName("Moon"))
		{
			t0 = 51544.5;							//LAN_MJD, MJD of the LAN in the "beginning"
			T_p = -6793.219721;						//Precession Period
			L_0 = 1.71817749;						//LAN in the "beginning"
			e_rel = 0.02692416821;					//Obliquity / axial tilt of the Moon in radians
			phi_0 = 4.769465382;					//Sidereal Rotational Offset
			T_s = 2360588.15 / 24.0 / 60.0 / 60.0;	//Sidereal Rotational Period
			e_ref = 7.259562816e-005;				//Precession Obliquity
			L_ref = 0.4643456618;					//Precession LAN
		}

		L_rel = L_0 + PI2 * (t - t0) / T_p;
		Rot1 = _M(cos(L_ref), 0.0, -sin(L_ref), 0.0, 1.0, 0.0, sin(L_ref), 0.0, cos(L_ref));
		Rot2 = _M(1.0, 0.0, 0.0, 0.0, cos(e_ref), -sin(e_ref), 0.0, sin(e_ref), cos(e_ref));
		R_ref = mul(Rot1, Rot2);
		Rot3 = _M(cos(L_rel), 0.0, -sin(L_rel), 0.0, 1.0, 0.0, sin(L_rel), 0.0, cos(L_rel));
		Rot4 = _M(1.0, 0.0, 0.0, 0.0, cos(e_rel), -sin(e_rel), 0.0, sin(e_rel), cos(e_rel));
		R_rel = mul(Rot3, Rot4);
		phi = phi_0 + PI2 * (t - t0) / T_s + (L_0 - L_rel)*cos(e_rel);
		R_rot = _M(cos(phi), 0.0, -sin(phi), 0.0, 1.0, 0.0, sin(phi), 0.0, cos(phi));
		Rot = mul(R_ref, mul(R_rel, R_rot));
		s = mul(Rot, _V(0.0, 1.0, 0.0));
		e_ecl = acos(s.y);
		L_ecl = atan(-s.x / s.z);
		Rot5 = _M(cos(L_ecl), 0.0, -sin(L_ecl), 0.0, 1.0, 0.0, sin(L_ecl), 0.0, cos(L_ecl));
		Rot6 = _M(1.0, 0.0, 0.0, 0.0, cos(e_ecl), -sin(e_ecl), 0.0, sin(e_ecl), cos(e_ecl));
		return mul(Rot5, Rot6);
	}

	MATRIX3 GetRotationMatrix(OBJHANDLE plan, double t)
	{
		double t0, T_p, L_0, e_rel, phi_0, T_s, e_ref, L_ref, L_rel, phi;
		MATRIX3 Rot1, Rot2, R_ref, Rot3, Rot4, R_rel, R_rot, R, Rot;

		if (plan == oapiGetObjectByName("Earth"))
		{
			t0 = 51544.5;								//LAN_MJD, MJD of the LAN in the "beginning"
			T_p = -9413040.4;							//Precession Period
			L_0 = 0.0;									//LAN in the "beginning"
			e_rel = 0.4090928023;						//Obliquity / axial tilt of the Earth in radians
			phi_0 = 4.88948754;							//Sidereal Rotational Offset
			T_s = 86164.10132 / 24.0 / 60.0 / 60.0;	//Sidereal Rotational Period
			e_ref = 0;									//Precession Obliquity
			L_ref = 0;									//Precession LAN
		}
		else if (plan == oapiGetObjectByName("Moon"))
		{
			t0 = 51544.5;							//LAN_MJD, MJD of the LAN in the "beginning"
			T_p = -6793.219721;						//Precession Period
			L_0 = 1.71817749;						//LAN in the "beginning"
			e_rel = 0.02692416821;					//Obliquity / axial tilt of the Moon in radians
			phi_0 = 4.769465382;					//Sidereal Rotational Offset
			T_s = 2360588.15 / 24.0 / 60.0 / 60.0;	//Sidereal Rotational Period
			e_ref = 7.259562816e-005;				//Precession Obliquity
			L_ref = 0.4643456618;					//Precession LAN
		}

		Rot1 = _M(cos(L_ref), 0, -sin(L_ref), 0, 1, 0, sin(L_ref), 0, cos(L_ref));
		Rot2 = _M(1, 0, 0, 0, cos(e_ref), -sin(e_ref), 0, sin(e_ref), cos(e_ref));
		R_ref = mul(Rot1, Rot2);
		L_rel = L_0 + PI2 * (t - t0) / T_p;
		Rot3 = _M(cos(L_rel), 0, -sin(L_rel), 0, 1, 0, sin(L_rel), 0, cos(L_rel));
		Rot4 = _M(1, 0, 0, 0, cos(e_rel), -sin(e_rel), 0, sin(e_rel), cos(e_rel));
		R_rel = mul(Rot3, Rot4);
		phi = phi_0 + PI2 * (t - t0) / T_s + (L_0 - L_rel)*cos(e_rel);
		R_rot = _M(cos(phi), 0, -sin(phi), 0, 1, 0, sin(phi), 0, cos(phi));
		Rot = mul(R_rel, R_rot);
		R = mul(R_ref, Rot);
		return R;
	}

	VECTOR3 Polar2Cartesian(double r, double lat, double lng)
	{
		return _V(r*cos(lat)*cos(lng), r*cos(lat)*sin(lng), r*sin(lat));
	}

	VECTOR3 Polar2CartesianVel(double r, double lat, double lng, double r_dot, double lat_dot, double lng_dot)
	{
		return _V(r_dot*cos(lat)*cos(lng) - lat_dot * r*sin(lat)*cos(lng) - r * lng_dot*cos(lat)*sin(lng), r_dot*cos(lat)*sin(lng) - r * lat_dot*sin(lat)*sin(lng) + r * lng_dot*cos(lat)*cos(lng), r_dot*sin(lat) + r * lat_dot*cos(lat));
	}

	template <typename T> int sign(T val) {
		return (T(0) < val) - (val < T(0));
	}

	VECTOR3 rhmul(const MATRIX3 &A, const VECTOR3 &b)	//For the left handed Orbiter matrizes, A is left handed, b is right handed, result is right handed
	{
		return _V(
			A.m11*b.x + A.m12*b.z + A.m13*b.y,
			A.m31*b.x + A.m32*b.z + A.m33*b.y,
			A.m21*b.x + A.m22*b.z + A.m23*b.y);
	}

	VECTOR3 rhtmul(const MATRIX3 &A, const VECTOR3 &b)
	{
		return _V(
			A.m11*b.x + A.m21*b.z + A.m31*b.y,
			A.m13*b.x + A.m23*b.z + A.m33*b.y,
			A.m12*b.x + A.m22*b.z + A.m32*b.y);
	}

	void rv_from_r0v0(VECTOR3 R0, VECTOR3 V0, double t, VECTOR3 &R1, VECTOR3 &V1, double mu, double x)	//computes the state vector (R,V) from the initial state vector (R0,V0) and the elapsed time
	{
		double r0, v0, vr0, alpha, f, g, fdot, gdot, r, xx, paratol, x0;

		//If absolute value of alpha is smaller than paratol, then the parabolic initial guess is used
		paratol = 0.00000001;

		r0 = length(R0);
		v0 = length(V0);
		vr0 = dotp(R0, V0) / r0;
		alpha = 2.0 / r0 - v0 * v0 / mu;

		//Initial guess supplied by the calling function
		if (x != 0.0)
		{
			x0 = x;
		}
		//Calculate initial guess
		else
		{
			//Initial guess for elliptical and hyperbolic orbits (and nonsensical orbits)
			if (abs(alpha) > paratol || v0 == 0.0)
			{
				x0 = sqrt(mu)*abs(alpha)*t;
			}
			//Initial guess for (near) parabolic orbits
			else
			{
				VECTOR3 H = crossp(R0, V0);
				double hmag = length(H);
				double p = hmag * hmag / mu;
				double s = 0.5  * (PI05 - atan(3.0 *sqrt(mu / (p*p*p))* t));
				double w = atan(power(tan(s), 1.0 / 3.0));
				x0 = sqrt(p) * (2.0 *cot(2.0 *w));
			}
		}

		xx = kepler_U(t, r0, vr0, alpha, mu, x0);
		f_and_g(xx, t, r0, alpha, f, g, mu);
		R1 = R0 * f + V0 * g;
		r = length(R1);
		fDot_and_gDot(xx, r, r0, alpha, fdot, gdot, mu);
		V1 = R0 * fdot + V0 * gdot;
	}

	void rv_from_r0v0_obla(VECTOR3 R1, VECTOR3 V1, double MJD, double dt, VECTOR3 &R2, VECTOR3 &V2, OBJHANDLE gravref)
	{
		OELEMENTS coe, coe2;
		MATRIX3 Rot;
		VECTOR3 R1_equ, V1_equ, R2_equ, V2_equ;
		double h, e, Omega_0, i, omega_0, theta0, a, T, n, E_0, t_0, t_f, n_p, t_n, M_n, E_n, theta_n, Omega_dot, omega_dot, Omega_n, omega_n, mu, JCoeff;

		mu = GGRAV * oapiGetMass(gravref);

		if (oapiGetPlanetJCoeffCount(gravref) > 0)
		{
			JCoeff = oapiGetPlanetJCoeff(gravref, 0);
		}

		Rot = GetObliquityMatrix(gravref, MJD);

		R1_equ = rhtmul(Rot, R1);
		V1_equ = rhtmul(Rot, V1);

		coe = coe_from_sv(R1_equ, V1_equ, mu);
		h = coe.h;
		e = coe.e;
		Omega_0 = coe.RA;
		i = coe.i;
		omega_0 = coe.w;
		theta0 = coe.TA;

		a = h * h / mu * 1.0 / (1.0 - e * e);
		T = 2.0 * PI / sqrt(mu)*OrbMech::power(a, 3.0 / 2.0);
		n = 2.0 * PI / T;
		E_0 = 2.0 * atan(sqrt((1.0 - e) / (1.0 + e))*tan(theta0 / 2.0));
		t_0 = (E_0 - e * sin(E_0)) / n;
		t_f = t_0 + dt;
		n_p = t_f / T;
		t_n = (n_p - floor(n_p))*T;
		M_n = n * t_n;
		E_n = kepler_E(e, M_n);
		theta_n = 2.0 * atan(sqrt((1.0 + e) / (1.0 - e))*tan(E_n / 2.0));
		if (theta_n < 0)
		{
			theta_n += 2 * PI;
		}

		Omega_dot = -(3.0 / 2.0 * sqrt(mu)*JCoeff * OrbMech::power(oapiGetSize(gravref), 2.0) / (OrbMech::power(1.0 - OrbMech::power(e, 2.0), 2.0) * OrbMech::power(a, 7.0 / 2.0)))*cos(i);
		omega_dot = -(3.0 / 2.0 * sqrt(mu)*JCoeff * OrbMech::power(oapiGetSize(gravref), 2.0) / (OrbMech::power(1.0 - OrbMech::power(e, 2.0), 2.0) * OrbMech::power(a, 7.0 / 2.0)))*(5.0 / 2.0 * sin(i)*sin(i) - 2.0);

		Omega_n = Omega_0 + Omega_dot * dt;
		omega_n = omega_0 + omega_dot * dt;

		coe2.h = h;
		coe2.e = e;
		coe2.RA = Omega_n;
		coe2.i = i;
		coe2.w = omega_n;
		coe2.TA = theta_n;

		sv_from_coe(coe2, mu, R2_equ, V2_equ);

		R2 = rhmul(Rot, R2_equ);
		V2 = rhmul(Rot, V2_equ);
	}

	double kepler_U(double dt, double ro, double vro, double a, double mu, double x0) //This function uses Newton's method to solve the universal Kepler equation for the universal anomaly.
	{
		double error2, ratio, C, S, F, dFdx, x;
		int n, nMax;

		error2 = 1e-8;
		nMax = 1000;
		n = 0;
		ratio = 1;
		C = 0;
		S = 0;
		F = 0;
		dFdx = 0;
		x = x0;
		while ((abs(ratio) > error2) && (n <= nMax)) {
			n = n + 1;
			C = stumpC(a*x*x);
			S = stumpS(a*x*x);
			F = ro * vro / sqrt(mu)*x*x*C + (1.0 - a * ro)*OrbMech::power(x, 3.0)*S + ro * x - sqrt(mu)*dt;
			dFdx = ro * vro / sqrt(mu)*x*(1.0 - a * x*x*S) + (1.0 - a * ro)*x*x*C + ro;
			ratio = F / dFdx;
			x = x - ratio;
		}
		return x;
	}

	double kepler_E(double e, double M, double error2)
	{
		double ratio, E;
		//{
		//	This function uses Newton's method to solve Kepler's
		//		equation E - e*sin(E) = M for the eccentric anomaly,
		//		given the eccentricity and the mean anomaly.
		//		E - eccentric anomaly(radians)
		//		e - eccentricity, passed from the calling program
		//		M - mean anomaly(radians), passed from the calling program
		//}
		// ----------------------------------------------

		ratio = 1.0;

		if (M < PI)
		{
			E = M + e / 2.0;
		}
		else
		{
			E = M - e / 2.0;
		}

		while (abs(ratio) > error2)
		{
			ratio = (E - e * sin(E) - M) / (1.0 - e * cos(E));
			E = E - ratio;
		}
		return E;
	} //kepler_E

	double MeanToTrueAnomaly(double meanAnom, double eccdp, double error2)
	{
		double ta;

		double E = kepler_E(eccdp, meanAnom, error2);
		if (E < 0.0)
			E = E + PI2;
		double c = abs(E - PI);
		if (c >= 1.0e-8)
		{
			ta = 2.0 * atan(sqrt((1.0 + eccdp) / (1.0 - eccdp))*tan(E / 2.0));
		}
		else
		{
			ta = E;
		}

		if (ta < 0)
			ta += PI2;

		return ta;
	}

	double TrueToMeanAnomaly(double ta, double eccdp)
	{
		double ma = 0.0;
		double E = TrueToEccentricAnomaly(ta, eccdp);
		ma = E - eccdp * sin(E);
		if (ma < 0.0)
			ma = ma + PI2;
		return ma;
	}

	double TrueToEccentricAnomaly(double ta, double ecc)
	{
		double ea;
		double cosTa = cos(ta);
		double eccCosTa = ecc * cosTa;
		double sinEa = (sqrt(1.0 - ecc * ecc)*sin(ta)) / (1.0 + eccCosTa);
		double cosEa = (ecc + cosTa) / (1.0 + eccCosTa);
		ea = atan2(sinEa, cosEa);

		if (ea < 0.0)
			ea = ea + PI2;

		return ea;
	}

	VECTOR3 EclipticToECI(VECTOR3 v, double MJD)
	{
		OBJHANDLE hEarth = oapiGetObjectByName("Earth");
		MATRIX3 Rot = GetObliquityMatrix(hEarth, MJD);
		return rhtmul(Rot, v);
	}

	void EclipticToECI(VECTOR3 R, VECTOR3 V, double MJD, VECTOR3 &R_ECI, VECTOR3 &V_ECI)
	{
		OBJHANDLE hEarth = oapiGetObjectByName("Earth");
		MATRIX3 Rot = GetObliquityMatrix(hEarth, MJD);
		R_ECI = rhtmul(Rot, R);
		V_ECI = rhtmul(Rot, V);
	}

	VECTOR3 ECIToEcliptic(VECTOR3 v, double MJD)
	{
		OBJHANDLE hEarth = oapiGetObjectByName("Earth");
		MATRIX3 Rot = GetObliquityMatrix(hEarth, MJD);
		return rhmul(Rot, v);
	}

	void ECIToEcliptic(VECTOR3 R, VECTOR3 V, double MJD, VECTOR3 &R_ecl, VECTOR3 &V_ecl)
	{
		OBJHANDLE hEarth = oapiGetObjectByName("Earth");
		MATRIX3 Rot = GetObliquityMatrix(hEarth, MJD);
		R_ecl = rhmul(Rot, R);
		V_ecl = rhmul(Rot, V);
	}

	void f_and_g(double x, double t, double ro, double a, double &f, double &g, double mu)	//calculates the Lagrange f and g coefficients
	{
		double z;

		z = a * x*x;
		f = 1 - x * x / ro * stumpC(z);
		g = t - 1 / sqrt(mu)*OrbMech::power(x, 3)*stumpS(z);
	}

	void fDot_and_gDot(double x, double r, double ro, double a, double &fdot, double &gdot, double mu)	//calculates the time derivatives of the Lagrange f and g coefficients
	{
		double z;
		z = a * x*x;
		fdot = sqrt(mu) / r / ro * (z*stumpS(z) - 1.0)*x;
		gdot = 1.0 - x * x / r * stumpC(z);
	}

	double time_theta(VECTOR3 R, VECTOR3 V, double dtheta, double mu, bool future)
	{
		double r, v, alpha, a, f, g, fdot, gdot, sigma0, r1, dt, h, p;

		//double h, v_r, cotg, x, p;

		r = length(R);
		v = length(V);
		alpha = 2.0 / r - v * v / mu;
		a = 1.0 / alpha;
		f_and_g_ta(R, V, dtheta, f, g, mu);
		fDot_and_gDot_ta(R, V, dtheta, fdot, gdot, mu);
		sigma0 = dotp(R, V) / sqrt(mu);

		h = length(crossp(R, V));
		p = h * h / mu;

		r1 = r * p / (r + (p - r)*cos(dtheta) - sqrt(p)*sigma0*sin(dtheta));

		if (alpha > 0)
		{
			double dE, cos_dE, sin_dE;

			cos_dE = 1.0 - r / a * (1.0 - f);
			sin_dE = -r * r1*fdot / sqrt(mu*a);
			dE = atan2(sin_dE, cos_dE);

			dt = g + sqrt(power(a, 3.0) / mu)*(dE - sin_dE);

			if (future && dt < 0)
			{
				double T_P = period(R, V, mu);
				dt += T_P;
			}
		}
		else if (alpha == 0.0)
		{
			double c, s;

			c = sqrt(r*r + r1 * r1 - 2 * r*r1*cos(dtheta));
			s = (r + r1 + c) / 2.0;

			dt = 2.0 / 3.0*sqrt(power(s, 3.0) / 2.0 / mu)*(1.0 - power((s - c) / s, 3.0 / 2.0));
		}
		else
		{
			double dH;

			dH = acosh(1.0 - r / a * (1.0 - f));

			dt = g + sqrt(power(-a, 3.0) / mu)*(sinh(dH) - dH);
		}

		return dt;
	}

	void f_and_g_ta(VECTOR3 R0, VECTOR3 V0, double dt, double &f, double &g, double mu)
	{
		double h, vr0, r0, s, c, r;

		h = length(crossp(R0, V0));
		vr0 = dotp(V0, R0) / length(R0);
		r0 = length(R0);
		s = sin(dt);
		c = cos(dt);
		r = h * h / mu / (1 + (h*h / mu / r0 - 1)*c - h * vr0*s / mu);
		f = 1 - mu * r*(1 - c) / (h*h);
		g = r * r0*s / h;
	}

	void fDot_and_gDot_ta(VECTOR3 R0, VECTOR3 V0, double dt, double &fdot, double &gdot, double mu)
	{
		double h, vr0, r0, c, s;
		h = length(crossp(R0, V0));
		vr0 = dotp(V0, R0) / length(R0);
		r0 = length(R0);
		c = cos(dt);
		s = sin(dt);
		fdot = mu / h * (vr0 / h * (1 - c) - s / r0);
		gdot = 1 - mu * r0 / (h*h) * (1 - c);
	}

	double period(VECTOR3 R, VECTOR3 V, double mu)
	{
		double a, epsilon;

		epsilon = power(length(V), 2.0) / 2.0 - mu / length(R);
		a = -mu / (2.0*epsilon);
		return PI2 * sqrt(power(a, 3.0) / mu);
	}

	void oneclickcoast(VECTOR3 R0, VECTOR3 V0, double mjd0, double dt, VECTOR3 &R1, VECTOR3 &V1)
	{
		OBJHANDLE gravout = NULL;
		bool stop;
		CoastIntegrator* coast;
		coast = new CoastIntegrator(R0, V0, mjd0, dt);
		stop = false;
		while (stop == false)
		{
			stop = coast->iteration();
		}
		R1 = coast->R2;
		V1 = coast->V2;
		delete coast;
	}

	VECTOR3 HeightManeuver(VECTOR3 R, VECTOR3 V, double dh, double mu)
	{
		VECTOR3 R3, V3, am, V_HF;
		double dt3, r_D, dv_H, e_H, eps, p_H, c_I, e_Ho, dv_Ho;
		int s_F;

		OrbMech::REVUP(R, V, 0.5, mu, R3, V3, dt3);
		r_D = length(R3) + dh;
		am = unit(crossp(R, V));
		dv_H = 0.0;
		eps = 1.0;
		p_H = c_I = 0.0;
		s_F = 0;

		do
		{
			V_HF = V + unit(crossp(am, R))*dv_H;
			OrbMech::REVUP(R, V_HF, 0.5, mu, R3, V3, dt3);

			e_H = length(R3) - r_D;

			if (p_H == 0 || abs(e_H) >= eps)
			{
				OrbMech::ITER(c_I, s_F, e_H, p_H, dv_H, e_Ho, dv_Ho);
				if (s_F == 1)
				{
					return _V(0, 0, 0);
				}
			}
		} while (abs(e_H) >= eps);

		return V_HF - V;
	}

	VECTOR3 elegant_lambert(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double dt, int N, bool prog, double mu)
	{
		double tol, ratio, r1, r2, c, s, theta, lambda, T, l, m, x, h1, h2, B, y, z, x_new, A, root;
		int nMax, n;
		VECTOR3 c12, Vt1, Vt2;

		tol = 1e-8;
		nMax = 1000;
		n = 0;
		ratio = 1;

		r1 = length(R1);
		r2 = length(R2);
		c = length(R2 - R1);
		s = (r1 + r2 + c) / 2;

		c12 = crossp(unit(R1), unit(R2));

		theta = acos(dotp(R1, R2) / r1 / r2);
		if ((prog == true && c12.z < 0) || (prog == false && c12.z >= 0))
		{
			theta = PI2 - theta;
		}

		lambda = sqrt(r1*r2) / s * cos(theta / 2.0);
		T = sqrt(8.0 * mu / OrbMech::power(s, 3.0))*dt;
		l = OrbMech::power((1.0 - lambda) / (1.0 + lambda), 2.0);
		m = OrbMech::power(T, 2.0) / (OrbMech::power(1.0 + lambda, 6.0));

		//T_m = pi*((2 * N + 1) - lambda ^ 5);

		//Low Energy
		x = 1.0 + 4.0 * l;

		while (abs(ratio) > tol && nMax >= n)
		{
			n = n + 1;
			if (N == 0)
			{
				double xi = fraction_xi(x);
				h1 = OrbMech::power(l + x, 2.0)*(1.0 + xi * (1.0 + 3.0*x)) / ((1.0 + 2.0*x + l)*(3.0 + x * (1.0 + 4.0*xi)));
				h2 = m * (1.0 + (x - l)*xi) / ((1.0 + 2.0*x + l)*(3.0 + x * (1.0 + 4.0*xi)));
			}
			else
			{
				h1 = OrbMech::power(l + x, 2.0) / (4.0 * OrbMech::power(x, 2.0) * (1.0 + 2.0 * x + l))*(3.0 * OrbMech::power(1.0 + x, 2.0) * (N*PI / 2.0 + atan(sqrt(x))) / sqrt(x) - (3.0 + 5.0 * x));
				h2 = m / (4.0 * OrbMech::power(x, 2.0) * (1.0 + 2.0 * x + l))*((OrbMech::power(x, 2.0) - (1.0 + l)*x - 3.0 * l)*(N*PI / 2.0 + atan(sqrt(x))) / sqrt(x) + (3.0 * l + x));
			}
			B = 27 * h2 / (4 * OrbMech::power(1 + h1, 3));
			if (B >= 0)
			{
				z = 2.0 * cosh(1.0 / 3.0 * acosh(sqrt(B + 1.0)));
			}
			else
			{
				z = 2.0 * cos(1.0 / 3.0 * acos(sqrt(B + 1.0)));
			}
			y = 2.0 / 3.0 * (1.0 + h1)*(sqrt(B + 1.0) / z + 1.0);
			x_new = sqrt(OrbMech::power((1.0 - l) / 2.0, 2) + m / y / y) - (1.0 + l) / 2.0;
			ratio = x - x_new;
			x = x_new;
		}

		Vt1 = ((R2 - R1) + R1 * s*OrbMech::power(1.0 + lambda, 2.0) * (l + x) / (r1*(1.0 + x))) * 1.0 / (lambda*(1 + lambda))*sqrt(mu*(1.0 + x) / (2.0 * OrbMech::power(s, 3.0) * (l + x)));
		if (N == 0)
		{
			return Vt1;
		}

		//High Energy
		root = OrbMech::power((OrbMech::power((2.0 * m) / (OrbMech::power(N, 2.0)*OrbMech::power(PI, 2.0)), 1.0 / 3.0) - (1.0 + l) / 2.0), 2.0) - 4.0 * l;
		if (root >= 0)
		{
			x = (OrbMech::power(2.0 * m / (OrbMech::power(N, 2.0)*OrbMech::power(PI, 2.0)), 1.0 / 3.0) - (1.0 + l) / 2.0) - sqrt(root);
		}
		else
		{
			x = 0.0001;
		}

		ratio = 1;
		n = 0;
		while (abs(ratio) > tol && nMax >= n)
		{
			n = n + 1;
			h1 = (l + x)*(1.0 + 2.0 * x + l) / (2.0 * (l - OrbMech::power(x, 2.0)));
			h2 = m * sqrt(x) / (2.0 * (l - OrbMech::power(x, 2)))*((l - OrbMech::power(x, 2.0))*(N*PI / 2.0 + atan(sqrt(x))) / sqrt(x) - (l + x));
			B = 27.0 * h2 / (4.0 * OrbMech::power(sqrt(x)*(1 + h1), 3));
			if (B >= 0)
			{
				z = 2.0 * cosh(1.0 / 3.0 * acosh(sqrt(B + 1.0)));
			}
			else
			{
				z = 2.0 * cos(1.0 / 3.0 * acos(sqrt(B + 1.0)));
			}
			A = OrbMech::power((sqrt(B) + sqrt(B + 1.0)), 1.0 / 3.0);
			y = 2.0 / 3.0 * sqrt(x)*(1.0 + h1)*(sqrt(B + 1.0) / (A + 1.0 / A) + 1.0);
			//y = 2 / 3 * (1 + h1)*(sqrt(B + 1) / z + 1);
			x_new = 0.5*((m / y / y - (1.0 + l)) - sqrt(OrbMech::power(m / y / y - (1.0 + l), 2) - 4.0 * l));

			ratio = x - x_new;
			x = x_new;
		}

		Vt2 = ((R2 - R1) + R1 * s*OrbMech::power(1.0 + lambda, 2.0) * (l + x) / (r1*(1.0 + x))) * 1.0 / (lambda*(1.0 + lambda))*sqrt(mu*(1.0 + x) / (2.0 * OrbMech::power(s, 3.0) * (l + x)));

		if (length(V1 - Vt1) > length(V1 - Vt2))
		{
			return Vt2;
		}
		else
		{
			return Vt1;
		}
	}

	VECTOR3 Vinti(VECTOR3 R1, VECTOR3 V1, VECTOR3 R2, double mjd0, double dt, int N, bool prog, VECTOR3 V_guess, double tol)
	{
		double h, rho, error3, mu, max_dr;
		int nMax, nMax2, n;
		VECTOR3 Vt1, V1_star, dr2, R2_star, V2_star, R1_ref, V1_ref, R2_ref;
		VECTOR3 v_l[3][4];
		VECTOR3 R2l[3][4];
		VECTOR3 V2l[3][4];
		VECTOR3 T[3];
		MATRIX3 T2;
		OBJHANDLE hEarth;

		hEarth = oapiGetObjectByName("Earth");

		h = 10e-3;
		rho = 0.5;
		error3 = 100.0;
		dr2 = _V(1.0, 1.0, 1.0);
		n = 0;
		nMax = 100;
		nMax2 = 10;

		mu = GGRAV * oapiGetMass(hEarth);

		double hvec[4] = { h / 2, -h / 2, rho*h / 2, -rho * h / 2 };

		if (length(V_guess) == 0.0)
		{
			R1_ref = R1;
			V1_ref = V1;
			R2_ref = R2;

			if (dt > 0)
			{
				Vt1 = elegant_lambert(R1_ref, V1_ref, R2_ref, dt, N, prog, mu);
			}
			else
			{
				Vt1 = elegant_lambert(R1_ref, V1_ref, R2_ref, -dt, N, !prog, mu);
			}

			V1_star = Vt1 * sign(dt);
		}
		else
		{
			V1_star = V_guess;
		}

		if (dt > 0)
		{
			rv_from_r0v0_obla(R1, V1_star, mjd0, dt, R2_star, V2_star, hEarth);
			dr2 = R2 - R2_star;

			while (length(dr2) > error3 && nMax2 >= n)
			{
				n += 1;
				for (int i = 0; i < 4; i++)
				{
					v_l[0][i] = V1_star + _V(1, 0, 0)*hvec[i];
					v_l[1][i] = V1_star + _V(0, 1, 0)*hvec[i];
					v_l[2][i] = V1_star + _V(0, 0, 1)*hvec[i];
				}
				for (int i = 0; i < 3; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						rv_from_r0v0_obla(R1, v_l[i][j], mjd0, dt, R2l[i][j], V2l[i][j], hEarth);
					}
				}
				for (int i = 0; i < 3; i++)
				{
					T[i] = (R2l[i][2] - R2l[i][3] - (R2l[i][0] - R2l[i][1])*OrbMech::power(rho, 3.0)) * 1.0 / (rho*h*(1.0 - OrbMech::power(rho, 2.0)));
				}
				T2 = _M(T[0].x, T[1].x, T[2].x, T[0].y, T[1].y, T[2].y, T[0].z, T[1].z, T[2].z);
				V1_star = V1_star + mul(inverse(T2), dr2);
				rv_from_r0v0_obla(R1, V1_star, mjd0, dt, R2_star, V2_star, hEarth);
				dr2 = R2 - R2_star;
			}
			//return V1_star;

			if (n == nMax2 || _isnan(R2_star.x))// || isinf(R2_star.x))
			{
				return _V(0, 0, 0);
			}

			dr2 = _V(1.0, 1.0, 1.0);
			n = 0;
		}

		oneclickcoast(R1, V1_star, mjd0, dt, R2_star, V2_star);
		dr2 = R2 - R2_star;
		max_dr = 0.5*length(R2_star);
		if (length(dr2) > max_dr)
		{
			dr2 = unit(dr2)*max_dr;
		}

		while (length(dr2) > tol && nMax >= n)
		{
			n += 1;
			for (int i = 0; i < 4; i++)
			{
				v_l[0][i] = V1_star + _V(1, 0, 0)*hvec[i];
				v_l[1][i] = V1_star + _V(0, 1, 0)*hvec[i];
				v_l[2][i] = V1_star + _V(0, 0, 1)*hvec[i];
			}
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 4; j++)
				{
					oneclickcoast(R1, v_l[i][j], mjd0, dt, R2l[i][j], V2l[i][j]);
				}
			}
			for (int i = 0; i < 3; i++)
			{
				T[i] = (R2l[i][2] - R2l[i][3] - (R2l[i][0] - R2l[i][1])*OrbMech::power(rho, 3.0)) * 1.0 / (rho*h*(1.0 - OrbMech::power(rho, 2.0)));
			}
			T2 = _M(T[0].x, T[1].x, T[2].x, T[0].y, T[1].y, T[2].y, T[0].z, T[1].z, T[2].z);
			V1_star = V1_star + mul(inverse(T2), dr2);
			oneclickcoast(R1, V1_star, mjd0, dt, R2_star, V2_star);
			dr2 = R2 - R2_star;
			max_dr = 0.5*length(R2_star);
			if (length(dr2) > max_dr)
			{
				dr2 = unit(dr2)*max_dr;
			}
		}
		return V1_star;
	}

	void periapo(VECTOR3 R, VECTOR3 V, double mu, double &apo, double &peri)
	{
		double a, e, epsilon;
		VECTOR3 Ex, H;

		H = crossp(R, V);

		Ex = crossp(V, H) / mu - R / length(R);
		e = length(Ex);
		epsilon = 0.5*OrbMech::power(length(V), 2.0) - mu / length(R);
		a = -0.5*mu / epsilon;
		peri = (-e + 1.0)*a;
		apo = (e + 1.0)*a;
		if (apo < 0)
		{
			apo = DBL_MAX;
		}
	}

	void orbitmidnight(VECTOR3 R, VECTOR3 V, VECTOR3 sun, OBJHANDLE planet, bool night, double &v1)
	{
		double tol, swit, mu, R_E, g1, g2, beta1, beta2, aa, SS, p;
		OELEMENTS coe;
		VECTOR3 P, Q, h, h_proj, r_proj;

		if (night)
		{
			swit = -1.0;
		}
		else
		{
			swit = 1.0;
		}
		tol = 1e-6;
		mu = GGRAV * oapiGetMass(planet);
		R_E = oapiGetSize(planet);

		coe = coe_from_sv(R, V, mu);

		P = _V(cos(coe.w)*cos(coe.RA) - sin(coe.w)*sin(coe.RA)*cos(coe.i), cos(coe.w)*sin(coe.RA) + sin(coe.w)*cos(coe.RA)*cos(coe.i), sin(coe.w)*sin(coe.i));
		Q = _V(-sin(coe.w)*cos(coe.RA) - cos(coe.w)*sin(coe.RA)*cos(coe.i), -sin(coe.w)*sin(coe.RA) + cos(coe.w)*cos(coe.RA)*cos(coe.i), cos(coe.w)*sin(coe.i));

		beta1 = dotp(sun, P) / length(sun);

		aa = coe.h*coe.h / (mu*(1 - coe.e*coe.e));

		//if (beta1*beta1 > 1.0 - pow(R_E / (aa*(1.0 - coe.e)), 2) && beta1*beta1 < 1.0 - pow(R_E / (aa*(1.0 + coe.e)), 2))
		//{
		//	v1 = 0;
		//	return;
		//}
		beta2 = dotp(sun, Q) / length(sun);
		p = coe.h*coe.h / mu;

		h = unit(crossp(R, V));
		h_proj = unit(crossp(unit(sun), h));
		r_proj = unit(crossp(h, h_proj));
		g1 = dotp(r_proj, P);
		g2 = dotp(r_proj, Q);

		v1 = 2.0*atan(g2 / (g1 + swit * 1.0));

		SS = cos(v1)*g1 + sin(v1)*g2;

		if (abs(SS - swit) > tol)
		{
			v1 += PI;
		}
		if (v1 > PI)
		{
			v1 -= PI2;
		}
		else if (v1 < -PI)
		{
			v1 += PI2;
		}
		return;
	}

	void umbra(VECTOR3 R, VECTOR3 V, VECTOR3 sun, OBJHANDLE planet, bool rise, double &v1)
	{
		OELEMENTS coe;
		VECTOR3 P, Q;
		double R_E, beta1, beta2, a, b, c, d, e, p, q, D0, D1, S, DD, SS[2], sinx[2], pp, alpha, cond, aa, mu;
		double x[4];
		double cosv[2], sinv[2];
		int j, l;

		mu = GGRAV * oapiGetMass(planet);
		R_E = oapiGetSize(planet);

		coe = coe_from_sv(R, V, mu);

		P = _V(cos(coe.w)*cos(coe.RA) - sin(coe.w)*sin(coe.RA)*cos(coe.i), cos(coe.w)*sin(coe.RA) + sin(coe.w)*cos(coe.RA)*cos(coe.i), sin(coe.w)*sin(coe.i));
		P = unit(P);
		Q = _V(-sin(coe.w)*cos(coe.RA) - cos(coe.w)*sin(coe.RA)*cos(coe.i), -sin(coe.w)*sin(coe.RA) + cos(coe.w)*cos(coe.RA)*cos(coe.i), cos(coe.w)*sin(coe.i));
		Q = unit(Q);

		beta1 = dotp(unit(sun), P);

		aa = coe.h*coe.h / (mu*(1.0 - coe.e*coe.e));
		p = aa * (1.0 - coe.e*coe.e);

		if (beta1*beta1 > 1.0 - pow(R_E / (aa*(1.0 - coe.e)), 2) && beta1*beta1 < 1.0 - pow(R_E / (aa*(1.0 + coe.e)), 2))
		{
			v1 = 0;
			return;
		}

		beta2 = dotp(unit(sun), Q);
		p = coe.h*coe.h / mu;
		/*A = coe.e*coe.e*R_E*R_E + p*p*beta1*beta1 - p*p*beta2*beta2;
		B = 2.0 * coe.e*R_E;
		C = 2.0*beta1*beta2*p*p;
		D = R_E*R_E + p*p*beta2*beta2 - p*p;*/

		alpha = R_E / p;

		a = pow(alpha, 4)*pow(coe.e, 4) - 2.0*pow(alpha, 2)*(beta2*beta2 - beta1 * beta1)*coe.e*coe.e + pow(beta1*beta1 + beta2 * beta2, 2);
		b = 4.0*pow(alpha, 4)*pow(coe.e, 3) - 4.0*pow(alpha, 2)*(beta2*beta2 - beta1 * beta1)*coe.e;
		c = 6.0*pow(alpha, 4)*coe.e*coe.e - 2.0*pow(alpha, 2)*(beta2*beta2 - beta1 * beta1) - 2.0*pow(alpha, 2)*(1.0 - beta2 * beta2)*coe.e*coe.e + 2.0*(beta2*beta2 - beta1 * beta1)*(1.0 - beta2 * beta2) - 4.0*beta1*beta1*beta2*beta2;
		d = 4.0*pow(alpha, 4)*coe.e - 4.0*pow(alpha, 2)*(1.0 - beta2 * beta2)*coe.e;
		e = pow(alpha, 4) - 2.0*pow(alpha, 2)*(1.0 - beta2 * beta2) + pow(1.0 - beta2 * beta2, 2);

		pp = (8.0*a*c - 3.0 * b*b) / (8.0*a*a);
		q = (b*b*b - 4.0*a*b*c + 8.0*a*a*d) / (8.0 * a*a*a);
		D0 = c * c - 3.0*b*d + 12.0 * a*e;
		D1 = 2.0*c*c*c - 9.0*b*c*d + 27.0*b*b*e + 27.0*a*d*d - 72.0*a*c*e;
		DD = -(D1*D1 - 4.0*D0*D0*D0) / 27.0;

		if (DD > 0)
		{
			double phi;

			phi = acos(D1 / (2.0*sqrt(D0*D0*D0)));
			S = 0.5*sqrt(-2.0 / 3.0*pp + 2.0 / 3.0 / a * sqrt(D0)*cos(phi / 3.0));
		}
		else
		{
			double QQ;

			QQ = OrbMech::power((D1 + sqrt(D1*D1 - 4.0*D0*D0*D0)) / 2.0, 1.0 / 3.0);
			S = 0.5*sqrt(-2.0 / 3.0*pp + 1.0 / (3.0*a)*(QQ + D0 / QQ));
		}
		x[0] = -b / (4.0*a) - S + 0.5*sqrt(-4.0*S*S - 2.0 * pp + q / S);
		x[1] = -b / (4.0*a) - S - 0.5*sqrt(-4.0*S*S - 2.0 * pp + q / S);
		x[2] = -b / (4.0*a) + S + 0.5*sqrt(-4.0*S*S - 2.0 * pp - q / S);
		x[3] = -b / (4.0*a) + S - 0.5*sqrt(-4.0*S*S - 2.0 * pp - q / S);

		j = 0;

		//Select the two physicals solutions from the (up to) four real roots of the quartic
		for (int i = 0; i < 4; i++)
		{
			sinx[0] = sqrt(1.0 - x[i] * x[i]);
			sinx[1] = -sinx[0];

			for (int k = 0;k < 2;k++)
			{
				SS[k] = R_E * R_E*pow(1.0 + coe.e*x[i], 2) + p * p*pow(beta1*x[i] + beta2 * sinx[k], 2) - p * p;
			}
			if (abs(SS[0]) < abs(SS[1]))
			{
				l = 0;
			}
			else
			{
				l = 1;
			}
			cond = beta1 * x[i] + beta2 * sinx[l];
			if (cond < 0)
			{
				cosv[j] = x[i];
				sinv[j] = sinx[l];
				j++;
			}

			/*SS = R_E*R_E*pow(1.0 + coe.e*x[i], 2) + p*p*pow(beta1*x[i] + beta2*sinx, 2) - p*p;
			if (abs(SS) < 1.0)
			{
				cond = beta1*x[i] + beta2*sinx;
				if (cond < 0)
				{
					cosv[j] = x[i];
					sinv[j] = sinx;
					j++;
				}
			}
			sinx = -sinx;
			SS = R_E*R_E*pow(1.0 + coe.e*x[i], 2) + p*p*pow(beta1*x[i] + beta2*sinx, 2) - p*p;
			if (abs(SS) < 1.0)
			{
				cond = beta1*x[i] + beta2*sinx;
				if (cond < 0)
				{
					cosv[j] = x[i];
					sinv[j] = sinx;
					j++;
				}
			}*/
		}

		//If it didn't find 2 physical solutions, abort
		if (j != 2)
		{
			v1 = 0.0;
			return;
		}

		//Choose entry vs. exit
		double dSS0 = 2.0*p*p*(beta2*cosv[0] - beta1 * sinv[0])*(beta1*cosv[0] + beta2 * sinv[0]) - 2.0*R_E*R_E*coe.e*sinv[0] * (coe.e*cosv[0] + 1.0);

		if (rise)
		{
			if (dSS0 < 0)
			{
				v1 = atan2(sinv[0], cosv[0]);
			}
			else
			{
				v1 = atan2(sinv[1], cosv[1]);
			}
		}
		else
		{
			if (dSS0 > 0)
			{
				v1 = atan2(sinv[0], cosv[0]);
			}
			else
			{
				v1 = atan2(sinv[1], cosv[1]);
			}
		}
	}


	double sunrise(VECTOR3 R, VECTOR3 V, double MJD, OBJHANDLE planet, OBJHANDLE planet2, bool rise, bool midnight, bool future)
	{
		//midnight = 0-> rise=0:sunset, rise=1:sunrise
		//midnight = 1-> rise=0:midday, rise=1:midnight
		double PlanPos[12];
		VECTOR3 PlanVec, R_EM, R_SE;
		OBJHANDLE hEarth, hMoon, hSun;
		double mu, v1;
		unsigned char options;

		mu = GGRAV * oapiGetMass(planet);

		hEarth = oapiGetObjectByName("Earth");
		hMoon = oapiGetObjectByName("Moon");
		hSun = oapiGetObjectByName("Sun");

		CELBODY *cPlan = oapiGetCelbodyInterface(planet);

		OELEMENTS coe;
		double h, e, theta0, a, dt, dt_alt;

		dt = 0;
		dt_alt = 1;

		while (abs(dt_alt - dt) > 0.5)
		{
			if (planet == hMoon && planet2 == hSun)
			{
				CELBODY *cEarth = oapiGetCelbodyInterface(hEarth);
				options = cPlan->clbkEphemeris(MJD + dt / 24.0 / 3600.0, EPHEM_TRUEPOS, PlanPos);
				if (options & EPHEM_POLAR)
				{
					R_EM = Polar2Cartesian(PlanPos[2] * AU, PlanPos[1], PlanPos[0]);
				}
				else
				{
					R_EM = _V(PlanPos[0], PlanPos[2], PlanPos[1]);
				}
				options = cEarth->clbkEphemeris(MJD + dt / 24.0 / 3600.0, EPHEM_TRUEPOS, PlanPos);
				if (options & EPHEM_POLAR)
				{
					R_SE = Polar2Cartesian(PlanPos[2] * AU, PlanPos[1], PlanPos[0]);
				}
				else
				{
					R_SE = _V(PlanPos[0], PlanPos[2], PlanPos[1]);
				}
				PlanVec = -(R_EM + R_SE);
			}
			else
			{
				options = cPlan->clbkEphemeris(MJD + dt / 24.0 / 3600.0, EPHEM_TRUEPOS, PlanPos);

				if (options & EPHEM_POLAR)
				{
					PlanVec = -Polar2Cartesian(PlanPos[2] * AU, PlanPos[1], PlanPos[0]);
				}
				else
				{
					PlanVec = -_V(PlanPos[0], PlanPos[2], PlanPos[1]);
				}
			}

			if (midnight)
			{
				orbitmidnight(R, V, PlanVec, planet, rise, v1);
			}
			else
			{
				umbra(R, V, PlanVec, planet, rise, v1);
			}

			coe = coe_from_sv(R, V, mu);
			h = coe.h;
			e = coe.e;
			theta0 = coe.TA;

			if (e > 1.0)
			{
				VECTOR3 R1, V1;
				double ddt;

				rv_from_r0v0(R, V, dt, R1, V1, mu);

				coe = coe_from_sv(R1, V1, mu);
				h = coe.h;
				e = coe.e;
				theta0 = coe.TA;

				dt_alt = dt;
				ddt = time_theta(R1, V1, calculateDifferenceBetweenAngles(theta0, v1), mu);
				dt += ddt;
			}
			else
			{
				double T;

				a = h * h / mu * 1.0 / (1.0 - e * e);
				T = PI2 / sqrt(mu)*OrbMech::power(a, 3.0 / 2.0);

				dt_alt = dt;
				dt = time_theta(R, V, calculateDifferenceBetweenAngles(theta0, v1), mu);

				if (dt < 0 && future)
				{
					dt += T;
				}
			}
		}

		return dt;
	}

	void poweredflight(VECTOR3 R, VECTOR3 V, double mjd0, double f_T, double v_ex, double m, VECTOR3 V_G, bool nonspherical, VECTOR3 &R_cutoff, VECTOR3 &V_cutoff, double &m_cutoff, double &t_go)
	{
		double dt, dt_max, a_T, tau, m0, mnow, dV, dVnow, t_remain, t;
		VECTOR3 U_TD, gp, g, R0, V0, Rnow, Vnow, dvdt;

		dV = length(V_G);
		U_TD = unit(V_G);
		R0 = R;
		V0 = V;
		m0 = m;
		t = 0.0;
		Rnow = R0;
		Vnow = V0;
		mnow = m;
		dVnow = dV;

		dt_max = 0.1;
		dt = 1.0;

		gp = gravityroutine(R0, mjd0, nonspherical);

		while (dt != 0.0)
		{
			a_T = f_T / mnow;
			tau = v_ex / a_T;
			t_remain = tau * (1.0 - exp(-dVnow / v_ex));
			dt = min(dt_max, t_remain);
			dvdt = U_TD * f_T / mnow * dt;

			Rnow = Rnow + (Vnow + gp * dt*0.5 + dvdt * 0.5)*dt;
			g = gravityroutine(Rnow, mjd0, nonspherical);
			Vnow = Vnow + (g + gp)*dt*0.5 + dvdt;
			gp = g;
			dVnow -= length(dvdt);
			mnow -= f_T / v_ex * dt;
			t += dt;
		}
		R_cutoff = Rnow;
		V_cutoff = Vnow;
		m_cutoff = mnow;
		t_go = t;
	}

	VECTOR3 gravityroutine(VECTOR3 R, double mjd0, bool nonspherical)
	{
		OBJHANDLE hEarth;
		VECTOR3 U_R, U_Z, g;
		double rr, mu;

		hEarth = oapiGetObjectByName("Earth");
		U_R = unit(R);
		MATRIX3 obli_E = OrbMech::GetObliquityMatrix(hEarth, mjd0);
		U_Z = mul(obli_E, _V(0, 1, 0));
		U_Z = _V(U_Z.x, U_Z.z, U_Z.y);

		rr = dotp(R, R);
		mu = GGRAV * oapiGetMass(hEarth);

		if (nonspherical)
		{
			double costheta, R_E, J2E;
			VECTOR3 g_b;

			costheta = dotp(U_R, U_Z);
			R_E = oapiGetSize(hEarth);
			J2E = oapiGetPlanetJCoeff(hEarth, 0);
			g_b = -(U_R*(1.0 - 5.0*costheta*costheta) + U_Z * 2.0*costheta)*mu / rr * 3.0 / 2.0*J2E*power(R_E, 2.0) / rr;
			g = -U_R * mu / rr + g_b;
		}
		else
		{
			g = -U_R * mu / rr;
		}

		return g;
	}

	void REL_COMP(VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3 &R_S_INER, VECTOR3 &R_REL)
	{
		VECTOR3 V_S_INER, V_REL;
		V_REL = _V(0, 0, 0);
		REL_COMP(false, R_T_INER, V_T_INER, R_S_INER, V_S_INER, R_REL, V_REL);
	}

	void REL_COMP(bool INER_TO_LVC, VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3 &R_S_INER, VECTOR3 &V_S_INER, VECTOR3 &R_REL, VECTOR3 &V_REL)
	{
		MATRIX3 MAT_M50_LVIR;
		VECTOR3 V1, V2, V3, OMEGA_LV_PROX, RTS_M50, VTS_M50, RTS_LVIR, VTS_LVIR, RTS_LV, VTS_LV, VTAN;
		double RT_MAG, ZCON, THETA, THETA_DOT, OMEGA_PROX;

		RT_MAG = length(R_T_INER);
		V1 = unit(crossp(crossp(R_T_INER, V_T_INER), R_T_INER));
		V2 = -unit(crossp(R_T_INER, V_T_INER));
		V3 = -unit(R_T_INER);
		MAT_M50_LVIR = _M(V1.x, V1.y, V1.z, V2.x, V2.y, V2.z, V3.x, V3.y, V3.z);
		VTAN = V_T_INER - unit(R_T_INER)*dotp(R_T_INER, V_T_INER) / RT_MAG;
		OMEGA_PROX = length(VTAN) / RT_MAG;
		OMEGA_LV_PROX = _V(0, -1, 0)*OMEGA_PROX;
		if (INER_TO_LVC)
		{
			RTS_M50 = R_S_INER - R_T_INER;
			VTS_M50 = V_S_INER - V_T_INER;
			RTS_LVIR = mul(MAT_M50_LVIR, RTS_M50);
			VTS_LVIR = mul(MAT_M50_LVIR, VTS_M50);
			RTS_LV = RTS_LVIR;
			VTS_LV = VTS_LVIR - crossp(OMEGA_LV_PROX, RTS_LVIR);
			ZCON = RT_MAG - RTS_LV.z;
			THETA = atan2(RTS_LV.x, ZCON);
			R_REL = _V(RT_MAG*THETA, RTS_LV.y, RT_MAG - ZCON / cos(THETA));
			THETA_DOT = pow(cos(THETA), 2)*(VTS_LV.x*ZCON + RTS_LV.x*VTS_LV.z) / (pow(ZCON, 2));
			V_REL = _V(RT_MAG*THETA_DOT, VTS_LV.y, (VTS_LV.z - ZCON * THETA_DOT*tan(THETA)) / cos(THETA));
		}
		else
		{
			THETA = R_REL.x / RT_MAG;
			THETA_DOT = V_REL.x / RT_MAG;
			ZCON = RT_MAG - R_REL.z;
			RTS_LV = _V(ZCON*sin(THETA), R_REL.y, RT_MAG - ZCON * cos(THETA));
			VTS_LV = _V(ZCON*THETA_DOT*cos(THETA) - V_REL.z*sin(THETA), V_REL.y, RTS_LV.x*THETA_DOT + V_REL.z*cos(THETA));
			RTS_LVIR = RTS_LV;
			VTS_LVIR = VTS_LV + crossp(OMEGA_LV_PROX, RTS_LVIR);
			RTS_M50 = tmul(MAT_M50_LVIR, RTS_LVIR);
			VTS_M50 = tmul(MAT_M50_LVIR, VTS_LVIR);
			R_S_INER = R_T_INER + RTS_M50;
			V_S_INER = V_T_INER + VTS_M50;
		}
	}

	void LVC_to_M50(VECTOR3 R_T_INER, VECTOR3 V_T_INER, VECTOR3 R_REL, VECTOR3 V_REL, VECTOR3& R_S_INER, VECTOR3 &V_S_INER)
	{
		MATRIX3 MAT_M50_LVIR;
		VECTOR3 V1, V2, V3, VTAN, OMEGA_LV_PROX, RTS_LV, VTS_LV, RTS_LVIR, VTS_LVIR, RTS_M50, VTS_M50;
		double RT_MAG, OMEGA_PROX, THETA, THETA_DOT, ZCON;

		RT_MAG = length(R_T_INER);
		V1 = unit(crossp(crossp(R_T_INER, V_T_INER), R_T_INER));
		V2 = -unit(crossp(R_T_INER, V_T_INER));
		V3 = -unit(R_T_INER);
		MAT_M50_LVIR = _M(V1.x, V1.y, V1.z, V2.x, V2.y, V2.z, V3.x, V3.y, V3.z);
		VTAN = V_T_INER - unit(R_T_INER)*dotp(R_T_INER, V_T_INER) / RT_MAG;
		OMEGA_PROX = length(VTAN) / RT_MAG;
		OMEGA_LV_PROX = _V(0, -1, 0)*OMEGA_PROX;
		THETA = R_REL.x / RT_MAG;
		THETA_DOT = V_REL.x / RT_MAG;
		ZCON = RT_MAG - R_REL.z;
		RTS_LV = _V(ZCON*sin(THETA), R_REL.y, RT_MAG - ZCON * cos(THETA));
		VTS_LV = _V(ZCON*THETA_DOT*cos(THETA) - V_REL.z*sin(THETA), V_REL.y, RTS_LV.x*THETA_DOT + V_REL.z*cos(THETA));
		RTS_LVIR = RTS_LV;
		VTS_LVIR = VTS_LV + crossp(OMEGA_LV_PROX, RTS_LVIR);
		RTS_M50 = tmul(MAT_M50_LVIR, RTS_LVIR);
		VTS_M50 = tmul(MAT_M50_LVIR, VTS_LVIR);
		R_S_INER = R_T_INER + RTS_M50;
		V_S_INER = V_T_INER + VTS_M50;
	}

	double calculateDifferenceBetweenAngles(double firstAngle, double secondAngle)
	{
		if (firstAngle > PI2)
		{
			firstAngle -= PI2;
		}
		else if (firstAngle < 0)
		{
			firstAngle += PI2;
		}

		if (secondAngle > PI2)
		{
			secondAngle -= PI2;
		}
		else if (secondAngle < 0)
		{
			secondAngle += PI2;
		}

		double difference = secondAngle - firstAngle;
		while (difference < -PI) difference += PI2;
		while (difference > PI) difference -= PI2;
		return difference;
	}

	MATRIX3 inverse(MATRIX3 a)
	{
		double det;

		det = determinant(a);
		return _M(a.m22*a.m33 - a.m23*a.m32, a.m13*a.m32 - a.m12*a.m33, a.m12*a.m23 - a.m13*a.m22, a.m23*a.m31 - a.m21*a.m33, a.m11*a.m33 - a.m13*a.m31, a.m13*a.m21 - a.m11*a.m23, a.m21*a.m32 - a.m22*a.m31, a.m12*a.m31 - a.m11*a.m32, a.m11*a.m22 - a.m12*a.m21) / det;

	}

	double determinant(MATRIX3 a)
	{
		return a.m11*a.m22*a.m33 + a.m12*a.m23*a.m31 + a.m13*a.m21*a.m32 - a.m13*a.m22*a.m31 - a.m12*a.m21*a.m33 - a.m11*a.m23*a.m32;
	}

	MATRIX3 tmat(MATRIX3 a)
	{
		MATRIX3 b;

		b = _M(a.m11, a.m21, a.m31, a.m12, a.m22, a.m32, a.m13, a.m23, a.m33);
		return b;
	}

	double MJDToDate(double MJD)
	{
		double JD, temp, JDint, JDfrac, tu, year, leapyrs, days;

		JD = MJD2JD(MJD);
		JDfrac = modf(JD, &JDint);
		temp = JDint - 2415019.5;
		tu = temp / 365.25;
		year = 1900 + (int)floor(tu);
		leapyrs = (int)floor((year - 1901) * 0.25);
		days = floor(temp - ((year - 1900) * 365.0 + leapyrs));

		return (days + JDfrac - 0.5)*24.0*3600.0;
	}

	MATRIX3 LVLH_Matrix(VECTOR3 R, VECTOR3 V)
	{
		VECTOR3 i, j, k;
		j = unit(crossp(V, R));
		k = unit(-R);
		i = crossp(j, k);
		return _M(i.x, i.y, i.z, j.x, j.y, j.z, k.x, k.y, k.z); //rotation matrix to LVLH
	}

	double GETfromMJD(double MJD, double GETBase)
	{
		return (MJD - GETBase)*24.0*3600.0;
	}

	double MJDfromGET(double GET, double GETBase)
	{
		return GETBase + GET / 24.0 / 3600.0;
	}

	int Date2JD(int Y, int M, int D)
	{
		return (1461 * (Y + 4800 + (M - 14) / 12)) / 4 + (367 * (M - 2 - 12 * ((M - 14) / 12))) / 12 - (3 * ((Y + 4900 + (M - 14) / 12) / 100)) / 4 + D - 32075;
	}

	double Date2MJD(int Y, int D, int H, int M, double S)
	{
		double MJD, timeofday;
		int JD;

		JD = Date2JD(Y, 1, 1);
		JD = JD + D - 1;
		MJD = (double)JD - 2400000.5;
		timeofday = 3600.0*(double)H + 60.0*(double)M + S;
		timeofday /= 24.0*3600.0;
		MJD = MJD + timeofday - 0.5;
		return MJD;
	}

	double JD2MJD(double jd)
	{
		return jd - 2400000.5;
	}

	double MJD2JD(double mjd)
	{
		return mjd + 2400000.5;
	}	

	VECTOR3 Ecl2M50(OBJHANDLE hEarth, VECTOR3 ecl)
	{
		MATRIX3 obliquityMat;
		oapiGetPlanetObliquityMatrix(hEarth, &obliquityMat);
		return rhtmul(obliquityMat, ecl);
	}

	OELEMENTS coe_from_sv(VECTOR3 R, VECTOR3 V, double mu)
	{
		/*% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		%{
		% This function computes the classical orbital elements(coe)
		% from the state vector(R, V) using Algorithm 4.1.
		%
		mu - gravitational parameter(km ^ 3 / s ^ 2)
		R - position vector in the geocentric equatorial frame(km)
		V - velocity vector in the geocentric equatorial frame(km)
		r, v - the magnitudes of R and V
		vr - radial velocity component(km / s)
		H - the angular momentum vector(km ^ 2 / s)
		h - the magnitude of H(km ^ 2 / s)
		incl - inclination of the orbit(rad)
		N - the node line vector(km ^ 2 / s)
		n - the magnitude of N
		cp - cross product of N and R
		RA - right ascension of the ascending node(rad)
		E - eccentricity vector
		e - eccentricity(magnitude of E)
		eps - a small number below which the eccentricity is considered
		to be zero
		w - argument of perigee(rad)
		TA - true anomaly(rad)
		a - semimajor axis(km)
		pi - 3.1415926...
		coe - vector of orbital elements[h e RA incl w TA a]
		User M - functions required : None
		%}
		% -------------------------------------------- -*/
		double eps, r, v, vr, h, incl, n, RA, e, w, TA;
		VECTOR3 H, N, E, cp;
		OELEMENTS coe;

		eps = 1.e-10;
		r = length(R);
		v = length(V);
		vr = dotp(R, V) / r;
		H = crossp(R, V);
		h = length(H);
		incl = acos(H.z / h);
		N = crossp(_V(0, 0, 1), H);
		n = length(N);
		if (n != 0)
		{
			RA = acos(N.x / n);
			if (N.y < 0)
			{
				RA = PI2 - RA;
			}
		}
		else
		{
			RA = 0;
		}

		E = (R*(OrbMech::power(v, 2) - mu / r) - V * r*vr) * 1.0 / mu;
		e = length(E);

		if (n != 0)
		{
			if (e > eps)
			{
				w = acos(dotp(N, E) / n / e);
				if (E.z < 0)
				{
					w = PI2 - w;
				}
			}
			else
			{
				w = 0;
			}
		}
		else
		{
			w = 0;
		}
		if (e > eps)
		{
			TA = acos2(dotp(unit(E), unit(R)));
			if (vr < 0.0)
			{
				TA = PI2 - TA;
			}
		}
		else
		{
			cp = crossp(N, R);
			if (cp.z >= 0)
			{
				TA = acos2(dotp(unit(N), unit(R)));
			}
			else
			{
				TA = PI2 - acos2(dotp(unit(N), unit(R)));
			}
		}
		//a = OrbMech::power(h,2) / mu / (1 - OrbMech::power(e,2));
		coe.e = e;
		coe.h = h;
		coe.i = incl;
		coe.RA = RA;
		coe.TA = TA;
		coe.w = w;
		return coe;
	}

	void sv_from_coe(OELEMENTS el, double mu, VECTOR3 &R, VECTOR3 &V)	//computes the state vector(R, V) from the classical orbital elements
	{
		double e, h, RA, incl, TA, w, rp_factor, vp_factor;
		VECTOR3 rp, vp;
		MATRIX3 R3_W, R1_i, R3_w, R2, Q_Xp;

		e = el.e;
		//a = el.a;
		//r_p = (1 - e)*a;
		h = el.h;// sqrt(r_p*mu*(1 + e));
		RA = el.RA;
		incl = el.i;
		TA = el.TA;
		w = el.w;// el.omegab - el.theta;
		rp_factor = h * h / (mu*(1.0 + e * cos(TA)));
		rp.x = rp_factor * cos(TA);
		rp.y = rp_factor * sin(TA);
		rp.z = 0;
		vp_factor = mu / h;
		vp.x = -vp_factor * sin(TA);
		vp.y = vp_factor * (e + cos(TA));
		vp.z = 0;
		R3_W = _M(cos(RA), sin(RA), 0.0, -sin(RA), cos(RA), 0.0, 0.0, 0.0, 1.0);
		R1_i = _M(1, 0, 0, 0, cos(incl), sin(incl), 0, -sin(incl), cos(incl));
		R3_w = _M(cos(w), sin(w), 0, -sin(w), cos(w), 0, 0, 0, 1);
		R2 = mul(R3_w, R1_i);
		Q_Xp = mul(R2, R3_W);
		R = tmul(Q_Xp, rp);
		V = tmul(Q_Xp, vp);
	}

	CELEMENTS CartesianToKeplerian(VECTOR3 R, VECTOR3 V, double mu)
	{
		CELEMENTS out;
		OELEMENTS coe = coe_from_sv(R, V, mu);

		out.a = coe.h*coe.h / (mu*(1.0 - coe.e*coe.e));
		out.e = coe.e;
		out.i = coe.i;
		out.g = coe.w;
		out.h = coe.RA;
		out.l = TrueToMeanAnomaly(coe.TA, coe.e);

		return out;
	}

	void KeplerianToCartesian(CELEMENTS coe, double mu, VECTOR3 &R, VECTOR3 &V)
	{
		OELEMENTS el;

		el.e = coe.e;
		el.h = sqrt(coe.a*mu*(1.0 - coe.e*coe.e));
		el.i = coe.i;
		el.RA = coe.h;
		el.TA = MeanToTrueAnomaly(coe.l, coe.e);
		el.w = coe.g;

		sv_from_coe(el, mu, R, V);
	}

	CELEMENTS BrouwerMeanLongToOsculatingElements(CELEMENTS mean)
	{
		CELEMENTS out = mean;

		int pseudostate = 0;

		//Earth radius
		double re = 6.37101e6;
		double j2 = 1082.6269e-6;
		double j3 = -2.51e-6;
		double j4 = -1.60e-6;
		double j5 = -0.15e-6;

		//GMAT
		/*double re = 6.3781363e6;
		double j2 = 1.082626925638815e-03;
		double j3 = -0.2532307818191774e-5;
		double j4 = -0.1620429990000000e-5;
		double j5 = -0.2270711043920343e-6;*/

		//Earth radius
		double ae = 1.0;
		//Semi-major axis in Earth radii
		double smadp = mean.a / re;
		double eccdp = mean.e;
		double incdp = mean.i;
		double raandp = mean.h;
		double aopdp = mean.g;
		double meanAnom = mean.l;

		if (incdp > 175.0*RAD)
		{
			incdp = PI - incdp;
			raandp = -raandp;
			pseudostate = 1;
		}
		if (eccdp > 0.99)
		{
			//Error
			return out;
		}
		double radper = mean.a*(1.0 - mean.e);
		if (radper < 3000.0e3)
		{
			//Error
			return out;
		}
		if (mean.i > PI)
		{
			//Error
			return out;
		}
		raandp = fmod(raandp, PI2);
		aopdp = fmod(aopdp, PI2);
		meanAnom = fmod(meanAnom, PI2);
		if (raandp < 0.0)
		{
			raandp = raandp + PI2;
		}
		if (aopdp < 0.0)
		{
			aopdp = aopdp + PI2;
		}
		if (meanAnom < 0.0)
		{
			meanAnom = meanAnom + PI2;
		}

		double bk2 = (1.0 / 2.0)*(j2*ae*ae);
		double bk3 = -j3 * pow(ae, 3.0);
		double bk4 = -(3.0 / 8.0)*j4*pow(ae, 4.0);
		double bk5 = -j5 * pow(ae, 5.0);
		double eccdp2 = eccdp * eccdp;
		double cn2 = 1.0 - eccdp2;
		double cn = sqrt(cn2);

		double gm2 = bk2 / pow(smadp, 2.0);
		double gmp2 = gm2 / (cn2*cn2);
		double gm3 = bk3 / pow(smadp, 3.0);
		double gmp3 = gm3 / (cn2*cn2*cn2);
		double gm4 = bk4 / pow(smadp, 4.0);
		double gmp4 = gm4 / pow(cn, 8.0);
		double gm5 = bk5 / pow(smadp, 5.0);
		double gmp5 = gm5 / pow(cn, 10.0);

		double g3dg2 = gmp3 / gmp2;
		double g4dg2 = gmp4 / gmp2;
		double g5dg2 = gmp5 / gmp2;

		double theta = cos(incdp);
		double theta2 = theta * theta;
		double theta4 = theta2 * theta2;

		double sinMADP = sin(meanAnom);
		double cosMADP = cos(meanAnom);
		double sinraandp = sin(raandp);
		double cosraandp = cos(raandp);

		double tadp = MeanToTrueAnomaly(meanAnom, eccdp, 1e-12);

		double rp = smadp * (1.0 - eccdp * eccdp) / (1.0 + eccdp * cos(tadp));
		double adr = smadp / rp;
		double sinta = sin(tadp);
		double costa = cos(tadp);
		double cs2gta = cos(2.0*aopdp + 2.0*tadp);
		double adr2 = adr * adr;
		double adr3 = adr2 * adr;
		double costa2 = costa * costa;

		double a1 = ((1.0 / 8.0)*gmp2*cn2)*(1.0 - 11.0*theta2 - ((40.0*theta4) / (1.0 - 5.0*theta2)));
		double a2 = ((5.0 / 12.0)*g4dg2*cn2)*(1.0 - ((8.0*theta4) / (1.0 - 5.0*theta2)) - 3.0*theta2);
		double a3 = g5dg2 * ((3.0*eccdp2) + 4.0);
		double a4 = g5dg2 * (1.0 - (24.0*theta4) / (1.0 - 5.0*theta2) - 9.0*theta2);
		double a5 = (g5dg2*(3.0*eccdp2 + 4.0))*(1.0 - (24.0*theta4) / (1.0 - 5.0*theta2) - 9.0*theta2);
		double a6 = g3dg2 * (1.0 / 4.0);
		double sinI = sin(incdp);
		double a10 = cn2 * sinI;
		double a7 = a6*a10;
		double a8p = g5dg2 * eccdp*(1.0 - (16.0*theta4) / (1.0 - 5.0*theta2) - 5.0*theta2);
		double a8 = a8p * eccdp;

		double b13 = eccdp * (a1 - a2);
		double b14 = a7 + (5.0 / 64.0)*a5*a10;
		double b15 = a8 * a10*(35.0 / 384.0);

		double a11 = 2.0 + eccdp2;
		double a12 = 3.0*eccdp2 + 2.0;
		double a13 = theta2 * a12;
		double a14 = (5.0*eccdp2 + 2.0)*(theta4 / (1.0 - 5.0*theta2));
		double a17 = theta4 / ((1.0 - 5.0*theta2)*(1.0 - 5.0*theta2));
		double a15 = (eccdp2*theta4*theta2) / ((1.0 - 5.0*theta2)*(1.0 - 5.0*theta2));
		double a16 = theta2 / (1.0 - 5.0*theta2);
		double a18 = eccdp * sinI;
		double a19 = a18 / (1.0 + cn);
		double a21 = eccdp * theta;
		double a22 = eccdp2 * theta;
		double sinI2 = sin(incdp / 2.0);
		double cosI2 = cos(incdp / 2.0);
		double tanI2 = tan(incdp / 2.0);
		double a26 = 16.0*a16 + 40.0*a17 + 3.0;
		double a27 = a22 * (1.0 / 8.0)*(11.0 + 200.0*a17 + 80.0*a16);

		double b1 = cn * (a1 - a2) - ((a11 - 400.0*a15 - 40.0*a14 - 11.0*a13)*(1.0 / 16.0) + (11.0 + 200.0*a17 + 80.0*a16)*a22*(1.0 / 8.0))*gmp2 + ((-80.0*a15 - 8.0*a14 - 3.0*a13 + a11)*(5.0 / 24.0) + (5.0 / 12.0)*a26*a22)*g4dg2;
		double b2 = a6 * a19*(2.0 + cn - eccdp2) + (5.0 / 64.0)*a5*a19*cn2 - (15.0 / 32.0)*a4*a18*cn*cn2 + ((5.0 / 64.0)*a5 + a6)*a21*tanI2 + (9.0*eccdp2 + 26.0)*(5.0 / 64.0)*a4*a18 + (15.0 / 32.0)*a3*a21*a26*sinI*(1.0 - theta);
		double b3 = ((80.0*a17 + 5.0 + 32.0*a16)*a22*sinI*(theta - 1.0)*(35.0 / 576.0)*g5dg2*eccdp) - ((a22*tanI2 + (2.0*eccdp2 + 3.0*(1.0 - cn2 * cn))*sinI)*(35.0 / 1152.0)*a8p);
		double b4 = cn * eccdp*(a1 - a2);
		double b5 = ((9.0*eccdp2 + 4.0)*a10*a4*(5.0 / 64.0) + a7)*cn;
		double b6 = (35.0 / 384.0)*a8*cn2*cn*sinI;
		double b7 = ((cn2*a18) / (1.0 - 5.0*theta2))*((1.0 / 8.0)*gmp2*(1.0 - 15.0*theta2) + (1.0 - 7.0*theta2)*g4dg2*(-(5.0 / 12.0)));
		double b8 = (5.0 / 64.0)*(a3*cn2*(1.0 - 9.0*theta2 - (24.0*theta4 / (1.0 - 5.0*theta2)))) + a6 * cn2;
		double b9 = a8 * (35.0 / 384.0)*cn2;
		double b10 = sinI * (a22*a26*g4dg2*(5.0 / 12.0) - a27 * gmp2);
		double b11 = a21 * (a5*(5.0 / 64.0) + a6 + a3 * a26*(15.0 / 32.0)*sinI*sinI);
		double b12 = -((80.0*a17 + 32.0*a16 + 5.0)*(a22*eccdp*sinI*sinI*(35.0 / 576.0)*g5dg2) + (a8*a21*(35.0 / 1152.0)));

		double sma = smadp * (1.0 + gm2 * ((3.0*theta2 - 1.0)*(eccdp2 / (cn2*cn2*cn2))*(cn + (1.0 / (1.0 + cn))) + ((3.0*theta2 - 1.0) / (cn2*cn2*cn2))*(eccdp*costa)*(3.0 + 3.0*eccdp*costa + eccdp2 * costa2) + 3.0*(1.0 - theta2)*adr3*cs2gta));
		
		double sn2gta = sin(2.0*aopdp + 2.0*tadp);
		double snf2gd = sin(2.0*aopdp + tadp);
		double csf2gd = cos(2.0*aopdp + tadp);
		double sn2gd = sin(2.0*aopdp);
		double cs2gd = cos(2.0*aopdp);
		double sin3gd = sin(3.0*aopdp);
		double cs3gd = cos(3.0*aopdp);
		double sn3fgd = sin(3.0*tadp + 2.0*aopdp);
		double cs3fgd = cos(3.0*tadp + 2.0*aopdp);
		double sinGD = sin(aopdp);
		double cosGD = cos(aopdp);		

		double bisubc = pow((1.0 - 5.0*theta2), -2.0)*((25.0*theta4*theta)*(gmp2*eccdp2));
		double blghp, eccdpdl, dltI, sinDH, dlt1e;

		if (bisubc >= 0.001)
		{
			dlt1e = 0.0;
			blghp = 0.0;
			eccdpdl = 0.0;
			dltI = 0.0;
			sinDH = 0.0;
		}
		else
		{
			blghp = raandp + aopdp + meanAnom + b3 * cs3gd + b1 * sn2gd + b2 * cosGD;
			blghp = fmod(blghp, PI2);
			if (blghp < 0.0)
			{
				blghp = blghp + PI2;
			}
			dlt1e = b14 * sinGD + b13 * cs2gd - b15 * sin3gd;
			eccdpdl = b4 * sn2gd - b5 * cosGD + b6 * cs3gd - (1.0 / 4.0)*cn2*cn*gmp2*(2.0*(3.0*theta2 - 1.0)*(adr2*cn2 + adr + 1.0)*sinta + 3.0*(1.0 - theta2)*((-adr2 * cn2 - adr + 1.0)*snf2gd + (adr2*cn2 + adr + (1.0 / 3.0))*sn3fgd));
			dltI = (1.0 / 2.0)*theta*gmp2*sinI*(eccdp*cs3fgd + 3.0*(eccdp*csf2gd + cs2gta)) - (a21 / cn2)*(b8*sinGD + b7 * cs2gd - b9 * sin3gd);
			sinDH = (1.0 / cosI2)*((1.0 / 2.0)*(b12*cs3gd + b11 * cosGD + b10 * sn2gd - ((1.0 / 2.0)*gmp2*theta*sinI*(6.0*(eccdp*sinta - meanAnom + tadp) - (3.0*(sn2gta + eccdp * snf2gd) + eccdp * sn3fgd)))));
		}

		double blgh = blghp + ((1.0 / (cn + 1.0))*(1.0 / 4.0)*eccdp*gmp2*cn2*(3.0*(1.0 - theta2)*(sn3fgd*((1.0 / 3.0) + adr2 * cn2 + adr) + snf2gd * (1.0 - (adr2*cn2 + adr))) + 2.0*sinta*(3.0*theta2 - 1.0)*(adr2*cn2 + adr + 1.0)))
			+ gmp2 * (3.0 / 2.0)*((-2.0*theta - 1.0 + 5.0*theta2)*(eccdp*sinta + tadp - meanAnom)) + (3.0 + 2.0*theta - 5.0*theta2)*(gmp2*(1.0 / 4.0)*(eccdp*sn3fgd + 3.0*(sn2gta + eccdp * snf2gd)));
		blgh = fmod(blgh, PI2);
		if (blgh < 0.0)
		{
			blgh = blgh + PI2;
		}

		double dlte = dlt1e + ((1.0 / 2.0)*cn2*((3.0*(1.0 / (cn2*cn2*cn2))*gm2*(1.0 - theta2)*cs2gta*(3.0*eccdp*costa2 + 3.0*costa + eccdp2 * costa*costa2 + eccdp))
			- (gmp2*(1.0 - theta2)*(3.0*csf2gd + cs3fgd)) + (3.0*theta2 - 1.0)*gm2*(1.0 / (cn2*cn2*cn2))*(eccdp*cn + (eccdp / (1.0 + cn)) + 3.0*eccdp*costa2 + 3.0*costa + eccdp2 * costa*costa2)));
		double eccdpdl2 = eccdpdl * eccdpdl;
		double eccdpde2 = (eccdp + dlte)*(eccdp + dlte);

		double ecc = sqrt(eccdpdl2 + eccdpde2);
		double sinDH2 = sinDH * sinDH;
		double squar = (dltI*cosI2*(1.0 / 2.0) + sinI2)*(dltI*cosI2*(1.0 / 2.0) + sinI2);
		double sqrI = sqrt(sinDH2 + squar);

		double inc = 2 * asin(sqrI);
		inc = fmod(inc, PI2);

		double ma, raan, aop;
		if (ecc <= 1.0e-11)
		{
			aop = 0.0;
			if (inc <= 1.0e-7)
			{
				raan = 0.0;
				ma = blgh;
			}
			else
			{
				double arg1 = sinDH * cosraandp + sinraandp * ((1.0 / 2.0)*dltI*cosI2 + sinI2);
				double arg2 = cosraandp * ((1.0 / 2.0)*dltI*cosI2 + sinI2) - (sinDH*sinraandp);
				raan = atan2(arg1, arg2);
				ma = blgh - aop - raan;
			}
		}
		else
		{
			double arg1 = eccdpdl * cosMADP + (eccdp + dlte)*sinMADP;
			double arg2 = (eccdp + dlte)*cosMADP - (eccdpdl*sinMADP);
			ma = atan2(arg1, arg2);
			ma = fmod(ma, PI2);

			if (inc <= 1.0e-7)
			{
				raan = 0.0;
				aop = blgh - raan - ma;
			}
			else
			{
				double arg1 = sinDH * cosraandp + sinraandp * ((1.0 / 2.0)*dltI*cosI2 + sinI2);
				double arg2 = cosraandp * ((1.0 / 2.0)*dltI*cosI2 + sinI2) - (sinDH*sinraandp);
				raan = atan2(arg1, arg2);
				aop = blgh - ma - raan;
			}
		}

		if (ma < 0.0)
		{
			ma = ma + PI2;
		}

		raan = fmod(raan, PI2);
		if (raan < 0)
		{
			raan = raan + PI2;
		}

		aop = fmod(aop, PI2);
		if (aop < 0.0)
		{
			aop = aop + PI2;
		}

		/*
		//Lyddane Cohen Correction
		double cosI2osc = cos(inc)*cos(inc);
		double taosc = MeanToTrueAnomaly(ma, ecc);
		double cs2gtaosc = cos(2.0*aop + 2.0*taosc);
		double cnosc = sqrt(1.0 - ecc * ecc);
		double adrosc = (1.0 + ecc * cos(taosc)) / (1.0 - ecc * ecc);
		double adrosc3 = adrosc*adrosc*adrosc;
		double Xi = (-1.0 + 3.0*cosI2osc)*(adrosc3 - cnosc*cnosc*cnosc) + 3.0*(1.0 - cosI2osc)*adrosc3*cs2gtaosc;

		double ecc2 = ecc * ecc;
		double SARG = 1.0 - ecc2;
		double AFPRIM = MeanToTrueAnomaly(ma, ecc);
		theta = cos(inc);
		theta2 = theta * theta;
		cn = sqrt(SARG);
		cn2 = cn * cn;
		double cn3 = cn2 * cn;
		double SARG3 = 1.0 - 3.0*theta2;
		double DA1 = 102.0*SARG3;

		double DA2 = cos(aop);
		double DA3 = sin(aop);
		double DA4 = cos(AFPRIM);
		double DA5 = sin(AFPRIM);
		double ADVERR = (1.0 + ecc * DA4) / SARG;
		double DA7 = DA3 * DA4 + DA2 * DA5;
		double DA72 = DA7 * DA7;
		double PH20 = ADVERR * ADVERR*ADVERR;
		double Xitest = DA1 + (302.0*(1.0 - theta2)*(1.0 - DA72 * DA72) - DA1)*PH20*cn3;

		double A0 = -5016.0*(cn2*(1.0-theta2*(3.6-theta2))+0.8*cn*(1.0-theta2*(6.0-DA12))-(1.0-theta2*(12.0+D10)));
		double A1 = 203.0/PGAM52+cn*SARG3;
		double A6 = 108.0*ecc2*(1.0-theta2*(16.0-15.0*theta2));
		double A4 = 2.0*ecc*DA8;
		double CAPC=DA9/cn3*(A0+XI*(A1-cn*XI)+(302.0*DA8+A4*DA4)*(1.0-2.0*DA12)+A4*DA5*DA6*DA7*A6*(1.0-2.0*DA32));


		//double Xi = (-1.0 + 3.0*theta2)*(adr3 - cn * cn*cn) + 3.0*(1.0 - cosI2)*adr3*cs2gta;
		double cn7 = cn2*cn2*cn2*cn;
		double test = (1.0 / (sma*sma*sma))*(bk2*Xi*(sma*sma - bk2 * Xi + (3.0 / 2.0)*bk2 / (cn*cn*cn)*(1.0 - 3.0*theta2)) - (bk2*bk2 / (16.0*cn7))*(15.0*cn2*(1.0 - (18.0 / 5.0)*theta2 + theta4)
			+ 12.0*cn*(1.0 - 6.0*theta2 + 9.0*theta4) - 15.0*(1.0 - 2.0*theta2 - 7.0*theta4) + 6.0*eccdp2*(1.0 - 16.0*theta2 + 15.0*theta4)*cs2gd)
			+ (9.0*bk2*bk2 / (2.0*cn7))*(1.0 - 6.0*theta2 + 5.0*theta4)*(cs2gta + eccdp * csf2gd + (1.0 / 3.0)*eccdp*cs3fgd));

		sma = smadp + test;*/

		out.a = sma * re;
		out.e = ecc;
		out.g = aop;
		out.h = raan;
		out.i = inc;
		out.l = ma;

		if (pseudostate != 0)
		{
			out.i = PI - out.i;
			out.h = PI2 - out.h;
		}

		return out;
	}

	void BrouwerSecularRates(CELEMENTS mean, double mu, double &l_dot, double &g_dot, double &h_dot, double &n0)
	{
		//Orbiter 2016
		double re = 6.37101e6;
		double j2 = 1082.6269e-6;
		double j4 = -1.60e-6;

		double ae = 1.0;
		double smadp = mean.a / re;
		double eccdp = mean.e;
		double incdp = mean.i;

		n0 = sqrt(mu / (pow(mean.a, 3.0)));
		double eccdp2 = eccdp * eccdp;
		double cn2 = 1.0 - eccdp2;
		double cn = sqrt(cn2);
		double bk2 = (1.0 / 2.0)*(j2*ae*ae);
		double bk4 = -(3.0 / 8.0)*j4*pow(ae, 4.0);
		double gm2 = bk2 / pow(smadp, 2.0);
		double gmp2 = gm2 / (cn2*cn2);
		double gm4 = bk4 / pow(smadp, 4.0);
		double gmp4 = gm4 / pow(cn, 8.0);
		double theta = cos(incdp);
		double theta2 = theta * theta;
		double theta3 = theta2 * theta;
		double theta4 = theta2 * theta2;

		l_dot = n0 * cn*(gmp2*((3.0 / 2.0)*(3.0*theta2 - 1.0) + (3.0 / 32.0)*gmp2*(25.0*cn2 + 16.0*cn - 15.0 + (30.0 - 96.0*cn - 90.0*cn2)*theta2 +
			(105.0 + 144.0*cn + 25.0*cn2)*theta4)) + (15.0 / 16.0)*gmp4*eccdp2*(3.0 - 30.0*theta2 + 35.0*theta4));
		g_dot = n0 * (gmp2*((3.0 / 2.0)*(5.0*theta2 - 1.0) + (3.0 / 32.0)*gmp2*(25.0*cn2 + 24.0*cn - 35.0
			+ (90.0 - 192.0*cn - 126.0*cn2)*theta2 + (385.0 + 360.0*cn + 45.0*cn2)*theta4))
			+ (5.0 / 16.0)*gmp4*(21.0 - 9.0*cn2 + (126.0*cn2 - 270.0)*theta2 + (385.0 - 189.0*cn2)*theta4));
		h_dot = n0 * (gmp2*((3.0 / 8.0)*gmp2*((9.0*cn2 + 12.0*cn - 5.0)*theta - (35.0 + 36.0*cn + 5.0*cn2)*theta3) - 3.0*theta)
			+ (5.0 / 4.0)*gmp4*theta*(5.0 - 3.0*cn2)*(3.0 - 7.0*theta2));
	}

	CELEMENTS OsculatingToBrouwerMeanLong(CELEMENTS osc, double mu)
	{
		VECTOR3 R, V;
		KeplerianToCartesian(osc, mu, R, V);
		return CartesianToBrouwerMeanLong(R, V, mu);
	}

	CELEMENTS CartesianToBrouwerMeanLong(VECTOR3 R, VECTOR3 V, double mu)
	{
		VECTOR3 R0, V0;
		CELEMENTS out;
		double tol = 1.0e-8;
		int maxiter = 75;
		R0 = R;
		V0 = V;

		CELEMENTS kep = CartesianToKeplerian(R, V, mu);
		out = kep;

		if ((kep.e > 0.99) || (kep.e < 0.0))
		{
			//Error
			return out;
		}
		double radper = (kep.a*(1.0 - kep.e));
		if (radper < 3000.0e3)
		{
			//Error
			return out;
		}
		if (kep.i > PI)
		{
			//Error
			return out;
		}
		int pseudostate = 0;
		if (kep.i > 175.0*RAD)
		{
			kep.i = PI - kep.i;
			kep.h = -kep.h;
			KeplerianToCartesian(kep, mu, R0, V0);
			pseudostate = 1;
		}

		CELEMENTS blmean = kep;
		CELEMENTS kep2;
		kep2 = BrouwerMeanLongToOsculatingElements(kep);
		CELEMENTS blmean2;

		CELEMENTS aeq, aeq2, aeqmean, aeqmean2;

		aeq.a = kep.a;
		aeq.e = kep.e*sin(kep.g + kep.h);
		aeq.i = kep.e*cos(kep.g + kep.h);
		aeq.h = sin(kep.i / 2.0)*sin(kep.h);
		aeq.g = sin(kep.i / 2.0)*cos(kep.h);
		aeq.l = kep.h + kep.g + kep.l;

		aeq2.a = kep2.a;
		aeq2.e = kep2.e*sin(kep2.g + kep2.h);
		aeq2.i = kep2.e*cos(kep2.g + kep2.h);
		aeq2.h = sin(kep2.i / 2.0)*sin(kep2.h);
		aeq2.g = sin(kep2.i / 2.0)*cos(kep2.h);
		aeq2.l = kep2.h + kep2.g + kep2.l;

		aeqmean.a = blmean.a;
		aeqmean.e = blmean.e*sin(blmean.g + blmean.h);
		aeqmean.i = blmean.e*cos(blmean.g + blmean.h);
		aeqmean.h = sin(blmean.i / 2.0)*sin(blmean.h);
		aeqmean.g = sin(blmean.i / 2.0)*cos(blmean.h);
		aeqmean.l = blmean.h + blmean.g + blmean.l;

		aeqmean2 = aeqmean + (aeq - aeq2);

		double emag = 0.9;
		double emag_old = 1.0;
		int ii = 0;
		VECTOR3 R2, V2, Rtemp, Vtemp;

		while (emag > tol)
		{
			blmean2.a = aeqmean2.a;
			blmean2.e = sqrt(aeqmean2.e*aeqmean2.e + aeqmean2.i*aeqmean2.i);
			if ((aeqmean2.h*aeqmean2.h + aeqmean2.g*aeqmean2.g) <= 1.0)
				blmean2.i = acos2(1.0 - 2.0*(aeqmean2.h*aeqmean2.h + aeqmean2.g*aeqmean2.g));
			if ((aeqmean2.h*aeqmean2.h + aeqmean2.g*aeqmean2.g) > 1.0)
				blmean2.i = acos2(1.0 - 2.0*1.0);

			blmean2.h = atan2(aeqmean2.h, aeqmean2.g);
			if (blmean2.h < 0)
			{
				blmean2.h = blmean2.h + PI2;
			}
			blmean2.g = atan2(aeqmean2.e, aeqmean2.i) - blmean2.h;
			blmean2.l = aeqmean2.l - atan2(aeqmean2.e, aeqmean2.i);

			kep2 = BrouwerMeanLongToOsculatingElements(blmean2);
			KeplerianToCartesian(kep2, mu, R2, V2);
			Rtemp = R0 - R2;
			Vtemp = V0 - V2;

			emag = sqrt(pow(Rtemp.x, 2.0) + pow(Rtemp.y, 2.0) + pow(Rtemp.z, 2.0) + pow(Vtemp.x, 2.0) + pow(Vtemp.y, 2.0) + pow(Vtemp.z, 2.0)) / sqrt(pow(R0.x, 2.0) + pow(R0.y, 2.0)
				+ pow(R0.z, 2.0) + pow(V0.x, 2.0) + pow(V0.y, 2.0) + pow(V0.z, 2.0));

			if (emag_old > emag)
			{
				emag_old = emag;

				aeq2.a = kep2.a;
				aeq2.e = kep2.e*sin(kep2.g + kep2.h);
				aeq2.i = kep2.e*cos(kep2.g + kep2.h);
				aeq2.h = sin(kep2.i / 2.0)*sin(kep2.h);
				aeq2.g = sin(kep2.i / 2.0)*cos(kep2.h);
				aeq2.l = kep2.h + kep2.g + kep2.l;

				aeqmean = aeqmean2;
				aeqmean2 = aeqmean + (aeq - aeq2);
			}
			else
			{
				//Error
				break;
			}
			if (ii > maxiter)
			{
				//Error
				break;
			}
			ii = ii + 1;
		}

		blmean.a = aeqmean.a;
		blmean.e = sqrt(aeqmean.e*aeqmean.e + aeqmean.i*aeqmean.i);
		if ((aeqmean.h*aeqmean.h + aeqmean.g*aeqmean.g) <= 1.0)
			blmean.i = acos2(1.0 - 2.0*(aeqmean.h*aeqmean.h + aeqmean.g*aeqmean.g));
		if ((aeqmean.h*aeqmean.h + aeqmean.g*aeqmean.g) > 1.0)
			blmean.i = acos2(1.0 - 2.0*1.0);

		blmean.h = atan2(aeqmean.h, aeqmean.g);
		if (blmean.h < 0)
		{
			blmean.h = blmean.h + PI2;
		}
		blmean.g = atan2(aeqmean.e, aeqmean.i) - blmean.h;
		blmean.l = aeqmean.l - atan2(aeqmean.e, aeqmean.i);

		if (pseudostate != 0)
		{
			blmean.i = PI - blmean.i;
			blmean.h = -blmean.h;
		}

		blmean.h = fmod(blmean.h, PI2);
		blmean.g = fmod(blmean.g, PI2);
		blmean.l = fmod(blmean.l, PI2);
		if (blmean.h < 0.0)
		{
			blmean.h = blmean.h + PI2;
		}
		if (blmean.g < 0.0)
		{
			blmean.g = blmean.g + PI2;
		}
		if (blmean.l < 0.0)
		{
			blmean.l = blmean.l + PI2;
		}

		return blmean;
	}

	void AEGServiceRoutine(VECTOR3 R, VECTOR3 V, double MJD, int opt, double dval, double DN, VECTOR3 &R2, VECTOR3 &V2, double &MJD_out)
	{
		VECTOR3 R_equ0, V_equ0, R_equ1, V_equ1;
		double DeltaTime;
		OBJHANDLE hEarth = oapiGetObjectByName("Earth");

		EclipticToECI(R, V, MJD, R_equ0, V_equ0);
		double mu = GGRAV * oapiGetMass(hEarth);
		CELEMENTS osc0 = CartesianToKeplerian(R_equ0, V_equ0, mu);

		CELEMENTS osc1 = AnalyticEphemerisGenerator(osc0, opt, dval, DN, mu, DeltaTime);

		KeplerianToCartesian(osc1, mu, R_equ1, V_equ1);
		ECIToEcliptic(R_equ1, V_equ1, MJD, R2, V2);
		MJD_out = MJD + DeltaTime / 24.0 / 3600.0;
	}

	CELEMENTS AnalyticEphemerisGenerator(CELEMENTS osc0, int opt, double dval, double DN, double mu, double &DeltaTime)
	{
		//INPUT:
		//opt: 0 = update to time, 1 = update to mean anomaly, 2 = update to argument of latitude, 3 = update to maneuver counter line

		double l_dot, g_dot, h_dot, n0, dt, ll_dot;
		CELEMENTS mean0, mean1, osc1;

		mean0 = OsculatingToBrouwerMeanLong(osc0, mu);
		BrouwerSecularRates(mean0, mu, l_dot, g_dot, h_dot, n0);
		ll_dot = l_dot + n0;

		if (opt == 0)
		{
			dt = dval;

			mean1.a = mean0.a;
			mean1.e = mean0.e;
			mean1.i = mean0.i;
			mean1.l = ll_dot * dt + mean0.l;
			mean1.g = g_dot * dt + mean0.g;
			mean1.h = h_dot * dt + mean0.h;
			DeltaTime = dt;
		}
		else
		{
			double DX_L, X_L, X_L_dot, ddt, L_D;
			int LINE, COUNT;
			bool DH;

			osc1 = osc0;
			if (opt != 3)
			{
				L_D = dval;
			}
			else
			{
				double u = MeanToTrueAnomaly(osc1.l, osc1.e) + osc1.g;
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
					double u = MeanToTrueAnomaly(osc1.l, osc1.e) + osc1.g;
					u = fmod(u, PI2);
					if (u < 0)
						u += PI2;

					X_L = u;
					X_L_dot = ll_dot + g_dot;
				}
				//Maneuver line
				else
				{
					double u = MeanToTrueAnomaly(osc1.l, osc1.e) + osc1.g;
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

				mean1.a = mean0.a;
				mean1.e = mean0.e;
				mean1.i = mean0.i;
				mean1.l = ll_dot * dt + mean0.l;
				mean1.g = g_dot * dt + mean0.g;
				mean1.h = h_dot * dt + mean0.h;

				osc1 = BrouwerMeanLongToOsculatingElements(mean1);

				COUNT--;

			} while (abs(DX_L) > 0.0005 && COUNT > 0);

			DeltaTime = dt;
		}

		mean1.l = fmod(mean1.l, PI2);
		if (mean1.l < 0)
		{
			mean1.l = mean1.l + PI2;
		}

		mean1.g = fmod(mean1.g, PI2);
		if (mean1.g < 0)
		{
			mean1.g = mean1.g + PI2;
		}

		mean1.h = fmod(mean1.h, PI2);
		if (mean1.h < 0)
		{
			mean1.h = mean1.h + PI2;
		}

		osc1 = BrouwerMeanLongToOsculatingElements(mean1);

		return osc1;
	}

	double timetoapo(VECTOR3 R, VECTOR3 V, double mu, int s)
	{
		//s = 1: ensure the next apoapsis is returned

		OELEMENTS coe;
		double a, chi, alpha, r0, vr0, dt;

		coe = coe_from_sv(R, V, mu);
		//[h e RA incl w TA a]

		a = coe.h*coe.h / mu / (1.0 - coe.e*coe.e);

		double E_0;
		E_0 = 2 * atan(sqrt((1.0 - coe.e) / (1.0 + coe.e))*tan(coe.TA / 2.0));
		chi = sqrt(a)*(PI - E_0);

		alpha = 1.0 / a;

		r0 = length(R);
		vr0 = dotp(R, V) / r0;
		dt = kepler_U_equation(chi, r0, vr0, alpha, mu);

		if (s == 0 || dt >= 0)
		{
			return dt;
		}

		double T_P = period(R, V, mu);

		return dt + T_P;
	}

	double timetoperi(VECTOR3 R, VECTOR3 V, double mu, int s)
	{
		OELEMENTS coe;
		double a, chi, alpha, r0, vr0, dt;

		coe = coe_from_sv(R, V, mu);
		//[h e RA incl w TA a]

		a = coe.h*coe.h / mu / (1.0 - coe.e*coe.e);

		if (coe.e > 1.0)
		{
			double F;
			F = log((sqrt(coe.e + 1.0) + sqrt(coe.e - 1.0)*tan(coe.TA / 2.0)) / (sqrt(coe.e + 1.0) - sqrt(coe.e - 1.0)*tan(coe.TA / 2.0)));
			chi = -sqrt(-a)*F;
		}
		else if (coe.e == 1)
		{
			chi = -coe.h / sqrt(mu)*tan(coe.TA / 2.0);
		}
		else
		{
			double E_0;
			E_0 = 2.0 * atan(sqrt((1.0 - coe.e) / (1.0 + coe.e))*tan(coe.TA / 2.0));
			chi = -sqrt(a)*E_0;
		}

		alpha = 1.0 / a;

		r0 = length(R);
		vr0 = dotp(R, V) / r0;
		dt = kepler_U_equation(chi, r0, vr0, alpha, mu);

		if (s == 0 || dt >= 0)
		{
			return dt;
		}

		double T_P = period(R, V, mu);

		return dt + T_P;
	}

	double timetoapo_integ(VECTOR3 R, VECTOR3 V, double MJD)
	{
		VECTOR3 R2, V2;

		return timetoapo_integ(R, V, MJD, R2, V2);
	}

	double timetoapo_integ(VECTOR3 R, VECTOR3 V, double MJD, VECTOR3 &R2, VECTOR3 &V2)
	{
		OBJHANDLE hEarth;
		OELEMENTS coe;
		VECTOR3 R0, V0, R1, V1;
		double mu, dt, dt_total, T_p;
		int n, nmax;

		hEarth = oapiGetObjectByName("Earth");
		mu = GGRAV * oapiGetMass(hEarth);
		dt_total = 0.0;
		n = 0;
		nmax = 20;

		R0 = R;
		V0 = V;

		coe = coe_from_sv(R0, V0, mu);
		T_p = period(R0, V0, mu);

		if (coe.e > 0.005)
		{
			dt = timetoapo(R0, V0, mu, 1);
			oneclickcoast(R, V, MJD, dt, R1, V1);

			dt_total += dt;

			do
			{
				dt = timetoapo(R1, V1, mu);
				T_p = period(R1, V1, mu);
				if (dt_total + dt > T_p)
				{
					dt -= T_p;
				}
				oneclickcoast(R1, V1, MJD + dt_total / 24.0 / 3600.0, dt, R1, V1);
				dt_total += dt;
				n++;
			} while (abs(dt) > 0.01 && nmax >= n);

		}
		else
		{
			MATRIX3 Rot;
			VECTOR3 Rt[3], Vt[3], Rt_equ, Vt_equ, R11, R12, V11, V12;
			double u[3], r[3], gamma, u0, ux, uy, du1, du2, dt1, dt2, vr;

			R1 = R0;
			V1 = V0;

			oneclickcoast(R1, V1, MJD + dt_total / 24.0 / 3600.0, 0.0*60.0, Rt[0], Vt[0]);
			oneclickcoast(R1, V1, MJD + dt_total / 24.0 / 3600.0, 15.0*60.0, Rt[1], Vt[1]);
			oneclickcoast(R1, V1, MJD + dt_total / 24.0 / 3600.0, 30.0*60.0, Rt[2], Vt[2]);

			Rot = GetObliquityMatrix(hEarth, MJD + dt_total / 24.0 / 3600.0);

			for (int i = 0;i < 3;i++)
			{
				Rt_equ = rhtmul(Rot, Rt[i]);
				Vt_equ = rhtmul(Rot, Vt[i]);
				coe = coe_from_sv(Rt_equ, Vt_equ, mu);

				r[i] = length(Rt_equ);
				u[i] = fmod(coe.w + coe.TA, PI2);
			}

			gamma = (r[0] - r[1]) / (r[0] - r[2]);
			u0 = atan2(sin(u[0]) - sin(u[1]) - gamma * (sin(u[0]) - sin(u[2])), gamma*(cos(u[2]) - cos(u[0])) - cos(u[1]) + cos(u[0]));

			ux = u0 + PI05;
			uy = u0 - PI05;

			du1 = fmod(ux - u[0], PI2);
			du2 = fmod(uy - u[0], PI2);

			dt1 = time_theta(R1, V1, du1, mu);
			if (dt1 < 0 && n == 0)
			{
				dt1 += T_p;
			}
			dt2 = time_theta(R1, V1, du2, mu);
			if (dt2 < 0 && n == 0)
			{
				dt2 += T_p;
			}

			oneclickcoast(R1, V1, MJD + dt_total / 24.0 / 3600.0, dt1, R11, V11);
			oneclickcoast(R1, V1, MJD + dt_total / 24.0 / 3600.0, dt2, R12, V12);

			if (length(R11) > length(R12))
			{
				dt = dt1;
				R1 = R11;
				V1 = V11;
			}
			else
			{
				dt = dt2;
				R1 = R12;
				V1 = V12;
			}

			dt_total += dt;

			dt = 10.0;

			do
			{
				oneclickcoast(R1, V1, MJD + dt_total / 24.0 / 3600.0, dt, R1, V1);
				dt_total += dt;
				vr = dotp(R1, V1) / length(R1);
				if (dt*vr < 0)
				{
					dt = -dt * 0.5;
				}

				n++;
			} while (abs(dt) > 0.01);
		}

		R2 = R1;
		V2 = V1;

		return dt_total;
	}

	double kepler_U_equation(double x, double ro, double vro, double a, double mu)
	{
		return (ro*vro / sqrt(mu)*x*x*stumpC(a*x*x) + (1.0 - a * ro)*x*x*x*stumpS(a*x*x) + ro * x) / sqrt(mu);
	}

	void REVUP(VECTOR3 R, VECTOR3 V, double n, double mu, VECTOR3 &R1, VECTOR3 &V1, double &t)
	{
		double a;

		a = 1.0 / (2.0 / length(R) - dotp(V, V) / mu);
		t = n * PI2*sqrt(power(a, 3.0) / mu);
		rv_from_r0v0(R, V, t, R1, V1, mu);
	}

	void RADUP(VECTOR3 R_W, VECTOR3 V_W, VECTOR3 R_C, double mu, VECTOR3 &R_W1, VECTOR3 &V_W1)
	{
		double theta, dt;

		theta = sign(dotp(crossp(R_W, R_C), crossp(R_W, V_W)))*acos(dotp(R_W / length(R_W), R_C / length(R_C)));
		dt = time_theta(R_W, V_W, theta, mu);
		rv_from_r0v0(R_W, V_W, dt, R_W1, V_W1, mu);
	}

	bool CSIToDH(VECTOR3 R_A1, VECTOR3 V_A1, VECTOR3 R_P2, VECTOR3 V_P2, double DH, double mu, double &dv)
	{
		int s_F;
		double c_I, tt, e_H, dvo, eps2, p_H, e_Ho;
		VECTOR3 u, R_A2, V_A2, V_A1F, R_PH2, V_PH2;

		p_H = c_I = 0.0;
		s_F = 0;
		eps2 = 1.0;

		u = unit(crossp(R_P2, V_P2));
		R_A1 = unit(R_A1 - u * dotp(R_A1, u))*length(R_A1);
		V_A1 = unit(V_A1 - u * dotp(V_A1, u))*length(V_A1);

		do
		{
			V_A1F = V_A1 + unit(crossp(u, R_A1))*dv;
			OrbMech::REVUP(R_A1, V_A1F, 0.5, mu, R_A2, V_A2, tt);
			//t_H2 = t_H1 + tt;
			OrbMech::RADUP(R_P2, V_P2, R_A2, mu, R_PH2, V_PH2);
			e_H = length(R_PH2) - length(R_A2) - DH;

			if (abs(e_H) >= eps2)
			{
				ITER(c_I, s_F, e_H, p_H, dv, e_Ho, dvo);
				if (s_F == 1)
				{
					return false;
				}
			}
		} while (abs(e_H) >= eps2);

		return true;
	}

	void ITER(double &c, int &s, double e, double &p, double &x, double &eo, double &xo, double dx0)
	{
		double dx;

		if (c == 0)
		{
			dx = dx0;
			c = c + 1.0;eo = e;xo = x;x = x - dx;
		}
		else if (c == 0.5)
		{
			dx = e / p;
			c = c + 1.0;eo = e;xo = x;x = x - dx;
		}
		else
		{
			if (e - eo == 0)
			{
				dx = 3.0*dx0;
				c = c + 1.0;eo = e;xo = x;x = x - dx;
			}
			else
			{
				p = (e - eo) / (x - xo);
				if (c > 20)
				{
					s = 1;
				}
				else
				{
					dx = e / p;
					c = c + 1.0;eo = e;xo = x;x = x - dx;
				}
			}
		}
	}

	CELEMENTS CELEMENTS::operator+(const CELEMENTS& c) const
	{
		CELEMENTS result;

		result.a = this->a + c.a;
		result.e = this->e + c.e;
		result.i = this->i + c.i;
		result.h = this->h + c.h;
		result.g = this->g + c.g;
		result.l = this->l + c.l;

		return result;
	}

	CELEMENTS CELEMENTS::operator-(const CELEMENTS& c) const
	{
		CELEMENTS result;

		result.a = this->a - c.a;
		result.e = this->e - c.e;
		result.i = this->i - c.i;
		result.h = this->h - c.h;
		result.g = this->g - c.g;
		result.l = this->l - c.l;

		return result;
	}

	CoastIntegrator::CoastIntegrator(VECTOR3 R00, VECTOR3 V00, double mjd0, double deltat)
	{
		hEarth = oapiGetObjectByName("Earth");

		K = 0.3;
		dt_lim = 4000;
		R_E = oapiGetSize(hEarth);
		mu = oapiGetMass(hEarth)*GGRAV;
		jcount = oapiGetPlanetJCoeffCount(hEarth);
		JCoeff = new double[jcount];
		for (int i = 0; i < jcount; i++)
		{
			JCoeff[i] = oapiGetPlanetJCoeff(hEarth, i);
		}

		this->R00 = R00;
		this->V00 = V00;
		this->mjd0 = mjd0;
		R0 = R00;
		V0 = V00;
		t_0 = 0;
		t = 0;
		tau = 0;
		t_F = t_0 + deltat;
		delta = _V(0, 0, 0);// [0 0 0]';
		nu = _V(0, 0, 0);// [0 0 0]';
		R_CON = R0;
		V_CON = V0;
		x = 0;
		rect1 = 0.75*OrbMech::power(2.0, 22.0);
		rect2 = 0.75*OrbMech::power(2.0, 3.0);

		MATRIX3 obli_E = OrbMech::GetObliquityMatrix(hEarth, mjd0);
		U_Z = mul(obli_E, _V(0, 1, 0));
		U_Z = _V(U_Z.x, U_Z.z, U_Z.y);
	}

	CoastIntegrator::~CoastIntegrator()
	{
		delete[] JCoeff;
	}

	bool CoastIntegrator::iteration()
	{
		double rr, dt_max, dt, h, x_apo, gamma, s, alpha_N, x_t, Y;
		VECTOR3 alpha, R_apo, V_apo, R, a_d, ff;
		VECTOR3 k[3];

		R = R_CON;
		rr = length(R_CON);

		dt_max = 0.3*min(dt_lim, K*OrbMech::power(rr, 1.5) / sqrt(mu));
		Y = OrbMech::sign(t_F - t);
		dt = Y * min(abs(t_F - t), dt_max);

		if (length(delta) / length(R_CON) > 0.01 || length(delta) > rect1 || length(nu) > rect2)
		{
			R0 = R_CON + delta;
			V0 = V_CON + nu;
			R_CON = R0;
			V_CON = V0;
			delta = _V(0, 0, 0);
			nu = _V(0, 0, 0);// [0 0 0]';
			x = 0;
			tau = 0;
		}
		h = 0;
		alpha = delta;
		R_apo = R_CON;
		V_apo = V_CON;
		x_apo = x;
		for (int j = 0; j < 3; j++)
		{
			R = R_CON + alpha;
			a_d = adfunc(R);
			ff = f(alpha, R, a_d);
			k[j] = ff;
			if (j < 2)
			{
				h = h + 0.5*dt;
				alpha = delta + (nu + ff * h*0.5)*h;
				t = t + 0.5*dt;
				tau = tau + 0.5*dt;
				s = sqrt(mu) / length(R_apo)*0.5*dt;
				gamma = dotp(R_apo, V_apo) / (length(R_apo)*sqrt(mu)*2.0);
				alpha_N = 2.0 / length(R0) - OrbMech::power(length(V0), 2.0) / mu;
				x_t = x_apo + s * (1.0 - gamma * s*(1.0 - 2.0 * gamma*s) - 1.0 / 6.0 * (1.0 / length(R_apo) - alpha_N)*s*s);
				OrbMech::rv_from_r0v0(R0, V0, tau, R_CON, V_CON, mu, x_t);
			}
		}
		delta = delta + (nu + (k[0] + k[1] * 2.0)*dt*1.0 / 6.0)*dt;
		nu = nu + (k[0] + k[1] * 4.0 + k[2]) * 1.0 / 6.0 *dt;

		if (abs(t - t_F) < 1e-6)
		{
			R2 = R_CON + delta;
			V2 = V_CON + nu;

			return true;
		}
		return false;
	}

	VECTOR3 CoastIntegrator::f(VECTOR3 alpha, VECTOR3 R, VECTOR3 a_d)
	{
		VECTOR3 R_CON;
		double q;

		R_CON = R - alpha;
		q = dotp((alpha - R * 2.0), alpha) / (OrbMech::power(length(R), 2.0));
		return -(R*fq(q) + alpha)*mu / OrbMech::power(length(R_CON), 3.0) + a_d;
	}

	double CoastIntegrator::fq(double q)
	{
		return q * (3.0 + 3.0 * q + q * q) / (1.0 + OrbMech::power(1 + q, 1.5));
	}

	VECTOR3 CoastIntegrator::adfunc(VECTOR3 R)
	{
		double r, costheta, P2, P3, P4, P5;
		VECTOR3 U_R, a_dP, a_d;
		a_dP = _V(0, 0, 0);
		r = length(R);
		U_R = unit(R);
		costheta = dotp(U_R, U_Z);
		P2 = 3.0 * costheta;
		P3 = 0.5*(15.0*costheta*costheta - 3.0);
		a_dP += (U_R*P3 - U_Z * P2)*JCoeff[0] * OrbMech::power(R_E / r, 2.0);
		if (jcount > 1)
		{
			P4 = 1.0 / 3.0*(7.0*costheta*P3 - 4.0*P2);
			a_dP += (U_R*P4 - U_Z * P3)*JCoeff[1] * OrbMech::power(R_E / r, 3.0);
			if (jcount > 2)
			{
				P5 = 0.25*(9.0*costheta*P4 - 5.0 * P3);
				a_dP += (U_R*P5 - U_Z * P4)*JCoeff[2] * OrbMech::power(R_E / r, 4.0);
			}
		}
		a_dP *= mu / OrbMech::power(r, 2.0);

		a_d = a_dP;

		return a_d;
	}

}