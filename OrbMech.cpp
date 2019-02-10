/****************************************************************************
  This file is part of OMP MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  OMP MFD Orbital Mechanics Calculations

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

	double kepler_E(double e, double M)
	{
		double error2, ratio, E;
		//{
		//	This function uses Newton's method to solve Kepler's
		//		equation E - e*sin(E) = M for the eccentric anomaly,
		//		given the eccentricity and the mean anomaly.
		//		E - eccentric anomaly(radians)
		//		e - eccentricity, passed from the calling program
		//		M - mean anomaly(radians), passed from the calling program
		//}
		// ----------------------------------------------

		error2 = 1.e-8;
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
			if (future && dE < 0) dE += PI2;

			dt = g + sqrt(power(a, 3.0) / mu)*(dE - sin_dE);
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

	double MJDToDate(double MJD)
	{
		double fSimGMT = (fmod(MJD - 41317, 365))*86400.0; //MJD 40952 == Jan. 1, 1970, 00:00:00
		int Days = (int)(MJD - 41317.0);
		int leap_days = Days / 1460;
		fSimGMT -= leap_days * 86400.0; //compensate for leap years
		return fSimGMT;
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
				if (c > 15)
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