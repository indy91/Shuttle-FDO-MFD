/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Checkout Monitor Display

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

#include "CheckoutMonitor.h"

CheckoutMonitor::CheckoutMonitor(OrbMech::SessionConstants& scnst) : sesconst(scnst)
{
	sprintf_s(Buffer, "");
}

void CheckoutMonitor::RUN(const OrbMech::SV& sv, bool useNonSphericalGravity, CheckoutMonitorDisplay& disp)
{
	MATRIX3 M_TEG_EF;
	VECTOR3 R_TEG, V_TEG, R_M50, V_M50, u_SUN_TEG, V_REL, R_EF, V_EF, H_TEG;
	double apo, peri, r, v, lat, lng, gamma, azi, r_rel, v_rel, lat_rel, lng_rel, gamma_rel, azi_rel, HA, HP, HS, HO, T_P, beta, DEC_M50, RA_M50, TA;
	OrbMech::CELEMENTS elem_TEG, elem_M50;

	// Calculations
	R_TEG = sv.R;
	V_TEG = sv.V;
	M_TEG_EF = OrbMech::TEG_to_EF_Matrix(OrbMech::w_Earth, sv.GMT);
	R_M50 = mul(sesconst.M_TEG_TO_M50, sv.R);
	V_M50 = mul(sesconst.M_TEG_TO_M50, sv.V);
	R_EF = mul(M_TEG_EF, sv.R);
	V_EF = mul(M_TEG_EF, sv.V);
	H_TEG = unit(crossp(sv.R, sv.V));

	if (useNonSphericalGravity)
	{
		OrbMech::ApsidesMagnitudeDetermination(sv, apo, peri);
	}
	else
	{
		OrbMech::periapo(sv.R, sv.V, OrbMech::mu_Earth, apo, peri);
	}
	HA = apo - OrbMech::EARTH_RADIUS_EQUATOR;
	HP = peri - OrbMech::EARTH_RADIUS_EQUATOR;
	u_SUN_TEG = unit(OrbMech::SUN(sesconst.GMTBASE, sv.GMT, sesconst.M_TEG_TO_M50));
	V_REL = sv.V - crossp(_V(0, 0, 1), sv.R) * OrbMech::w_Earth;
	OrbMech::PICSSC(true, R_EF, V_EF, r, v, lat, lng, gamma, azi);
	OrbMech::PICSSC(true, R_TEG, V_REL, r_rel, v_rel, lat_rel, lng_rel, gamma_rel, azi_rel);
	HO = r - OrbMech::EARTH_RADIUS_ORBITER; // Should be height above oblate Earth, for now height above Orbiter radius
	HS = r - OrbMech::EARTH_RADIUS_EQUATOR;
	beta = asin(dotp(H_TEG, u_SUN_TEG));
	T_P = OrbMech::REVTIM(sv.R, sv.V, useNonSphericalGravity);
	OrbMech::latlong_from_r(R_M50, DEC_M50, RA_M50);
	if (RA_M50 < 0.0) RA_M50 += PI2;
	elem_TEG = OrbMech::CartesianToKeplerian(R_TEG, V_TEG, OrbMech::mu_Earth);
	elem_M50 = OrbMech::CartesianToKeplerian(R_M50, V_M50, OrbMech::mu_Earth);
	TA = OrbMech::MeanToTrueAnomaly(elem_TEG.l, elem_TEG.e);

	// Output formatting

	disp.GMT = FormatTime_DDDHHSSSSS(sv.GMT, sesconst.DayOfYear);
	disp.MET = FormatTime_DDDHHSSSSS(sv.GMT - sesconst.GMTLO, 0);
	disp.M50_POS_FT[0] = FormatString("%.1lf", R_M50.x / OrbMech::FPS2MPS);
	disp.M50_POS_FT[1] = FormatString("%.1lf", R_M50.y / OrbMech::FPS2MPS);
	disp.M50_POS_FT[2] = FormatString("%.1lf", R_M50.z / OrbMech::FPS2MPS);
	disp.M50_VEL_FPS[0] = FormatString("%.6lf", V_M50.x / OrbMech::FPS2MPS);
	disp.M50_VEL_FPS[1] = FormatString("%.6lf", V_M50.y / OrbMech::FPS2MPS);
	disp.M50_VEL_FPS[2] = FormatString("%.6lf", V_M50.z / OrbMech::FPS2MPS);
	disp.M50_POS_M[0] = FormatString("%.1lf", R_M50.x);
	disp.M50_POS_M[1] = FormatString("%.1lf", R_M50.y);
	disp.M50_POS_M[2] = FormatString("%.1lf", R_M50.z);
	disp.M50_VEL_MPS[0] = FormatString("%.6lf", V_M50.x);
	disp.M50_VEL_MPS[1] = FormatString("%.6lf", V_M50.y);
	disp.M50_VEL_MPS[2] = FormatString("%.6lf", V_M50.z);
	// TBD: METHA, METHP
	disp.HA = FormatString("%.3lf", HA / OrbMech::NM2M);
	disp.HP = FormatString("%.3lf", HP / OrbMech::NM2M);
	disp.V_I = FormatString("%.4lf", v / OrbMech::FPS2MPS);
	disp.V_REL = FormatString("%.4lf", v_rel / OrbMech::FPS2MPS);
	disp.GAMMA = FormatString("%.6lf", gamma * DEG);
	disp.PSI_REL = FormatString("%.6lf", azi_rel * DEG);
	disp.PSI_TEI = FormatString("%.6lf", azi * DEG); // TBD: This is really TEG, not TEI
	disp.PHI_C[0] = FormatLatitude(lat);
	disp.PHI_C[1] = FormatString("%.6lf", lat * DEG);
	disp.PHI_D[0] = FormatLatitude(lat);
	disp.PHI_D[1] = FormatString("%.6lf", lat * DEG);
	disp.LAMBDA[0] = FormatLongitude(lng);
	disp.LAMBDA[1] = FormatString("%.6lf", lng * DEG);
	disp.h_s = FormatString("%.5lf", HS / OrbMech::NM2M);
	disp.h_o[0] = FormatString("%.5lf", HO / OrbMech::NM2M);
	disp.h_o[1] = FormatString("%.2lf", HO / OrbMech::FPS2MPS);
	disp.R = FormatString("%.5lf", r / OrbMech::NM2M);
	// TBD: T_an, lambda_an
	disp.BETA_ANG = FormatString("%.3lf", beta * DEG);
	disp.PERIOD = FormatTime(T_P);
	disp.RA_M50 = FormatString("%.4lf", RA_M50 * DEG);
	disp.DEC_M50 = FormatString("%.4lf", DEC_M50 * DEG);
	// TBD: Inputs etc.
	disp.REF_DAY_D = FormatString("%d", sesconst.Day);
	disp.REF_DAY_M = FormatString("%d", sesconst.Month);
	disp.REF_DAY_Y = FormatString("%d", sesconst.Year);

	OrbMech::GMT2String(Buffer,sesconst.GMTLO,sesconst.DayOfYear);
	disp.LO.assign(Buffer);

	disp.A = FormatString("%.4lf", elem_TEG.a / OrbMech::NM2M);
	disp.E = FormatString("%.6lf", elem_TEG.e);
	disp.I_M50 = FormatString("%.5lf", elem_M50.i * DEG);
	disp.I_TEI = FormatString("%.5lf", elem_TEG.i * DEG);
	disp.WP_M50 = FormatString("%.5lf", elem_M50.g * DEG);
	disp.WP_TEI = FormatString("%.5lf", elem_TEG.g * DEG);
	disp.RAAN_M50 = FormatString("%.5lf", elem_M50.h * DEG);
	disp.WP_M50 = FormatString("%.5lf", elem_M50.g * DEG);
	disp.N = FormatString("%.5lf", TA * DEG);
	disp.M = FormatString("%.5lf", elem_TEG.l * DEG);
}

std::string CheckoutMonitor::FormatString(const char* _Format, double Val)
{
	std::string str;

	sprintf_s(Buffer, _Format, Val);
	str.assign(Buffer);
	return str;
}

std::string CheckoutMonitor::FormatString(const char* _Format, int Val)
{
	std::string str;

	sprintf_s(Buffer, _Format, Val);
	str.assign(Buffer);
	return str;
}

std::string CheckoutMonitor::FormatLatitude(double lat)
{
	// Input in radians
	std::string str;
	double lat2;

	// Convert to arc seconds and round
	lat2 = abs(round(lat * DEG * 3600.0));

	str = FormatTime(lat2);
	if (lat >= 0.0)
	{
		str.append("N");
	}
	else
	{
		str.append("S");
	}
	
	return str;
}

std::string CheckoutMonitor::FormatLongitude(double lng)
{
	// Input in radians
	std::string str;
	double lng2;

	// Convert to arc seconds and round
	lng2 = abs(round(lng * DEG * 3600.0));

	str = FormatTime(lng2);
	if (lng >= 0.0)
	{
		str.append("E");
	}
	else
	{
		str.append("W");
	}

	return str;
}

std::string CheckoutMonitor::FormatTime(double MET)
{
	OrbMech::MET2String2(Buffer, MET);
	return std::string(Buffer);
}

std::string CheckoutMonitor::FormatTime_DDDHHSSSSS(double time, int day)
{
	OrbMech::GMT2String2(Buffer, time, day);
	return std::string(Buffer);
}