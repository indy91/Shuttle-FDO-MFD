/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Deorbit Opportunities Processor

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

#include <algorithm>
#include "DeorbitOpportunities.h"

LandingOpportunitiesProcessor::LandingOpportunitiesProcessor()
{

}

void LandingOpportunitiesProcessor::LOPT(const LOPTInput &in, LOPTOutput &out)
{
	double GMTS, GMTF, UC, RORB, TARB, AN[4], lng_min, lng_max, lng_asc, T_CA, T_TIG, T_LAND;
	VECTOR3 SVEC, H, N, CVEC, R_SUN;
	double lat_S, lng_S, EN, GN, T, TN, SNANG, HC, UCPA, DTN, DT, GCPA, TAC, EAC, T_OLD, XRNG, SLAT, SLON, DM;
	double DELR, DELC, DTSS, DTSR, DANG;
	int i, numrevs, IFL;
	LOPTOutputDataSet temp;

	opt = in;
	out.data.clear();

	//Convert crossrange to angle
	ANG = opt.XRNG *1852.0 / OrbMech::R_Earth;
	GMTS = opt.GETS + opt.GMTR;
	GMTF = opt.GETF + opt.GMTR;

	//Generate a table which contains the times and longitudes of the ascending nodes for all orbits
	//Get state vector at beginning time
	sv_cur = UPDATE(opt.sv_in, GMTS - opt.sv_in.GMT);
	elem_cur = OrbMech::CalculateInvariantElementsBlock(sv_cur, OrbMech::mu_Earth, 0.0, opt.SVPROP);
	//Get ascending node crossing before start time
	UC = ADVU(elem_cur.TIMV);
	i = 0;
	TAU(elem_cur.TIMV, UC, 0.0, RORB, TARB);
	//Store
	table[i][0] = TARB;
	table[i][1] = elem_cur.h + elem_cur.h_dot*(TARB - elem_cur.TIMV) - OrbMech::w_Earth*TARB;
	table[i][1] = OrbMech::normalize_angle(table[i][1], 0.0, PI2);
	while (TARB < GMTF && i < 47)
	{
		i++;
		TAU(TARB, 0.0, PI2, RORB, TARB);
		//Store
		table[i][0] = TARB;
		table[i][1] = elem_cur.h + elem_cur.h_dot*(TARB - elem_cur.TIMV) - OrbMech::w_Earth*TARB;
		table[i][1] = OrbMech::normalize_angle(table[i][1], 0.0, PI2);
	}
	numrevs = i;
	//For each rev
	for (unsigned i = 0;i < opt.sites.size();i++)
	{
		lat_S = opt.sites[i].lat;
		lng_S = opt.sites[i].lng;
		CNODS(lat_S, lng_S, ANG, 600.0, AN, IFL);

		if (IFL == -1) continue; //No opportunities, try next site

		for (int j = 0;j <= numrevs;j++)
		{
			for (int k = 0;k < IFL;k++)
			{
				lng_min = AN[k * 2];
				lng_max = AN[k * 2 + 1];
				lng_asc = table[j][1];
				//Overlaps 0° longitude, needs this special logic
				if (lng_max < lng_min)
				{
					lng_max += PI2;
					lng_asc += PI2;
				}

				if (lng_min < lng_asc && lng_max > lng_asc)
				{
					//Found one
					T = table[j][0];
					//Calculate some numbers at ascending node crossing
					GN = elem_cur.g + elem_cur.g_dot*(T - elem_cur.TIMV);
					TN = PI2 - GN;
					EN = 2.0*atan((1.0 - elem_cur.e) / (1.0 + elem_cur.e)*tan(TN / 2.0));
					EN = OrbMech::normalize_angle(EN, 0.0, PI2);

					//Calculate time of closest approach
					do
					{
						SNANG = lng_S + OrbMech::w_Earth*T;
						HC = elem_cur.h + elem_cur.h_dot*(T - elem_cur.TIMV);
						SVEC = _V(cos(SNANG)*cos(lat_S), sin(SNANG)*cos(lat_S), sin(lat_S));
						H = _V(sin(HC)*sin(elem_cur.i), -cos(HC)*sin(elem_cur.i), cos(elem_cur.i));
						//Ascending node vector
						N = _V(cos(HC), sin(HC), 0.0);
						//Position unit vector at closest approach
						CVEC = unit(crossp(H, crossp(SVEC, H)));
						//Argument of latitude at closest approach
						UCPA = acos(dotp(CVEC, N));
						//If closest aproach is in southern hemisphere (z-component of CVEC is negative) then argument of latitude is in range PI to PI2
						if (CVEC.z < 0.0)
						{
							UCPA = PI2 - UCPA;
						}
						//Approximate time from ascending node to closest approach
						DTN = UCPA / elem_cur.l_dot;
						//Adjust argument of perigee for apsidal precession in that timespan
						GCPA = GN + DTN * elem_cur.g_dot;
						//True anomaly at closest approach
						TAC = UCPA - GCPA;
						//Eccentric anomaly at closest approach
						EAC = 2.0*atan((1.0 - elem_cur.e) / (1.0 + elem_cur.e)*tan(TAC / 2.0));
						//Calculate the difference in mean anomalies at ascending node and at closest approach
						DM = ((EAC - elem_cur.e*sin(EAC)) - (EN - elem_cur.e*sin(EN)));
						//Force positive angle from ascending node
						if (DM < 0)
						{
							DM += PI2;
						}
						//Use difference in mean anomalies to correct the delta time
						DT = DM / elem_cur.l_dot;
						//Update times
						T_OLD = T;
						T = table[j][0] + DT;
					} while (abs(T - T_OLD) > opt.RTOL);

					//Save time of closest approach
					T_CA = T;
					
					//Calculate crossrange as an angle
					XRNG = asin(dotp(SVEC, H));

					//Check against crossrange angle constraint
					if (abs(XRNG) < ANG)
					{
						//Calculate time of ignition and landing from curve fits
						T_TIG = CalculateTIG(T_CA, elem_cur.a);
						T_LAND = CalculateLandingTime(T_CA, elem_cur.a);

						//Begin storing data
						temp.Site = opt.sites[i].name;
						temp.Rev = opt.INORB + j;
						temp.TIG_MET = T_TIG - opt.GMTR;
						temp.Landing_MET = T_LAND - opt.GMTR;
						temp.Landing_GMT = T_LAND;

						char Buffer[16];
						char Ascending;
						char Left;

						if (UCPA > PI05 && UCPA < PI05*3.0)
						{
							Ascending = 'D';
						}
						else
						{
							Ascending = 'A';
						}
						if (XRNG > 0)
						{
							Left = 'L';
						}
						else
						{
							Left = 'R';
						}

						sprintf(Buffer, "%.0f%c%c", abs(XRNG)*OrbMech::R_Earth / 1852.0, Ascending, Left);
						temp.XRNG.assign(Buffer);

						//Calculate local sun angle
						R_SUN = OrbMech::SUN(opt.BaseMJD + T_LAND / 24.0 / 3600.0, opt.RM);
						OrbMech::latlong_from_r(R_SUN, SLAT, SLON);
						SLON -= T_LAND * OrbMech::w_Earth;
						SLON = OrbMech::normalize_angle(SLON, 0.0, PI2);

						//Calculate local sunrise/sunset times
						DELR = acos((cos(opt.ANGZ) - sin(lat_S)*sin(SLAT)) / cos(lat_S) / cos(SLAT));
						DELC = SLON - lng_S;
						DANG = DELR - DELC;
						DANG = OrbMech::normalize_angle(DANG, -PI, PI);
						DTSR = DANG / OrbMech::w_Earth;
						DANG = DELR + DELC;
						DANG = OrbMech::normalize_angle(DANG, -PI, PI);
						DTSS = DANG / OrbMech::w_Earth;

						double dTemp, ss;
						int hh, mm;
						char AfterSun, Rise;

						//Show closest sunrise or sunset
						if (abs(DTSR) > abs(DTSS))
						{
							dTemp = DTSS;
							Rise = 'S';
							if (dTemp < 0.0)
							{
								AfterSun = 'A';
							}
							else
							{
								AfterSun = 'B';
							}
						}
						else
						{
							dTemp = DTSR;
							Rise = 'R';
							if (dTemp > 0.0)
							{
								AfterSun = 'A';
							}
							else
							{
								AfterSun = 'B';
							}
						}

						OrbMech::days2hms(abs(dTemp) / 24.0 / 3600.0, hh, mm, ss);
						sprintf(Buffer, "%d:%02d%c%c", hh, mm, AfterSun, Rise);
						temp.T_Light.assign(Buffer);

						out.data.push_back(temp);
					}
				}
			}
		}
	}

	//Sort
	std::sort(out.data.begin(), out.data.end());
}

double LandingOpportunitiesProcessor::ADVU(double TIME)
{
	double M, G, F, U;

	M = elem_cur.l + elem_cur.l_dot*(TIME - elem_cur.TIMV);
	G = elem_cur.g + elem_cur.g_dot*(TIME - elem_cur.TIMV);
	F = OrbMech::MeanToTrueAnomaly(M, elem_cur.e);
	U = G + F;
	if (U < 0)
	{
		U += PI2;
	}
	else if (U >= PI2)
	{
		U -= PI2;
	}
	return U;
}

void LandingOpportunitiesProcessor::TAU(double TIMC, double UC, double UT, double &RORB, double &TARB)
{
	double dt, g_p, F, E, M_p, DM, ddt, dt_new, M_c, g_c, DU;

	dt = TIMC - elem_cur.TIMV;
	M_c = elem_cur.l + elem_cur.l_dot*dt;
	M_c = OrbMech::normalize_angle(M_c, 0.0, PI2);
	g_c = elem_cur.g + elem_cur.g_dot*dt;
	g_c = OrbMech::normalize_angle(g_c, 0.0, PI2);

	//Compute first guess delta t
	ddt = 100.0;
	DU = UT - UC;
	dt = DU / (elem_cur.l_dot + elem_cur.g_dot);
	while (abs(ddt) > 0.1)
	{
		//Compute new argument of perigee
		g_p = g_c + elem_cur.g_dot*dt;
		g_p = OrbMech::normalize_angle(g_p, 0.0, PI2);
		//Compute new true anomaly
		F = UT - g_p;
		F = OrbMech::normalize_angle(F, 0.0, PI2);
		//Compute new eccentric anomaly
		E = 2.0*atan((1.0 - elem_cur.e) / (1.0 + elem_cur.e)*tan(F / 2.0));
		E = OrbMech::normalize_angle(E, 0.0, PI2);
		//Compute new mean anomaly
		M_p = E - elem_cur.e*sin(E);
		//Compute delta mean anomaly
		DM = M_p - M_c;
		//Is the sign of the delta mean anomaly not the same as the sign of (UT-UC)?
		while (DU - DM > PI)
		{
			DM += PI2;
		}
		while (DU - DM < -PI)
		{
			DM -= PI2;
		}

		//Compute new delta t
		dt_new = DM / elem_cur.l_dot;
		ddt = dt_new - dt;
		dt = dt_new;
	}
	RORB = elem_cur.a*(1.0 - elem_cur.e) / (1.0 + elem_cur.e*cos(F));
	TARB = TIMC + dt;
}

void LandingOpportunitiesProcessor::CNODS(double lat_S, double lng_S, double ANG, double ABIAS, double *AN, int &IFL)
{
	//IFL: -1 = no solution, 1 = one range, 2 = two ranges
	double sin_dlng_p, sin_dlng_m, dlng_p, dlng_m, dlng_min[2], dlng_max[2], TEM[2], DT[2];

	//Compute sin(dlng) terms with + and - signs
	sin_dlng_p = (sin(lat_S)*cos(elem_cur.i) + sin(ANG)) / sin(elem_cur.i) / cos(lat_S);
	sin_dlng_m = (sin(lat_S)*cos(elem_cur.i) - sin(ANG)) / sin(elem_cur.i) / cos(lat_S);
	if (sin_dlng_p < 1.0)
	{
		if (sin_dlng_m < 1.0)
		{
			//1/2
			dlng_p = asin(sin_dlng_p);
			dlng_m = asin(sin_dlng_m);
			dlng_min[0] = dlng_p;
			dlng_max[0] = dlng_m;
			dlng_min[1] = -dlng_max[0] + PI;
			dlng_max[1] = -dlng_min[0] - PI;
			IFL = 2;
		}
		else
		{
			//2/2
			dlng_p = asin(sin_dlng_p);
			dlng_max[0] = dlng_p;
			dlng_min[0] = -dlng_p - PI;
			IFL = 1;
		}
	}
	else
	{
		if (sin_dlng_m < 1.0)
		{
			//3/2
			dlng_m = asin(sin_dlng_m);
			dlng_max[0] = dlng_m;
			dlng_min[0] = -dlng_m - PI;
			IFL = 1;
		}
		else
		{
			IFL = -1;
			return;
		}
	}

	//Compute in-plane angles from the node to CPM(?) for dlng_min[0] and dlng_max[0]
	TEM[0] = acos(cos(lat_S)*cos(dlng_min[0]) / cos(ANG));
	TEM[1] = acos(cos(lat_S)*cos(dlng_max[0]) / cos(ANG));
	//Compute approximate flight times for dlng_min[0] and dlng_max[0]
	DT[0] = TEM[0] / elem_cur.l_dot;
	DT[1] = TEM[1] / elem_cur.l_dot;
	//Compute longitude of nodes lng_min[0] and lng_max[0]
	AN[0] = lng_S - dlng_min[0] + DT[0] * OrbMech::w_Earth;
	AN[0] = OrbMech::normalize_angle(AN[0], 0.0, PI2);
	AN[1] = lng_S - dlng_max[0] + DT[1] * OrbMech::w_Earth;
	AN[1] = OrbMech::normalize_angle(AN[1], 0.0, PI2);

	if (IFL == 2)
	{
		//Compute in-plane angles from the node to CPM(?) for dlng_min[1] and dlng_max[1]
		TEM[0] = acos(cos(lat_S)*cos(dlng_min[1]) / cos(ANG));
		TEM[1] = acos(cos(lat_S)*cos(dlng_max[1]) / cos(ANG));
		//Compute approximate flight times for dlng_min[0] and dlng_max[0]
		DT[0] = TEM[0] / elem_cur.l_dot;
		DT[1] = TEM[1] / elem_cur.l_dot;
		//Compute longitude of nodes lng_min[0] and lng_max[0]
		AN[2] = lng_S - dlng_min[1] + DT[0] * OrbMech::w_Earth;
		AN[2] = OrbMech::normalize_angle(AN[2], 0.0, PI2);
		AN[3] = lng_S - dlng_max[1] + DT[1] * OrbMech::w_Earth;
		AN[3] = OrbMech::normalize_angle(AN[3], 0.0, PI2);
	}

	//TBD: Apply ABIAS
}

OrbMech::SV LandingOpportunitiesProcessor::UPDATE(OrbMech::SV sv0, double dt)
{
	if (opt.SVPROP)
	{
		return coast(sv0, dt);
	}
	else
	{
		return coast_osc(sv0, dt, OrbMech::mu_Earth);
	}
}

double LandingOpportunitiesProcessor::CalculateTIG(double T_CA, double R)
{
	double dt;

	R = R / 0.3048;
	//dt = 3.8954375e5 - 3.42471e-2*R + 7.44588e-10*R*R; //From Crew Return Vehicle (CRV) Deorbit Opportunities
	dt = 2.9660e5 -2.6181e-02*R + 5.7111e-10 *R*R; //This works much better with the Shuttle
	return T_CA + dt;
}

double LandingOpportunitiesProcessor::CalculateLandingTime(double T_CA, double R)
{
	double dt;

	R = R / 0.3048;
	dt = 1.40702e3 + 1.287e-4*R - 6.45541e-12*R*R; //From Crew Return Vehicle (CRV) Deorbit Opportunities
	return T_CA + dt;
}