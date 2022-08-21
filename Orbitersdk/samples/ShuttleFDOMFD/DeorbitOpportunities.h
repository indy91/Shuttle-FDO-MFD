/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Deorbit Opportunities Processor (Header)

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

#include <string>
#include <vector>
#include "OrbMech.h"

struct LOPTSite
{
	double lat = 0.0;
	double lng = 0.0;
	double rad = 0.0;
	int timezone = 0;
	std::string name;
};

struct LOPTInput
{
	//Input state vector in TEG coordinates
	OrbMech::SV sv_in;
	//Reference time for GET computations
	double GMTR;
	//Start and end time for search in GET
	double GETS, GETF;
	//Initial orbit counter, corresponds to the orbit counter for GETS
	int INORB;
	//false = conic, true = integrated
	bool SVPROP;
	std::vector<LOPTSite> sites;
	//Base MJD
	double BaseMJD;
	//Rotation matrix from desired system to ecliptic
	MATRIX3 RM;

	//Constants
	//Crossrange constraint (NM)
	double XRNG = 800.0;
	//Angle between zenith and local sunrise/sunset
	double ANGZ = 1.5853407;
	//Iteration tolerance for CPA iteration
	double RTOL = 2.0;
};

struct LOPTOutputDataSet
{
	//Orbit Number
	int Rev = 0;
	//Time of ignition
	double TIG_MET = 0.0;
	//Time of landing in MET
	double Landing_MET = 0.0;
	//Time of landing GMT
	double Landing_GMT = 0.0;
	//Formated code for light condition. Time in H:MM, a for after or b for before, r for sunrise, s for sunset
	std::string T_Light;
	//Formated code for crossrange: Crossrange on nautical miles, A or D for ascending descending, L or R for left or right of landing site
	std::string XRNG;
	//Name of landing site
	std::string Site;

	//For sorting
	bool operator<(const LOPTOutputDataSet& rhs) const
	{
		return TIG_MET < rhs.TIG_MET;
	}
};

struct LOPTOutput
{
	std::vector<LOPTOutputDataSet> data;
};

//Based on landing opportunities processor (78-FM-18, Volume III, Book 2)
class LandingOpportunitiesProcessor
{
public:
	LandingOpportunitiesProcessor();
	void LOPT(const LOPTInput &in, LOPTOutput &out);
protected:
	OrbMech::SV UPDATE(OrbMech::SV sv0, double dt);
	double ADVU(double TIME);
	void TAU(double TIMC, double UC, double UT, double &RORB, double &TARB);
	void CNODS(double lat_S, double lng_S, double ANG, double ABIAS, double *AN, int &IFL);
	double CalculateTIG(double T_CA, double R);
	double CalculateLandingTime(double T_CA, double R);

	LOPTInput opt;

	//Crossrange constraint angle
	double ANG;
	//Current state vector
	OrbMech::SV sv_cur;
	OrbMech::InvariantElements elem_cur;
	double table[48][2];
};