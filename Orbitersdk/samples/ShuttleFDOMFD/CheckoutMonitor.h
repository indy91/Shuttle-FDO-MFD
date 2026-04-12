/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Checkout Monitor Display (Header)

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

struct CheckoutMonitorDisplay
{
	std::string GMT;
	std::string MET;
	std::string M50_POS_FT[3];
	std::string M50_VEL_FPS[3];
	std::string M50_POS_M[3];
	std::string M50_VEL_MPS[3];
	std::string HA;
	std::string MET_HA;
	std::string HP;
	std::string MET_HP;
	std::string V_I;
	std::string V_REL;
	std::string GAMMA;
	std::string PSI_REL;
	std::string PSI_TEI;
	std::string PHI_C[2];
	std::string PHI_D[2];
	std::string LAMBDA[2];
	std::string h_s;
	std::string h_o[2];
	std::string R;
	std::string T_an;
	std::string lambda_an;
	std::string BETA_ANG;
	std::string PERIOD;
	std::string RA_M50;
	std::string DEC_M50;
	std::string REF_DAY_D;
	std::string REF_DAY_M;
	std::string REF_DAY_Y;
	std::string LO;
	std::string A;
	std::string E;
	std::string I_M50;
	std::string I_TEI;
	std::string WP_M50;
	std::string WP_TEI;
	std::string RAAN_M50;
	std::string N;
	std::string M;
};

class CheckoutMonitor
{
public:
	CheckoutMonitor(OrbMech::SessionConstants& scnst);

	void RUN(const OrbMech::SV &sv, bool useNonSphericalGravity, CheckoutMonitorDisplay &disp);

protected:

	// Output formatting
	std::string FormatString(const char* _Format, double Val);
	std::string FormatString(const char* _Format, int Val);
	std::string FormatLatitude(double lat);
	std::string FormatLongitude(double lng);
	std::string FormatTime(double MET);
	std::string FormatTime_DDDHHSSSSS(double time, int day);

	// Temporary variables
	char Buffer[128];

	OrbMech::SessionConstants& sesconst;
};