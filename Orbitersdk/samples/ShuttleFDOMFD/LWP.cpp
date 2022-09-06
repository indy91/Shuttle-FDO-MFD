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

#include "Orbitersdk.h"
#include "LWP.h"

LWPSettings::LWPSettings()
{
	LAZCOE[0] = 490.931*RAD;
	LAZCOE[1] = -20.131;
	LAZCOE[2] = -1.2501;
	LAZCOE[3] = 0.0975*DEG;

	DELNOF = true;
	NS = 0;
	SVPROP = 0;
	DTOPT = 0.0;
	WRAP = 0;
	NEGTIV = 0;
	GAMINS = 0.0;
	LATLS = 0.0;
	LONGLS = 0.0;
	PFT = 0.0;
	PFA = 0.0;
	RINS = 0.0;
	VINS = 0.0;
	YSMAX = 0.0;
	LOT = 0;
	LW = 0;
	TPLANE = 0.0;
	TSTART = 0.0;
	TEND = 0.0;
	DELNO = 0.0;
	OFFSET = 0.0;
	BIAS = 0.0;
	DTGRR = 0.0;
	TRANS = 0.0;
	INSCO = 1;
	GMTLOR = 0.0;
}


LaunchWindowProcessor::LaunchWindowProcessor()
{
	CMAX = 20;
	ANOM = 0.0;
	GMTLO = 0.0;
	DET = 1.0;
	DELH = 250.0*0.3048;
	DELNOF = true;
	DELNOD = 0.0;
	TSTEP = 0.0;
	DX1 = 25.0;
	DVTOL = 0.1*0.3048;
	PAO = 0.0;
	PAC = 0.0;
	OPEN = 0.0;
	CLOSE = 0.0;
	GMTINS = 0.0;
	WEDGE = 0.0;
	LPT = 1;
	IIGM = 0.0;
	AZL = 0.0;
	TIGM = 0.0;
	TDIGM = 0.0;
	DN = 0.0;
	PA = 0.0;
	STABLE = 2;
	error = 0;
}

void LaunchWindowProcessor::Init(LWPSettings &set)
{
	inp = set;

	//For now
	TSTEP = (inp.TEND - inp.TSTART)/2.0;
	
	RT = set.TRGVEC.R;
	VT = set.TRGVEC.V;
	TT = set.TRGVEC.GMT;

	lwp_param_table = set.lwp_param_table;
	error = 0;
}

void LaunchWindowProcessor::LWP()
{
	//Call LWPIN
	if (inp.LW != 1)
	{
		LWT();
		if (error) return;
		LWDSP();
		if (LPT > 0)
		{
			LWPT();
		}
	}
	if (inp.LW != 0)
	{
		RLOT();
		if (error) return;
		RLOTD();
		LWPOT();
	}
}

void LaunchWindowProcessor::UPDAT(VECTOR3 &R, VECTOR3 &V, double &T, double TF)
{
	OrbMech::SV sv0, sv1;
	double dt;

	sv0.R = R;
	sv0.V = V;
	sv0.GMT = T;
	sv0.mass = 1.0; //TBD
	dt = TF - T;

	if (inp.SVPROP == 1)
	{
		sv1 = OrbMech::coast(sv0, dt);
	}
	else
	{
		sv1 = OrbMech::coast_osc(sv0, dt, OrbMech::mu_Earth);
	}
	
	R = sv1.R;
	V = sv1.V;
	T = sv1.GMT;
}

void LaunchWindowProcessor::LENSR(double GMTLO)
{
	OrbMech::OELEMENTS coe;
	MATRIX3 MATR1, MATR2, MATR3;
	VECTOR3 URLS, H, K, J;
	double psi, Gamma, DYAW, LONG;

	GMTINS = GMTLO + inp.PFT;

	UPDAT(RT, VT, TT, GMTINS);

	LONG = inp.LONGLS + OrbMech::w_Earth * GMTLO;
	URLS = _V(cos(inp.LATLS)*cos(LONG), cos(inp.LATLS)*sin(LONG), sin(inp.LATLS));
	H = unit(crossp(RT, VT));
	K = unit(crossp(H, URLS));
	J = crossp(K, H);

	psi = asin(dotp(H, URLS));
	WEDGE = asin(sin(psi) / cos(inp.PFA));
	Gamma = asin(sin(inp.PFA) / cos(psi));
	DYAW = abs(WEDGE) - inp.YSMAX;
	if (inp.YSMAX > 0.)
	{
		if (DYAW <= 0.)
		{
			Gamma = acos(cos(inp.PFA) / cos(psi));
			WEDGE = 0.;
		}
		else
		{
			double cosx, x, y, sinG;

			cosx = cos(inp.PFA) / cos(inp.YSMAX);
			x = acos(cosx);
			y = asin(sin(psi) / cosx);
			WEDGE = abs(y) - inp.YSMAX;
			sinG = sin(x) / cos(psi);
			Gamma = acos(sqrt(1.0 - sinG * sinG));
		}
	}
	MATR1 = _M(J.x, J.y, J.z, K.x, K.y, K.z, H.x, H.y, H.z);
	MATR2 = mul(_M(cos(Gamma), sin(Gamma), 0., -sin(Gamma), cos(Gamma), 0., 0., 0., 1.), MATR1);
	if (DYAW > 0.)
	{
		double phi;
		phi = psi / abs(psi)*WEDGE;
		MATR2 = mul(_M(cos(phi), 0., sin(phi), 0., 1., 0., -sin(phi), 0., cos(phi)), MATR2);
	}
	MATR3 = mul(_M(cos(inp.GAMINS), -sin(inp.GAMINS), 0., sin(inp.GAMINS), cos(inp.GAMINS), 0., 0., 0., 1.), MATR2);
	RP = _V(MATR2.m11, MATR2.m12, MATR2.m13)*inp.RINS;
	VP = _V(MATR3.m21, MATR3.m22, MATR3.m23)*inp.VINS;
	TP = GMTINS;
}

void LaunchWindowProcessor::NPLAN(double &TIP)
{
	OrbMech::OELEMENTS coe;
	double I_m, U_T, eta, lambda_N, LAMBDA, LONG, DLON, DOS, dt;
	int ITER;

	ITER = 1;
	DOS = 2.0e-4;

	do
	{
		//Update target to insertion
		UPDAT(RT, VT, TT, TIP);

		coe = OrbMech::coe_from_sv(RT, VT, OrbMech::mu_Earth);
		I_m = coe.i;
		lambda_N = coe.RA;
		U_T = OrbMech::asin2(sin(inp.LATLS) / sin(I_m));
		if (inp.NS == 1) U_T = PI - U_T;
		eta = acos(cos(U_T) / cos(inp.LATLS));
		if (I_m > PI05) eta = -eta;
		LAMBDA = lambda_N - OrbMech::w_Earth * TIP;
		LONG = LAMBDA + eta;
		if (LONG < 0) LONG += PI2;
		DLON = LONG - inp.LONGLS;
		while (DLON > PI) DLON -= PI2;
		while (DLON <= -PI) DLON += PI2;
		if (DLON < 0 && ITER == 1) DLON += PI2;
		dt = DLON / OrbMech::w_Earth;
		TIP += dt;
		ITER++;
	} while (abs(DLON) >= DOS && ITER < CMAX);

	if (ITER >= CMAX)
	{
		//Error: NPLAN did not converge
		error = 4;
	}
}

void LaunchWindowProcessor::LWT()
{
	double GMTIP, V, DV, GMTIP0;
	OrbMech::ITERSTATE iter;

	GMTIP = TT;
	
LWT1:

	iter.dv = 0.0;
	NPLAN(GMTIP);
	if (error) return;
	GMTIP0 = GMTIP;
	do
	{
		GMTIP = GMTIP0 + iter.dv;

		//Call LENSR with negative YSMAX to force an in-plane insertion
		inp.YSMAX = -inp.YSMAX;
		LENSR(GMTIP);
		inp.YSMAX = -inp.YSMAX;

		V = length(VP);
		DV = 2.0*V*sin(WEDGE / 2.0);

		iter.err = DV;

		if (abs(iter.err) > DVTOL)
		{
			OrbMech::ITER(iter.c_I, iter.s_F, iter.err, iter.p_H, iter.dv, iter.erro, iter.dvo, DX1);
			if (iter.s_F)
			{
				error = 3;
				return;
			}
		}
	} while (abs(iter.err) > DVTOL);

	if (inp.NS != 1)
	{
		OPEN = GMTIP + inp.DTOPT;
		inp.TPLANE = OPEN;
		LENSR(inp.TPLANE);
		PAO = PHANG();
	}
	if (inp.NS == 2)
	{
		inp.NS = 1;
		goto LWT1;
	}
	if (inp.NS != 0)
	{
		CLOSE = GMTIP + inp.DTOPT;
		GMTINS = CLOSE + inp.PFT;
		inp.TPLANE = CLOSE;
		LENSR(inp.TPLANE);
		PAC = PHANG();
	}
}

double LaunchWindowProcessor::PHANG()
{
	VECTOR3 u, R_A1, U_L;
	double phase;

	u = unit(crossp(RT, VT));
	U_L = unit(crossp(u, RT));

	R_A1 = unit(RP - u * dotp(RP, u))*length(RP);
	phase = acos(dotp(unit(R_A1), unit(RT)));
	if (dotp(U_L, R_A1) > 0)
	{
		phase = -phase;
	}
	if (inp.NEGTIV == 0)
	{
		while (phase >= PI2) phase -= PI2;
		while (phase < 0) phase += PI2;
	}
	else if (inp.NEGTIV == 1)
	{
		while (phase >= 0) phase -= PI2;
		while (phase < -PI2) phase += PI2;
	}
	else
	{
		while (phase >= PI) phase -= PI2;
		while (phase < -PI) phase += PI2;
	}
	phase += PI2 * (double)inp.WRAP;
	return phase;
}

void LaunchWindowProcessor::GMTLS(double TI, double TF)
{
	double TLO, DT, phase, l_dot;
	int ITER;

	K25 = 0;
	TLO = TI;

	while (TLO <= TF)
	{
		K25++;
		if (K25 > 10)
		{
			error = 5;
			return;
		}
		ITER = 0;
		do
		{
			LENSR(TLO);
			phase = PHANG();
			l_dot = OrbMech::GetMeanMotion(RP, VP, OrbMech::mu_Earth);
			DT = phase / l_dot;
			ITER++;
			if (ITER > CMAX)
			{
				error = 6;
				return;
			}
			TLO = TLO - DT;
		} while (abs(DT) > DET);

		STAR[K25 - 1] = TLO;
		TLO = TLO + PI2 / l_dot;
		if (STABLE == 2)
		{
			TLO = TF + 60.0;
		}
	}

	GSTAR = STAR[K25 - 1];
}

void LaunchWindowProcessor::RLOT()
{
	double DALT, DTYAW, UINS, DH;
	int ITER, ITINS, LAST;

	ITER = 0;
	ITINS = 0;

	LAST = 1;

	DALT = 0.0;
	DTYAW = 0.0;

	if (inp.WRAP != 0) inp.OFFSET = inp.OFFSET + PI2 * (double)inp.WRAP;

	if (inp.INSCO != 1)
	{
		inp.VINS = sqrt(OrbMech::mu_Earth*(2.0 / inp.RINS - 1.0 / ANOM)) + 10.0*0.3048;
	}

	switch (inp.LOT)
	{
	case 1:
	case 2:
	case 3:
		GMTLO = inp.GMTLOR;
		break;
	case 4:
	case 6:
		GMTLO = inp.TPLANE;
		break;
	case 5:
		GMTLO = inp.TPLANE + inp.TRANS;
		break;
	}

	GMTLS(GMTLO, GMTLO + 1.0); //The 1.0 doesn't matter as STABLE is set to 2

	do
	{
		do
		{
			ITINS = 0;
			do
			{
				switch (inp.LOT)
				{
				case 1:
				case 5:
				case 6:
					NSERT(GMTLO, UINS, DH);
					break;
				case 2:
					break;
				case 3:
				case 4:
					break;
				}

				switch (inp.INSCO)
				{
				case 1:
					DALT = 0.0;
					break;
				case 2:
				case 3:
					//TBD
					break;
				}
				ITINS++;
				if (ITINS >= CMAX)
				{
					error = 8;
					return;
				}
			} while (abs(DALT) >= DELH);

			inp.BIAS = GMTLO - GSTAR;
			TARGT();
			if (LAST == 1)
			{
				TYAW = inp.TPLANE + inp.DELNO / OrbMech::w_Earth;
			}
			else
			{
				LAST = 1;
			}
			if (inp.LOT == 6)
			{
				DTYAW = TYAW - GMTLO;
				GMTLO = TYAW;
			}

			ITER++;
			if (ITER >= CMAX)
			{
				error = 8;
				return;
			}
		} while (abs(DTYAW) >= DET);

		if (inp.LOT == 6)
		{
			LAST = 0;
			inp.LOT = 1;
			GMTLO = TYAW + inp.TRANS;
		}

	} while (LAST != 1);
}

void LaunchWindowProcessor::TARGT()
{
	OrbMech::CELEMENTS coe_osc_T, coe_mean_T, coe_osc_C;
	double TINS, TGRR, WT, HDOTC, HDOTT, UC, LDOTT, LDOTC;

	TINS = GMTLO + inp.PFT;

	//Update target to insertion
	UPDAT(RT, VT, TT, TINS);
	//Update chaser to insertion
	UPDAT(RP, VP, TP, TINS);

	TGRR = GMTLO - 0.0;//TBD: inp.DTGRR;
	WT = OrbMech::w_Earth * TGRR;
	while (WT >= PI2)
	{
		WT -= PI2;
	}

	coe_osc_C = OrbMech::CartesianToKeplerian(RP, VP, OrbMech::mu_Earth);
	coe_osc_T = OrbMech::CartesianToKeplerian(RT, VT, OrbMech::mu_Earth);
	UC = coe_osc_C.g + OrbMech::MeanToTrueAnomaly(coe_osc_C.l, coe_osc_C.e);
	if (UC >= PI2)
	{
		UC -= PI2;
	}
	if (inp.SVPROP == 1)
	{
		OrbMech::CELEMENTS coe_mean_C;
		double GDOTT, GDOTC;

		coe_mean_C = OrbMech::OsculatingToBrouwerMeanLong(coe_osc_C, OrbMech::mu_Earth);
		OrbMech::BrouwerSecularRates(coe_osc_C, coe_mean_C, LDOTC, GDOTC, HDOTC);

		coe_mean_T = OrbMech::OsculatingToBrouwerMeanLong(coe_osc_T, OrbMech::mu_Earth);
		OrbMech::BrouwerSecularRates(coe_osc_T, coe_mean_T, LDOTT, GDOTT, HDOTT);
	}
	else
	{
		coe_mean_T = coe_osc_T;
		HDOTT = HDOTC = 0.0;
		LDOTC = OrbMech::GetMeanMotion(RP, VP, OrbMech::mu_Earth);
		LDOTT = OrbMech::GetMeanMotion(RT, VT, OrbMech::mu_Earth);
	}

	//H is now defined relative to GRR, not midnight
	coe_mean_T.h = coe_mean_T.h - WT;
	if (coe_mean_T.h < 0)
	{
		coe_mean_T.h += PI2;
	}

	//aegdata.h_dot = aegdata.h_dot*cos(aegdata.coe_mean.i) / cos(sv_T.coe_mean.i);

	DN = coe_mean_T.h - PI;
	if (DN < 0)
	{
		DN = coe_mean_T.h + PI;
	}

	if (inp.SVPROP == 1)
	{
		DN = DN + 1.5*OrbMech::J2_Earth*pow(OrbMech::R_Earth, 2) / (2.0*pow(coe_osc_C.a, 2)*pow(1.0 - pow(coe_osc_C.e, 2), 2))*cos(coe_mean_T.i)*sin(2.0*UC);
	}

	if (inp.DELNOF)
	{
		inp.DELNO = -((HDOTT - HDOTC)*PA) / (LDOTT - LDOTC);
		if (abs(inp.BIAS) > 0.0)
		{
			DELNOD = inp.DELNO / inp.BIAS;
		}
	}

	//Compute IIGM
	IIGM = coe_mean_T.i;
	if (inp.SVPROP == 1)
	{
		IIGM = IIGM + 1.5*OrbMech::J2_Earth*pow(OrbMech::R_Earth, 2) / (4.0*pow(coe_osc_C.a, 2)*pow(1.0 - pow(coe_osc_C.e, 2), 2))*sin(2.0*coe_mean_T.i)*cos(2.0*UC);
	}

	TIGM = DN - inp.LONGLS + inp.DELNO + OrbMech::w_Earth*(inp.PFT + inp.DTGRR);
	if (TIGM < 0)
	{
		TIGM += PI2;
	}
	TDIGM = HDOTT - OrbMech::w_Earth + DELNOD;

	//Compute AZL
	AZL = inp.LAZCOE[0] + inp.LAZCOE[1] * IIGM + inp.LAZCOE[2] * TIGM + inp.LAZCOE[3] * IIGM*TIGM;

	coe_osc_C.i = IIGM;
	coe_osc_C.h = DN + PI + inp.DELNO + OrbMech::w_Earth*GMTLO;
	while (coe_osc_C.h >= PI2)
	{
		coe_osc_C.h -= PI2;
	}
	OrbMech::KeplerianToCartesian(coe_osc_C, OrbMech::mu_Earth, RP, VP);
	//Compute GPAZ
	//TRS = DN / (GLOCON.w_E + sv_T.h_dot) + TINS;
	//TS = GMTLO - TRS;
	//lat_star = inp.LONGLS - GLOCON.w_E*TS;
	//GPAZ = atan2(-cos(IIGM)*cos(inp.LATLS) + sin(IIGM)*sin(lat_star)*sin(inp.LATLS), sin(IIGM)*cos(lat_star));
}

void LaunchWindowProcessor::NSERT(double GMTLO, double &UINS, double &DH)
{
	LENSR(GMTLO);

	DH = length(RT) - length(RP);
	PA = PHANG();
	UINS = 0.0;
}

void LaunchWindowProcessor::LWPT()
{
	//lwp_param_table->GMTLO[0] = inp.TPLANE + inp.TSTART;
	//LENSR(lwp_param_table->GMTLO[0]);
	//lwp_param_table->PHASE[0] = PHANG();

	//lwp_param_table->GMTLO[1] = inp.TPLANE + inp.TEND;
	//LENSR(lwp_param_table->GMTLO[1]);
	//lwp_param_table->PHASE[1] = PHANG();
}

void LaunchWindowProcessor::LWDSP()
{

}

//Recommended Lift-off-Time (RLOT) display routine
void LaunchWindowProcessor::RLOTD()
{

}

//Launch Window Processor Output Tables routine
void LaunchWindowProcessor::LWPOT()
{
	LWPSV.sv_P.R = RP;
	LWPSV.sv_P.V = VP;
	LWPSV.sv_P.GMT = TP;
	LWPSV.sv_P.mass = inp.CWHT;
	LWPSV.sv_T0 = inp.TRGVEC;
	LWPSV.sv_T_MECO.R = RT;
	LWPSV.sv_T_MECO.V = VT;
	LWPSV.sv_T_MECO.GMT = TT;
	LWPSV.sv_T_MECO.mass = inp.TRGVEC.mass;

	lwp_param_table->GMTLO[0] = GMTLO + inp.TSTART;
	LENSR(lwp_param_table->GMTLO[0]);
	lwp_param_table->PHASE[0] = PHANG();

	lwp_param_table->GMTLO[1] = GMTLO + inp.TEND;
	LENSR(lwp_param_table->GMTLO[1]);
	lwp_param_table->PHASE[1] = PHANG();
}

void LaunchWindowProcessor::GetOutput(LWPSummary &out)
{
	out.LWPERROR = error;
	out.GMTLO = GMTLO;
	out.GMTINS = GMTINS;
	out.AZL = AZL;
	out.VIGM = inp.VINS;
	out.RIGM = inp.RINS;
	out.IIGM = IIGM;
	out.TIGM = TIGM;
	out.TDIGM = TDIGM;
	out.DN = DN;
	out.DELNO = inp.DELNO;
	out.PA = PA;
	out.TPLANE = inp.TPLANE;
	out.LATLS = inp.LATLS;
	out.LONGLS = inp.LONGLS;
}