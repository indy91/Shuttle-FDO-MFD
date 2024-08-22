/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Deorbit Maneuver Processor

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

#include "DMP.h"
#include "PEG4.h"

DMP::DMP()
{

}

void DMP::Executive(const DMPOptions &opt, DMPResults &res)
{
	//Null some variables
	for (int i = 0; i < 10; i++)
	{
		AAA[i] = 0.0;
	}

	//Save options
	this->opt = opt;

	IDTM = 0;
DMP_DTM_1:
	IDTM++;
	if (IDTM != 1)
	{
		this->opt.TIG = this->opt.TTHRSH;
		this->opt.ITIGFR = 0;
	}
	//Initialization and units conversion task
	res.ErrorMessage = "";
	DTT1();
	if (IMSG > 0)
	{
		goto DMP_DTM_4;
	}
	if (this->opt.ITIGFR == 1)
	{
		//Compute a TIG free solution
		DTM2(res);
	}
	else
	{
		//Compute a TIG fixed solution
		DTM3(res);
	}
	if (IMSG > 0) goto DMP_DTM_4;
DMP_DTM_2:
	if (this->opt.TIG < this->opt.TTHRSH)
	{
		//Return for a TIG fixed solution
		goto DMP_DTM_1;
	}
	//Compute offset targets
	DTM14();
	if (IMSG > 6)
	{
		//PEG error
		goto DMP_DTM_4;
	}
	//Compute crossrange
	DTT24();
	if (this->opt.ITIGFR == 2)
	{
		//Set to TIG free mode
		this->opt.ITIGFR = 1;
	}
	//Compute LVLH components of primary system DV
	VGO = mul(OrbMech::LVLH_Matrix(RAAA, VA), VG);
	//Build position/velocity phase tables and summary table
	DTMOT(res);
	//Print primary solution output display
	DTMPR(res);
	//PEG guidance used to compute backup solution
	DTT12();
	if (IMSG > 6)
	{
		//PEG error
		goto DMP_DTM_4;
	}
	//Add J2 effects to the R and V vector
	//SUPRJ();
	//Integrate precise burnout state to EI
	DTT7();
	//Compute crossrange
	DTT24();
	//Compute EI range with EGTR
	DTT13();
DMP_DTM_4:
	//Write error message condition detected
	DTMER(res);
	if (IMSG == 0 || IMSG == 4 || IMSG == 5)
	{
		//No fatal error
		if (IMSG == 4 || IMSG == 5)
		{
			IMSG = 0;
			goto DMP_DTM_2;
		}
	}
}

void DMP::DTM2(DMPResults &res)
{
	//Compute an in-plane minimum DV solution
	DTM4();
	//Check for errors and print user message
	DTMER(res);
	if (IMSG != 0) return;
	//Compute crossrange and time of closest approach
	DTT24();
	//Estimate TIG and theta_EI for primary min DV solution
	DTT3();
DMP_DTM2_1:
	//Increment TIG by orbit period
	DTT21();
	//Check for error condition and print user message
	DTMER(res);
	//Search next rev if minimum DV TIG is within small bias of TTHRSH
	if (opt.TIG < opt.TTHRSH + TTHRB)
	{
		goto DMP_DTM2_1;
	}
	if (opt.IFUEL == 1)
	{
		//Initialize out-of-plane burn direction
		SBD = 1.0;
		//Compute equal yaw solution for input C1, C2 and range
		DTM6();
		//Check for error conditions and print user messages
		DTMER(res);
		if (IMSG == 1) return;
		//Compute time of closest approach to landing site
		DTT24();
		//Compute the out of plane burn direction to reduce crossrange
		DTT4();
		//Use ETG computed targets
		IPRNG = false;
		//Compute equal yaw angle solution using approx. ETG mode
		DTM6();
		//Check for error conditions and print user messages
		DTMER(res);
		if (IMSG == 1) return;
		if (abs(DWBU - DWPR) < DWTOL)
		{
			if (DTCOST < opt.TFFMIN)
			{
				//Backup free-fall time violation

				//Compute solution with new TIG to increase TFF in backup solution
				DTM9();
				//Check for error conditions and print user message
				DTMER(res);
				if (IMSG == 1 || IMSG == 6) return;
			}
		}
		else
		{
			//Use input targets
			SBD = 0.0;
			IPRNG = true;
			//Compute inplane equal OMS solution with input C1,C2 and range
			DTM8();
			//Check for error conditions and print user message
			DTMER(res);
			if (IMSG == 1)return;
		}
	}
	else
	{
		//Compute in-plane equal OMS solution for input C1, C2, range
		DTM8();
		//Check for error conditions and print user messages
		DTMER(res);
		if (IMSG == 1) return;
	}
	//Use ETG compute targets
	IPRNG = false;
	//Compute in-plane equal OMS solution using approx ETG mode
	DTM8();
	//Check for error conditions and print user messages
	DTMER(res);
	if (IMSG == 1) return;
	if (TFFBU < opt.TFFMIN)
	{
		ITFF = 4;
		IMSG = 5;
	}
	if (opt.TIG < opt.TTHRSH) goto DMP_DTM2_1;
	//Call ETG in precise mode
	IFINAL = true;
	ICALL = true;
	//Prime and backup solutions with precise ETG
	DTM7();
}

void DMP::DTM3(DMPResults &res)
{
	//Compute transfer angle to satisfy input range with impulsive data
	DTM5();
	//Check for error conditions and print user message
	DTMER(res);
	if (IMSG != 0.0) return;
	//Compute crossrange and time of closest approach
	DTT24();
	//Finite burn with ETG computed range
	IMPULS = false;
	IPRNG = false;
	if (opt.IFUEL == 1)
	{
		//Compute out of plane burn direction
		DTT4();
	}
	//Compute prime and backup solutions using approx. ETG mode
	DTM7();
	if (TFFBU < opt.TFFMIN)
	{
		//Free fall constraint violation
		ITFF = 4;
		IMSG = 5;
	}
	//Call ETG in precise mode
	IFINAL = true;
	ICALL = true;
	if (IAME != 2)
	{
		//Compute prime and backup solutions using precise ETG mode
		DTM7();
	}
}

void DMP::DTM4()
{
	JJ = 1;
	IMPULS = true;
	opt.ITIGFR = 1;
	NCNT4 = 0;
	while (!(JJ == 0 || NCNT4 >= NCMAX))
	{
		//Predict theta_EI and TIG for impulsive minimum DV
		DTT2();
		NCNT4++;
		//Compute impulsive solution and ensure compatible V, gamma, range at EI using input data
		DTM5();
		if (IMSG > 6 || IMSG == 1)
		{
			break;
		}
	}
	if (NCNT4 >= NCMAX)
	{
		//Max iterations exceeded
		MITER = 2;
		IMSG = 1;
		IMPULS = false;
	}
}

void DMP::DTM5()
{
	NCNT2 = 0;
	NSYS = 1;
DTM_DTM5_1:
	if (opt.ITIGFR == 1)
	{
		//Compute TIG to null the range error
		DTT15();
		NCNT2++;
	}
	//Integrate state to TIG
	DTT7();
	//Compute LVLH coordinates at TIG
	DTT8();
DTM_DTM5_2:
	if (opt.ITIGFR != 1)
	{
		NCNT2++;
		//Compute theta_EI to null the range error
		DTT16();
	}
	//Compute inplane target at EI
	DTT10();
	if (IMPULS)
	{
		//LTVCON of PEG
		DTT11();
	}
	else
	{
		//PEG guidance
		DTT12();
	}
	//Predict EI range
	DTT13();
	//Compute C1, C2, range
	DTT14();

	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "DTM5: NCNT2 %d THETLS %lf THELSD %lf RNGTOL %lf", NCNT2, THETLS*DEG, THELSD*DEG, RNGTOL);
		oapiWriteLog(DebugBuffer);
	}

	if (!(abs(THETLS - THELSD) < RNGTOL || NCNT2 >= NCMAX))
	{
		if (opt.ITIGFR == 1)
		{
			goto DTM_DTM5_1;
		}
		else
		{
			goto DTM_DTM5_2;
		}
	}
	if (NCNT2 >= NCMAX)
	{
		MITER = 3;
		IMSG = 1;
	}
	else
	{
		//End task for basic target routine
		DTT17();
	}
}

void DMP::DTM6()
{
	IMPULS = false; //Not in the document
	ITIGG = 0;
	NCNT = 0;
	TGSLPP = TGSLPY;
	TGDMN = TGDSI;
	do
	{
		//Compute TIG for equal yaw angles
		DTT5();
		NCNT++;
		//Compute the prime and backup system solutions
		DTM7();
		//Compute yaw angle delta
		DY2 = PSIBU - PSIPR;
	} while (abs(DY2) >= DSITOL);
	if (NCNT >= NCMAX)
	{
		MITER = 4;
		IMSG = 1;
	}
}

void DMP::DTM7()
{
	//Compute a primary system solution to ensure a compatible V,gamma,range at EI
	DTM5();
	//Update primary system entry range
	DTT13();
	//Compute primary system crossrange
	DTT24();
	//Use PEG to compute a backup system solution to targets computed by BTR
	DTT12();
	//Compute backup system crossrange
	DTT24();
	//Update backup system entry range
	DTT13();
}

void DMP::DTM8()
{
	ITIGG = 0;
	NCNT3 = 0;
	TGDMN = TGDOMS;
	TGSLPP = TGSLPV;
	do
	{
		//Compute TIG for equal OMS propellant usage
		DTT5();
		NCNT3++;
		//Compute prime and backup solution
		DTM7();
		//Compute TIG adjust parameter
		DY2 = DWBU - DWPR;
	} while (abs(DY2) >= DWTOL && NCNT3 < NCMAX);
	if (NCNT3 >= NCMAX)
	{
		MITER = 5;
		IMSG = 1;
	}
}

void DMP::DTM9()
{
	NCNT = 0;
	ITFF = 2;
	IMSG = 4;
	while ((TFFBU < opt.TFFMIN || TFFBU > opt.TFFMIN + TFFTOL) && NCNT < NCMAX)
	{
		//Compute TIG and THETEI to increase TFF
		DTT18();
		NCNT++;
		//Compute prime and backup solutions
		DTM7();
	}
	if (DWPR > opt.WCGOMS + WBIAS || DWBU > opt.WCGOMS + WBIAS)
	{
		//Set free fall constraint flag
		IMSG = 6;
		ITFF = 3;
	}
	if (NCNT >= NCMAX)
	{
		//Set error flags
		MITER = 6;
		IMSG = 1;
	}
}

void DMP::DTM14()
{
	IFFPTM = false;
	NCNT = 0;
	NSYS = 1;
	//Task to initiate offset targeting
	DTT22();
	do
	{
		NCNT++;
		if (NCNT != 1)
		{
			//Compute offset targets
			THETEI = THETEI + DTHELS;
			CA[0] = CA[0] + DC11;
		}
		//Burnout state using offset targets
		DTT12();
		//Precise burnout state prediction with J2
		//SUPRJ();
		//Integrate the burnout state to entry interface and save the state
		DTT7();
		//Compute the vertical velocity offset
		DTT23();
		//Compute EI range with offset targets
		DTT13();
		//Compute EI range offset
		DTHELS = THETLS - THELSD;
	} while ((abs(DTHELS) > RNGTOL || abs(DGAM) > GAMTOL) && NCNT < NCMAX);
	if (NCNT > NCMAX)
	{
		MITER = 8;
		IMSG = 1;
	}
}

double RLIMIT(double X, double XMIN, double XMAX)
{
	return min(XMAX, max(X, XMIN));
}

double SIGN(double a, double b)
{
	if (b < 0)
	{
		return -a;
	}
	return a;
}

void DMP::DTT3()
{
	double TBURNP;

	//Compute the primary system burn time to achieve DV
	TBURNP = opt.WT / WDPR * (1.0 - exp(-DVIMP * WDPR / FPR));
	//Estimate THETEI for a finite burn
	THETEI = THETEI + (TBURNP / 2.0*THETD);
	//Estimate TIG for a finite burn
	opt.TIG = opt.TIG - TBURNP / 2.0;
}

void DMP::DTT4()
{
	VECTOR3 ULS;
	double TBURN, THHAFB, LAMDAI, ULSUY, ULSUX;

	//Compute downrange vector at TIG plus the time of one half the burn arc
	TBURN = opt.WT / WDPR * (1.0 - exp(-DVOBPR * WDPR / (FPR)));
	THHAFB = TBURN / 2.0*THETD;
	UXDTM = UXDTM * cos(THHAFB) + UXDTM * sin(THHAFB);
	//Compute landing site longitude at time of closest approach
	LAMDAI = opt.TLONG + OrbMech::w_Earth*TC + ERAI;
	//Compute landing site unit vector at time of closest approach
	ULS = _V(cos(TLATC)*cos(LAMDAI), cos(TLATC)*sin(LAMDAI), sin(TLATC));
	//Compute burn direction parameter
	ULSUY = dotp(ULS, UYDTM);
	ULSUX = dotp(ULS, UXDTM);
	SBD = ULSUY * ULSUX;
	if (abs(SBD) <= 1.0e-7)
	{
		SBD = -1.0;
	}
	SBD = -OrbMech::sign(SBD);
	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "DTT5: SBD %lf UX %lf %lf %lf ULS %lf %lf %lf TBURN %lf THHAFB %lf", SBD, UXDTM.x, UXDTM.y, UXDTM.z, ULS.x, ULS.y, ULS.z, TBURN, THHAFB);
		oapiWriteLog(DebugBuffer);
	}
}

void DMP::DTT5()
{
	if (ITIGG > 0)
	{
		double DTIG;

		if (ITIGG > 1)
		{
			double TGDEN, TGSLP;

			TGDEN = DY2 - DY1;
			if (abs(TGDEN) < TGDMN)
			{
				TGSLP = TGSLPP;
			}
			else
			{
				TGSLP = (opt.TIG - TIG1) / TGDEN;
				TGSLPP = TGSLP;
			}
			DTIG = DY2 * TGSLP;
			if (abs(DTIG) > DTGMAX)
			{
				DTIG = SIGN(DTGMAX, DTIG);
			}
		}
		else
		{
			DTIG = DTIGO;
		}
		TIG1 = opt.TIG;
		DY1 = DY2;
		opt.TIG = opt.TIG - DTIG;
		THETEI = THETEI + DTIG*THETD;
	}
	ITIGG++;
	opt.ITIGFR = 2;
}

void DMP::DTT1()
{
	IAME = 0;
	IBTR = true;
	ICALL = true;
	IFFPTM = true;
	IFINAL = false;
	IMPULS = true;
	INCR = false;
	IMSG = 0;
	NCNT1 = 0;
	IPRNG = true;
	SBD = 0.0;
	KDSTER = false;
	CA[0] = opt.C1IP;
	CA[1] = opt.C2IP;

	if (IDTM == 1)
	{
		ERAI = 0.0; // EARTHR * opt.TIMEC;
		THETEI = THEIIP;
	}
	else
	{
		THETEI = THETEI - DTTHRS * THETD;
	}
	if (opt.ITIGFR == 1)
	{
		opt.TIG = opt.TTHRSH;
	}
	//if (opt.KDGUID == 6)
	//{
	//	KDSTER = true;
	//	opt.KDGUID = 3;
	//}

	double F, WD;
	int KOMS, KRCS;

	for (int ICOUNT = 0;ICOUNT < 2;ICOUNT++)
	{
		//Initialize thrust mode and thruster configuration
		if (ICOUNT == 1)
		{
			KDTHR = opt.INGBU;
		}
		else
		{
			KDTHR = opt.INGPR;
		}
		KOMS = 0;
		KRCS = 0;
		if (KDTHR == 16 || KDTHR == 17)
		{
			KOMS = 2;
		}
		else if (KDTHR == 14 || KDTHR == 15)
		{
			KOMS = 1;
		}
		else
		{
			KRCS = 1;
		}
		F = opt.FOMS * (double)KOMS + opt.FRCS * (double)KRCS;
		WD = opt.XMDOMS * (double)KOMS + opt.XMDRCS * (double)KRCS;

		if (ICOUNT == 1)
		{
			FBU = F;
			WDBU = WD;
		}
		else
		{
			FPR = F;
			WDPR = WD;
		}
	}

	SMISS = SMISSI;
	//Compute prime and backup system DV onboard
	DVOBPR = -FPR / WDPR * log((opt.WT - opt.WCGOMS) / opt.WT);
	DVOBBU = -FBU / WDBU * log((opt.WT - opt.WCGOMS) / opt.WT);
	//Compute orbit rate and closing rate between vehicle and landing site
	THETD = pow(2.0*XMU / length(opt.XYZI) - pow(length(opt.XYZID), 2), 1.5) / XMU;
	UYDTM = unit(crossp(opt.XYZID, opt.XYZI));
	THETDR = THETD + EARTHR * UYDTM.z;
	//Compute initial value of THEPEI for minimum DV computation
	THEPEI = SSS[3] + GGOL * (SSS[4] - SSS[3]);

	//Compute the geocentric landing site location
	GEOD(opt.TALTD, opt.TLATD, RCHMAG, TLATC);

	//Check if input C2 is zero
	if (CA[1] == 0.0)
	{
		IC2FLG = true;
		UpdateC1C2(25601.0*0.3048, CA[0], CA[1]);
		//Also set as input
		opt.C1IP = CA[0];
		opt.C2IP = CA[1];
	}
	else
	{
		IC2FLG = false;
	}
	//Check if input range is zero
	if (opt.RNGIP == 0.0)
	{
		FVE(25601.0*0.3048, THELSD, GAMETG);
		opt.RNGIP = THELSD * RCHMAG;
	}

	//TBD: Dump all variables in COM and ICOM
}

void DMP::GEOD(double GHD, double GPHID, double &GR, double &GPHIC)
{
	GR = EQRAD + GHD;
	GPHIC = GPHID;

	/*double ESQ, SPHID, CPHID, T1, T2, T3, T4, T5;
	
	ESQ = 1.0 - POLRAD / EQRAD / EQRAD;
	SPHID = sin(GPHID);
	CPHID = cos(GPHID);
	T1 = ESQ * SPHID*CPHID / (1.0 - ESQ * SPHID*SPHID);
	T2 = cos(GPHID - T1);
	T3 = ESQ * T2*T2;
	T4 = POLRAD * (1.0 + 0.5*T3*0.375*T3*T3);
	T5 = T4 * T1 / (T4 + GHD);
	GR = T4 + GHD * (1.0 - 0.5*T1*T5);
	GPHIC = GPHID - T5;*/
}

void DMP::DTT2()
{
	//Find a minimum THETEI for the input DV
	GLPRP();
	opt.TIG = opt.TIG + (THEPEI - THETEI) / THETD;
	THEPEI = THETEI;
	if (opt.IPOUT == 2)
	{
		//Print TIG, THETEI, AAA, JJ
		sprintf_s(DebugBuffer, "DTT2: TIG %lf THETEI %lf AAA %lf %lf %lf %lf %lf", opt.TIG, THETEI, AAA[0], AAA[1], AAA[2], AAA[3], AAA[4]);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "DTT2: AAA %lf %lf %lf %lf %lf JJ %d", AAA[5], AAA[6], AAA[7], AAA[8], AAA[9], JJ);
		oapiWriteLog(DebugBuffer);
	}
}

void DMP::GLPRP()
{
	if (JJ == 1)
	{
		//First entry
		AAA[9] = SSS[4] - SSS[3];
		AAA[8] = SSS[4] + AAA[9];
		AAA[4] = SSS[3] + GGOL * AAA[9];
		THETEI = AAA[4];
		AAA[3] = SSS[2];
		AAA[2] = SSS[2];
		AAA[1] = SSS[4];
		AAA[0] = SSS[3];
		JJ = 2;
		return;
	}
	else if (JJ == 2)
	{
		AAA[6] = DVIMP;
		AAA[5] = AAA[1] - GGOL * AAA[9];
		THETEI = AAA[5];
		AAA[9] = AAA[5] - AAA[0];
		JJ = 4;
		return;
	}
	AAA[JJ + 3] = DVIMP;
	JJ = 3;
	if (AAA[6] > AAA[7]) JJ = 4;
	AAA[9] = GGOL * (AAA[1] - AAA[0]);
	if (abs(AAA[9]) <= SSS[0])
	{
		//Solution converged
		THETEI = AAA[JJ + 1];
		JJ = 0;
		return;
	}

	double XMP, YMP;
	int L;

	//Move upper bound inward if JJ=3, move lower bound inward if JJ=4
	L = -JJ;
	AAA[L + 4] = AAA[L + 8];
	AAA[L + 8] = AAA[JJ + 1];
	XMP = AAA[JJ + 1];
	AAA[L + 6] = AAA[L + 10];
	AAA[L + 10] = AAA[JJ + 3];
	YMP = AAA[JJ + 3];

	//Compute the next value of THETEI
	AAA[JJ + 1] = AAA[JJ - 3] + GGOL * (AAA[L + 4] - AAA[JJ - 3]);
	THETEI = AAA[JJ + 1];

	//Has the minimum DV been bracketed between the computed values of DV and DVMAX?
	if (AAA[2] >= SSS[2]) return;
	if (AAA[3] >= SSS[2]) return;

	//Compute the parabolic prediction of the optimum value of THETEI
	double TAMP, TEMP, TIMP, TOMP;

	TAMP = AAA[0] * (AAA[3] - YMP);
	TEMP = XMP * (AAA[2] - AAA[3]);
	TIMP = AAA[1] * (YMP - AAA[2]);
	TOMP = TAMP + TEMP + TIMP;

	if (abs(TOMP) <= 0.0) return;

	double XP;
	XP = (AAA[0] * TAMP + XMP * TEMP + AAA[1] * TIMP) / (2.0*TOMP);
	TEMP = abs(XP - AAA[8]);
	AAA[8] = XP;

	//Test for convergence
	if (TEMP > SSS[1]) return;

	//Parabolic prediction converged, set THETEI equal to the predicted value
	THETEI = XP;
	JJ = 0;
}

void DMP::DTMER(DMPResults &res)
{
	char Buffer[128];

	res.ErrorCode = IMSG;

	switch (IMSG)
	{
	case 1:
		sprintf(Buffer, "DTM: Max iterations exceeded - MITER = %d (Error)", MITER);
		res.ErrorMessage.assign(Buffer);
		break;
	case 4:
		res.ErrorMessage = "DTM9: Free fall time force to TFFMIN constraint (Warning)";
		break;
	case 5:
		res.ErrorMessage = "DTM9: Free fall time constraint violated (Warning)";
		break;
	case 6:
		res.ErrorMessage = "DTM9: Free fall time forced to TFFMIN constraint - solution unacceptable in DV (Error)";
		break;
	case 7:
		res.ErrorMessage = "PEG: No physical solution found (Error)";
		break;
	case 8:
		res.ErrorMessage = "PEG: VGO equal zero (Error)";
		break;
	case 9:
		res.ErrorMessage = "PEG: No convergence - max iterations exceed (Error)";
		break;
	}
}

void DMP::DTT7()
{
	if (IFFPTM)
	{
		//TIG
		UPDTV(opt.XYZI, opt.XYZID, opt.TIMEC, opt.TIG, RAAA, VA);
		return;
	}

	//EI
	VECTOR3 URENK, UREIS;
	double ESQ, DR, RENKM, VSING, RES, HEIS, RERNK, HEINK, DHEI, DT;
	
	ESQ = 1.0 - POLRAD * POLRAD / EQRAD / EQRAD;
	XOFCN = 0;
	bool stop = false;
	while (stop == false)
	{
		XOFCN++;
		//Integrate PEG burnout state to 400k feet for offset targeting
		UPDTV(RDA, VDA, TP, TT, RENK, VENK);

		RTA = RENK;
		VTAA = VENK;

		//Compute radius error at EI
		REISM = length(REIS);
		RENKM = length(RENK);
		URENK = unit(RENK);
		VSING = dotp(VENK, URENK);
		DR = REISM - RENKM;

		//Check on radius error
		UREIS = unit(REIS);
		RES = EQRAD;// / sqrt(1.0 + (ESQ*pow(UREIS.z, 2)) / (1.0 - ESQ));
		HEIS = REISM - RES;
		RERNK = EQRAD;//TBD
		HEINK = RENKM - RERNK;
		DHEI = HEIS - HEINK;
		if (abs(DHEI) < 2.0*0.3048 || XOFCN > 10)
		{
			stop = true;
		}
		else
		{
			DT = DHEI / VSING;
			TT = TT + DT;
			DTCOST = TT - TP;
			TEINK = TT;
		}
	};
}

void DMP::UPDTV(VECTOR3 RS, VECTOR3 VS, double TS, double TF, VECTOR3 &RF, VECTOR3 &VF)
{
	OrbMech::SV sv0, sv1;

	sv0.R = RS;
	sv0.V = VS;
	sv0.GMT = TS;
	sv0.mass = 1.0;

	if (opt.INTEGF)
	{
		sv1 = OrbMech::coast(sv0, TF - TS);
	}
	else
	{
		sv1 = OrbMech::coast_osc(sv0, TF - TS, OrbMech::mu_Earth);
	}
	RF = sv1.R;
	VF = sv1.V;
}

void DMP::DTT8()
{
	UZDTM = -unit(RAAA);
	UYDTM = unit(crossp(VA, RAAA));
	UXDTM = crossp(UYDTM, UZDTM);
}

void DMP::DTT10()
{
	//Compute a unit vector in the direction of EI
	UEI = -UZDTM * cos(THETEI) + UXDTM * sin(THETEI);
	//Compute the EI position vector
	double REIMAG = EIALT + HLIP(UEI);
	RTA = UEI * REIMAG;
	if (opt.IPOUT == 2)
	{
		//TBD: Print UXDTM, UZDTM, THETEI, RTA, UEI, REIMAG
		sprintf_s(DebugBuffer, "DTT10: UXDTM %lf %lf %lf UZDTM %lf %lf %lf THETEI %lf", UXDTM.x, UXDTM.y, UXDTM.z, UZDTM.x, UZDTM.y, UZDTM.z, THETEI);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "DTT10: RTA %lf %lf %lf UEI %lf %lf %lf REIMAG %lf", RTA.x, RTA.y, RTA.z, UEI.x, UEI.y, UEI.z, REIMAG);
		oapiWriteLog(DebugBuffer);
	}
}

double DMP::HLIP(VECTOR3 U)
{
	return EQRAD; //TBD
}

void DMP::DTT11()
{
	VECTOR3 UIN, VGOIMP, VTGIMP;
	int KFLAG;

	//Define plane of conic transfer
	UIN = unit(crossp(RAAA, VA));
	//Compute a conic DV to achieve a target position at entry interface
	LTVCN(CA[0], CA[1], RAAA, RTA, UIN, DTCOST, KFLAG, VTGIMP, VTAA);
	if (opt.IPOUT == 2)
	{
		//TBD: Print RAAA, RTA, UIN, CA, VTGIMP, VTAA, DTCOST
		sprintf_s(DebugBuffer, "DTT11: RAAA %lf %lf %lf UIN %lf %lf %lf C1 %lf C2 %lf", RAAA.x, RAAA.y, RAAA.z, UIN.x, UIN.y, UIN.z, CA[0], CA[1]);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "DTT11: VTGIMP %lf %lf %lf VTAA %lf %lf %lf DTCOST %lf", VTGIMP.x, VTGIMP.y, VTGIMP.z, VTAA.x, VTAA.y, VTAA.z, DTCOST);
		oapiWriteLog(DebugBuffer);
	}
	//Compute impulsive data quantities
	double RBODEI, RBOM, REIM;

	VGOIMP = VTGIMP - VA;
	VDA = VTGIMP;
	RDA = RAAA;
	DVIMP = length(VGOIMP);
	UFIMP = unit(VGOIMP);
	TT = opt.TIG + DTCOST;
	RBODEI = dotp(RAAA, RTA);
	RBOM = length(RAAA);
	REIM = length(RTA);
	THEBO = acos(RBODEI / (RBOM*REIM));

	DWPR = opt.WT*(1.0 - exp((-DVIMP * WDPR) / FPR));

	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "DTT11: DWPR %lf THEBO %lf TT %lf DVIMP %lf VGOIMP %lf %lf %lf", DWPR, THEBO, TT, DVIMP, VGOIMP.x, VGOIMP.y, VGOIMP.z);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "DTT11: VDA %lf %lf %lf RDA %lf %lf %lf", VDA.x, VDA.y, VDA.z, RDA.x, RDA.y, RDA.z);
		oapiWriteLog(DebugBuffer);
	}
}

double DMP_QM(double W, double WW)
{
	double F, V, X, FOLD;
	int LEVEL, BN, BD;

	F = V = X = 1.0;
	LEVEL = 3;
	BN = 0;
	BD = 15;
	do
	{
		FOLD = F;
		LEVEL = LEVEL + 2;
		BN = BN + LEVEL;
		BD = BD + 4 * LEVEL;
		X = (double)BD / ((double)(BD) - WW*X*(double)BN);
		V = (X - 1.0)*V;
		F = FOLD + V;
	} while (F != FOLD);
	return W + (1.0 - WW)*(5.0 - F * WW) / 15.0;
}

void DMP::LTVCN(double C1, double C2, VECTOR3 R0, VECTOR3 R1, VECTOR3 XNUNIT, double &DELTAT, int &KFLAG, VECTOR3 &V0, VECTOR3 &V1)
{
	VECTOR3 R0UNIT, R1UNIT, VTEMP;
	double R0MAG, R1MAG, CTHETA, STHETA, SX, XLAMBDA, RCIRCL, VCIRCL, CK, RX, AX, BX, CX, DX;
	int MAXCYC;

	//Set constants for normalizing, determine the transfer plane and transfer angle
	KFLAG = 0;
	MAXCYC = 60;
	R0MAG = length(R0);
	R1MAG = length(R1);
	R0UNIT = unit(R0);
	R1UNIT = unit(R1);
	CTHETA = dotp(R0UNIT, R1UNIT);
	VTEMP = crossp(R0UNIT, R1UNIT);
	STHETA = dotp(VTEMP, XNUNIT);
	VTEMP = R1 - R0;
	CX = length(VTEMP);
	SX = (R0MAG + R1MAG + CX) / 2.0;
	if (STHETA >= 0)
	{
		XLAMBDA = 1.0;
	}
	else
	{
		XLAMBDA = -1.0;
	}
	XLAMBDA = XLAMBDA *sqrt(1.0 - CX / SX);
	RCIRCL = SX / 2.0;
	VCIRCL = sqrt(XMU / RCIRCL);
	CK = STHETA / (1.0 - CTHETA);
	RX = R1MAG / R0MAG;

	//Set coefficients for quadratic equation solution
	AX = (RX - CTHETA) - C2 * STHETA;
	BX = -(C1 / VCIRCL)*STHETA;
	CX = -RCIRCL / R1MAG * (1.0 - CTHETA);
	DX = BX * BX - 4.0*AX*CX;
	if (DX < 0.0)
	{
		KFLAG = 1;
		IMSG = 7;
		return;
	}

	double VH0TIL, VR0TIL, VH1TIL, VR1TIL, UX, WX, WW, QM;

	//Solve for initial and final velocity vectors
	VH1TIL = -2.0*CX / (BX + sqrt(DX));
	VH0TIL = RX * VH1TIL;
	VR1TIL = C1 / VCIRCL + C2 * VH1TIL;
	VR0TIL = VR1TIL * CTHETA - (VH1TIL - (RCIRCL / R1MAG) / VH1TIL)*STHETA;
	VTEMP = crossp(XNUNIT, R0UNIT);
	V0 = (R0UNIT*VR0TIL + VTEMP * VH0TIL)*VCIRCL;
	VTEMP = crossp(XNUNIT, R1UNIT);
	V1 = (R1UNIT*VR1TIL + VTEMP * VH1TIL)*VCIRCL;

	//Transfer time
	UX = CK * (1.0 - (R0MAG / SX))*VH0TIL - VR0TIL;
	if (UX <= -1.0)
	{
		KFLAG = 2;
		IMSG = 7;
		return;
	}

	//Compute intermediate variables for transfer time computation
	DX = sqrt(1.0 - XLAMBDA * XLAMBDA*(1.0 - UX * UX)) - XLAMBDA * UX;
	WX = sqrt((1.0 + XLAMBDA + UX * DX) / 2.0);
	WW = (WX - 1.0) / (WX + 1.0);

	QM = DMP_QM(WX, WW);
	DELTAT = (RCIRCL / VCIRCL)*(4.0*DX*XLAMBDA + pow(DX / WX, 3)*QM);

	//Test
	//VECTOR3 R1test, V1test;
	//OrbMech::rv_from_r0v0(R0, V0, DELTAT, R1test, V1test, XMU);
	//double teest = 1.0;
}

void DMP::DTT12()
{
	PEG4 peg4;

	VECTOR3 REI, VEI, VMISS;
	double WD, VEX, DW, MBO;

	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "PEG Inputs: RTIG %lf %lf %lf VTIG %lf %lf %lf WT %lf", RAAA.x, RAAA.y, RAAA.z, VA.x, VA.y, VA.z, opt.WT);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "PEG Inputs: C1 %lf C2 %lf THETAT %lf TIG %lf POLE %lf %lf %lf WCG %lf", CA[0], CA[1], THETEI*DEG, opt.TIG, POLE.x, POLE.y, POLE.z, opt.WCGOMS * SBD);
		oapiWriteLog(DebugBuffer);
	}

	if (NSYS == 1)
	{
		FT = FPR;
		WD = WDPR;
	}
	else
	{
		FT = FBU;
		WD = WDBU;
	}

	VEX = FT / WD;

	if (peg4.OMSBurnPrediction(RAAA, VA, opt.TIG, _V(opt.WCGOMS * SBD, 0, 0), CA[0], CA[1], EIALT, THETEI, FT, VEX, opt.WT, false))
	{
		//No PEG 4 solution
		//KFLAG = 4;
		IMSG = 9;
		return;
	}
	peg4.GetOutputD(RDA, VDA, VG, TGO, MBO, DTCOST, REI, VEI, FWYAW, VMISS);

	VGMAG = length(VG);

	if (VGMAG <= 0)
	{
		//KFLAG = 3;
		IMSG = 8;
		return;
	}

	ATP = unit(VG);
	DW = opt.WT - MBO;
	TP = opt.TIG + TGO;

	/*double WD, DW;

	if (NSYS == 1)
	{
		KDTHR = opt.INGPR;
		WD = WDPR;
	}
	else
	{
		KDTHR = opt.INGBU;
		WD = WDBU;
	}
	//Set up call to PEG guidance
	KINIT = true;
	WCG = opt.WCGOMS * SBD;
	HTGT = EIALT;
	THETAT = THETEI;
	KDSTER = 1;

	//Finite burn prediction using PEG guidance
	PGSUP();

	//OMS fuel weight expended
	VGMAG = length(VG);
	DW = opt.WT*(1.0 - exp(-VGMAG * WD / FT));*/

	if (NSYS == 1)
	{
		//Prime system
		DVPR = VGMAG;
		DWPR = DW;
		TFFPR = DTCOST;
		PSIPR = FWYAW;
		ATPPR = ATP;
	}
	else
	{
		//Backup system
		DVBU = VGMAG;
		DWBU = DW;
		TFFBU = DTCOST;
		PSIBU = FWYAW;
		ATPBU = ATP;
	}

	TT = TP + DTCOST;

	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "PEG Outputs: TBO %lf RBO %lf %lf %lf VBO %lf %lf %lf", TP, RDA.x, RDA.y, RDA.z, VDA.x, VDA.y, VDA.z);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "PEG Outputs: TEI %lf REI %lf %lf %lf VEI %lf %lf %lf", TT, REI.x, REI.y, REI.z, VEI.x, VEI.y, VEI.z);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "PEG Outputs: WCG %lf VGO %lf %lf %lf TGO %lf FWYAW %lf DTCOST %lf", DW, VG.x, VG.y, VG.z, TGO, FWYAW, DTCOST); //DTAVG?
		oapiWriteLog(DebugBuffer);
		if (NSYS == 1)
		{
			sprintf_s(DebugBuffer, "PEG Outputs: DVPR %lf DWPR %lf PSIPR %lf TFFPR %lf", DVPR, DWPR, PSIPR, TFFPR);
		}
		else
		{
			sprintf_s(DebugBuffer, "PEG Outputs: DVBU %lf DWBU %lf PSIBU %lf TFFBU %lf", DVBU, DWBU, PSIBU, TFFBU);
		}
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "PEG Outputs: THETEI %lf TIG %lf VMISS %lf %lf %lf VGOMAG %lf", THETEI, opt.TIG, VMISS.x, VMISS.y, VMISS.z, VGMAG);
		oapiWriteLog(DebugBuffer);
		sprintf_s(DebugBuffer, "PEG Outputs: IMSG %d", IMSG);// "IYAW %d NCYC %d NMAX %d KDGUID %d KFLAG %d KCONV %d" IYAW, NCYC, NMAX, KDGUID, KFLAG, KCONV
		oapiWriteLog(DebugBuffer);
	}
}

/*void DMP::PGSUP()
{
	VECTOR3 VS;
	double TGOP, DVTEMP, DTHETA;
	int NMAX;
	bool IPS;

	VS = _V(0, 0, 0);
	KFLAG = 0;

	if (KINIT)
	{
		VGOP = VG;
		VSP = VS;
		TGOP = TGO;
		KCUTOF = false;
		XMASSE = opt.WT;// / WTMASS;
		XMBO = XMASSE - abs(WCG);// / WTMASS;
		IPS = false;
		KFUELD = WCG >= 0.0 ? 1.0 : -1.0;
		if (WCG == 0.0)
		{
			KFUELD = true;
		}
		NMAX = NMAX1;
		TRCSOF = opt.TIG + TBRCS;

		if (KDTHR == 1 || KDTHR == 11)
		{
			FT = opt.FRCS;
			XMDOT = opt.XMDRCS;
		}
		else if (KDTHR == 14 || KDTHR == 15)
		{
			FT = opt.FOMS;
			XMDOT = opt.XMDOMS;
			NOMS = 1;
		}
		else
		{
			FT = opt.FOMS;
			XMDOT = opt.XMDOMS;
			NOMS = 2;
		}

		VEXX = FT / XMDOT;
		VEXS = VEXX;
		FTS = FT;
		CLMDXZ = XLD;
		if (KDSTER)
		{
			CLMDXZ = 0.0;
		}
	}
	else
	{
		NMAX = NMAX2;
	}
	if (KITER)
	{
		TTT = opt.TIG;
	}
	if (KCUTOF)
	{
		//Decrement TGO and VGO after guidance cutoff
		TGO = TGO - TTT + TGOP;
		VG = VG - DVSS;
		VGOP = VG;
		TGOP = TGO;
		return; //Is this correct
	}
	if (opt.KDGUID == 1 && TRCSOF > opt.TIG && TRCSOF <= TTT && IPS == false)
	{
		//Set second phase thrust characteristics
		VEXX = VEXS;
		FT = FTS;
		IPS = true;
	}
	//Compute guidance acceleration and mass characteristics
	DVTEMP = -length(DVSS) / VEXX;
	XMASSE = XMASSE * exp(DVTEMP);
	ATR = FT / XMASSE;
	NCYC = 0;

	if (KINIT == false || opt.KDGUID == 7)
	{
		//PEG7, nothing
	}
	else
	{
		if (opt.KDGUID == 3)
		{
			//Nothing
		}
		else
		{
			NPHASE = 1;
			if (TBRCS > 0)
			{
				if (KDTHR > 13 && KDTHR < 18)
				{
					//Add RCS to first phase thrust
					NPHASE = 2;
					FT = FT + opt.FRCS;
					XMDOT = XMDOT + opt.XMDRCS;
					VEXX = FT / XMDOT;
					ATR = FT / XMASSE;
				}
				else
				{
					KFLAG = 5;
					return;
				}
			}
		}
		H2M50(THETAT, DTHETA, RTA);
		//RTMAG = length(RTA);
	}
DMP_PGSUP_3_4:
	PGOP3();
	VGOP = VG;
	TGOP = TGO;
	if (KFLAG == 0)
	{
		NCYC++;
		//Has guidance converged?
		if (KSTOP || NCYC >= NMAX || opt.KDGUID == 7)
		{
			if (KSTOP || opt.KDGUID == 7)
			{
				//Nothing
			}
			else
			{
				//No PEG 4 solution
				KFLAG = 4;
				IMSG = 9;
			}
			if (TGO > TGOMIN)
			{
				//Nothing
			}
			else
			{
				KCUTOF = true;
			}
		}
		else
		{
			goto DMP_PGSUP_3_4;
		}
		//Compute previous guidance time
		TGDP = TTT;
	}
}

void DMP::H2M50(double THETA, double &DTHETA, VECTOR3 &RT)
{
	VECTOR3 DR, UDR, UR, URT, RERT;
	double RTMAG;

	if (KABORT == false && opt.KDGUID == 1)
	{
		VECTOR3 YD = crossp(VA, RAAA);
		UYD = unit(YD);
	}
	DR = crossp(RAAA, UYD);
	UDR = unit(DR);
	UR = crossp(UYD, UDR);
	DTHETA = THETA;

	URT = UR * cos(DTHETA) + UDR * sin(DTHETA);
	RERT = URT * EQRAD;

	RTMAG = EQRAD + HTGT;

	RT = URT * RTMAG;
}

void DMP::PGOP3()
{
	//Test for initial pass
	if (KINIT)
	{
		//Do first pass PEG initialization
		INI1();
		if (KFLAG)
		{
			return;
		}
	}
	else
	{
		//Update velocity to go
		VG = VG - DVSS;
		DVSS = _V(0, 0, 0);
	}
	VGMAG = length(VG);
	if (VGMAG <= 0)
	{
		KFLAG = 3;
		IMSG = 8;
		return;
	}

	TAU = VEXX / ATR;
	TGO = TAU * (1.0 - exp(-VGMAG / VEXX));
	XLAM = VG / VGMAG;

	if (opt.KDGUID != 7)
	{
		//Thrust integrals
		TIJOL = TAU - VEXX * TGO / VGMAG;
		TIS = VGMAG * (TGO - TIJOL);
		QPRIME = VEXX * TGO*(0.5*TGO - TIJOL);
		TLAM = TTT + TIJOL;
		VECTOR3 VTEMP = crossp(XLAM, UYY);
		XLDA = VTEMP * XLDXZ;
		TP = TTT + TGO;
	}
	if (KDSTER)
	{
		TLAMC = TLAM;
		XLAMC = XLAM;
		XLAMDC = XLDA;
		ATP = XLAMC + XLAMDC * (TTT - TLAMC);
		ATP = unit(ATP);
	}
	if (opt.KDGUID != 7)
	{
		TPREV = TP;
		PRDT6();
		CORT7();
		RDA = RP;
	}
}

void DMP::INI1()
{
	if (opt.KDGUID == 3)
	{
		THETA_DOT = sqrt(XMU / pow(length(RAAA), 3));
		XLDXZ = THETA_DOT * CLMDXZ;
		KINIT = false;
		TGO = 0.0;
		VG = _V(0, 0, 0);
		IYAW = false;
		KSTOP = false;
		RP = RAAA;
		VP = VA;
		TP = TTT;
		CORT7();
	}
	else
	{
		XLDA = _V(0, 0, 0);
		TLAM = 0.0;
		KINIT = false;
	}
}

void DMP::PRDT6()
{
	VECTOR3 RC1, VC1, RGRAV, VGRAV, DVSI;
	double KJ2;

	RGO = XLDA * QPRIME + XLAM * TIS;
	RC1 = RAAA - RGO * 0.1 - VG * TGO / 30.0;
	VC1 = VA + RGO * 1.2 / TGO - VG * 0.1;
	DTAVG = TGO / NSEG;
	if (DTAVG > DTMAXX)
	{
		DTAVG = DTMAXX;
	}
	else if (DTAVG < DTMINN)
	{
		DTAVG = DTMINN;
	}
	KJ2 = 0.0;
	if (opt.INTEGF)
	{
		KJ2 = 1.0;
	}
	DVSI = _V(0, 0, 0);
	RC2 = RC1;
	VC2 = VC1;
	SUPRG(DTAVG, DVSI, KJ2, RC2, TTT, TP, VC2);
	VGRAV = VC2 - VC1;
	RGRAV = RC2 - RC1 - VC1 * TGO;
	RP = RAAA + VA * TGO + RGRAV + RGO;
	VP = VA + VGRAV + VG;
}

void DMP::CORT7()
{
	VECTOR3 VTEMP, VMISS, VGIP;
	double THETA, CK1, CK, RTMAG, VGOD;

	//Velocity-to-be-gained subtask
	VTEMP = crossp(VP, RP);
	UYY = unit(VTEMP);
	CK1 = dotp(RTA, UYY);
	RTA = RTA - UYY * CK1;
	RTMAG = length(RTA);
	CK = (RTMAG - HELIP(RTA) + HTGT) / RTMAG;
	RTA = RTA * CK;
	//VTEMP = -UYY;
	LTVCN2(CA[0], CA[1], RAAA, RTA, UYY, THETA, KFLAG, VDA);
	if (KFLAG) return;
	DTCOST = THETA / THETA_DOT;
	RHOO = 1.0 / (1.0 + RHOI * TGO / DTCOST);
	VMISS = VP - VDA;
	VG = VG - VMISS * RHOO;

	//Velocity-to-be-gain fuel depletion subtask
	VGIP = VG - UYY * dotp(VG, UYY);
	VG = VGIP;
	if (KFUELD)
	{
		double VGOYS;

		if (XMASSE > XMBO)
		{
			VGOD = VEXX * log(XMASSE / XMBO);
		}
		else
		{
			VGOD = 0.0;
		}
		VGOYS = VGOD * VGOD - dotp(VGIP, VGIP);
		if (VGOYS < 0)
		{
			VGOYS = 0.0;
		}
		VG = VGIP - UYY*KFUELD * sqrt(VGOYS);
	}

	//Convergence check subtask
	VGMAG = length(VG);
	CK = length(VMISS);
	CK1 = SMISS * VGMAG;
	if (CK < CK1)
	{
		//Solution converged
		KSTOP = true;
		if (IYAW == false)
		{
			IYAW = true;
			double TEMP = length(VGIP) / VGMAG;
			FWYAW = acos(TEMP);
		}
	}
	else
	{
		KSTOP = false;
	}
}

double DMP::HELIP(VECTOR3 R)
{
	return length(R);
}

void DMP::SUPRG(double DT, VECTOR3 DVS, double J2FLAG, VECTOR3 &RF, double TF, double TI, VECTOR3 &VF)
{
	VECTOR3 GPREV, GFINAL;
	double DTSTEP;
	int NSTEPS, I;

	NSTEPS = (int)(abs(TF - TI) / DT) + 1;
	DTSTEP = (TF - TI) / NSTEPS;
	GPREV = GRAVJ(RF, J2FLAG);

	I = 1;
	while (I <= NSTEPS)
	{
		RF = RF + (VF + GPREV * DTSTEP*0.5)*DTSTEP;
		GFINAL = GRAVJ(RF, J2FLAG);
		
		VF = VF + (GPREV + GFINAL)*DTSTEP*0.5 + DVS;
		RF = RF + (GFINAL - GPREV) / 6.0*DTSTEP*DTSTEP + DVS * DTSTEP*0.5;
		GPREV = GFINAL;
		I++;
	}
}

void DMP::LTVCN2(double C1, double C2, VECTOR3 R0, VECTOR3 R1, VECTOR3 XNUNIT, double &THETA, int &KFLAG, VECTOR3 &V0)
{
	double R0MAG, R1MAG, AX, BX, CX, DX, K, Z, W, VH1, VR1;

	//Set constants for normalizing, determine the transfer plane and transfer angle
	R0MAG = length(R0);
	R1MAG = length(R1);
	K = (R1MAG - R0MAG) / R0MAG;
	Z = R0MAG * R1MAG - dotp(R0, R1);
	W = dotp(crossp(R1, R0), XNUNIT) / Z;
	THETA = PI + atan2(-2.0*W, 1.0 - W * W);

	//Set coefficients for quadratic equation solution
	AX = K * (1.0 + W * W) + 2.0*(1.0 - C2 * W);
	BX = C1 * W;
	CX = 2.0*XMU / R1MAG;
	DX = BX * BX + AX*CX;
	if (DX < 0.0)
	{
		KFLAG = 1;
		IMSG = 7;
		return;
	}

	VH1 = CX / (sqrt(DX) - BX);
	VR1 = C1 + C2*VH1;
	V0 = R0 * (K*W*VH1 - VR1) + crossp(R0, XNUNIT)*(1.0 + K)*VH1 / R0MAG;
	KFLAG = 0;
}
*/

void DMP::DTT13()
{
	MATRIX3 TEMPMX, TEMP, TEF2MF;
	VECTOR3 XNAVR, XNAVV, XYZEDN, TEMPVC;
	double GLAT, XLON, XALT, TSTATE, ERAEI;

	TSTATE = TT;
	ERAEI = EARTHR * TT + ERAI;
	XNAVR = tmul(TM502I, RTA);
	XNAVV = tmul(TM502I, VTAA);

	//Compute Earth fixed XYZ from geodetic lat and long
	RLS = GD2EF(opt.TLATD, opt.TLONG, opt.TALTD);
	//Compute topodetic transformation matrix from Earth fixed
	TEMPMX = EF2TD(opt.TLATD, opt.TLONG);
	//Compute a z-axis rotation matrix through angle RAZ
	TEMP = ROTMX(-opt.RAZ, 3);
	//EF to runway
	REC = mul(TEMP, TEMPMX); 
	//Compute Earth relative velocity from EI vector
	XYZEDN = VREL(XNAVR, XNAVV);
	//Compute transformation matrix from Earth fixed to mean 50
	TEF2MF = EF2MF(TSTATE);
	//Convert M50 vector to Earth fixed
	XYZEN = tmul(TEF2MF, XNAVR);
	//Compute geodetic lat,long,alt from Earth fixed
	EF2GD(XYZEN, GLAT, XLON, XALT);
	//Convert Earth relative velocity vector to fixed?
	TEMPVC = tmul(TEF2MF, XYZEDN);
	//Compute transformation matrix from Earth fixed to topodetic
	TEMPMX = EF2TD(GLAT, XLON);
	//Convert Earth relative velocity vector to topodetic
	VRHOTD = mul(TEMPMX, TEMPVC);
	//Compute entry range
	EGRT();
	//Range in radians
	THETLS = TRANG * AMILE / RCHMAG;
	if (DELAZ < -PI05 || DELAZ > PI05)
	{
		THETLS = -THETLS;
	}
	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "DTT13: TRANG %lf THETLS %lf ERAEI %lf DELAZ %lf RCHMAG %lf", TRANG, THETLS*DEG, ERAEI, DELAZ, RCHMAG);
		oapiWriteLog(DebugBuffer);
	}
}

VECTOR3 DMP::GD2EF(double GLAT, double GLON, double ALT)
{
	double CLAT, SLAT, DUM, DUM1, REFEQT;

	CLAT = cos(GLAT);
	SLAT = sin(GLAT);
	DUM = pow(1.0 - ELLIPT, 2);
	DUM1 = EQRAD / sqrt(CLAT*CLAT + DUM * SLAT*SLAT);
	REFEQT = (DUM1 + ALT)*CLAT;
	return _V(REFEQT*cos(GLON), REFEQT*sin(GLON), (DUM*DUM1 + ALT)*SLAT);
}

MATRIX3 DMP::EF2TD(double GLAT, double XLON)
{
	MATRIX3 TM;
	double CLON, SLON, CLAT, SLAT;

	CLON = cos(XLON);
	SLON = sin(XLON);
	CLAT = cos(GLAT);
	SLAT = sin(GLAT);

	TM.m11 = -SLAT * CLON;
	TM.m21 = -SLON;
	TM.m31 = -CLAT * CLON;
	TM.m12 = -SLAT * SLON;
	TM.m22 = CLON;
	TM.m32 = -CLAT * SLON;
	TM.m13 = CLAT;
	TM.m23 = 0.0;
	TM.m33 = -SLAT;
	return TM;
}

MATRIX3 DMP::ROTMX(double ANGLE, int IAXIS)
{
	if (IAXIS == 1)
	{
		return _M(1.0, 0.0, 0.0, 0.0, cos(ANGLE), -sin(ANGLE), 0.0, sin(ANGLE), cos(ANGLE));
	}
	else if (IAXIS == 2)
	{
		return _M(cos(ANGLE), 0.0, sin(ANGLE), 0.0, 1.0, 0.0, -sin(ANGLE), 0.0, cos(ANGLE));
	}
	else
	{
		return _M(cos(ANGLE), -sin(ANGLE), 0.0, sin(ANGLE), cos(ANGLE), 0.0, 0.0, 0.0, 1.0);
	}
}

VECTOR3 DMP::VREL(VECTOR3 R, VECTOR3 V)
{
	VECTOR3 VR;

	VR = crossp(POLE, R);
	VR = V - VR * EARTHR;
	return VR;
}

MATRIX3 DMP::EF2MF(double TIME)
{
	double XLAM = TIME * EARTHR + ERAI;
	MATRIX3 X = _M(cos(XLAM), -sin(XLAM), 0.0, sin(XLAM), cos(XLAM), 0.0, 0.0, 0.0, 1.0);
	return mul(OrbMech::tmat(TM502I), X);
}

void DMP::EF2GD(VECTOR3 R, double &GLAT, double &XLON, double &XALT)
{
	double FLATCON, A_0, R_XY, r, SIN_P, COS_P, RAD_P, DEL, DEL_LAT, PHI;

	FLATCON = 1.0 - pow(1.0 - ELLIPT, 2);
	A_0 = 1.0 - FLATCON;
	R_XY = R.x*R.x + R.y*R.y;
	r = sqrt(R_XY + R.z*R.z);
	SIN_P = R.z / r;
	COS_P = sqrt(R_XY) / r;
	RAD_P = EQRAD / sqrt(1.0 + FLATCON * SIN_P*SIN_P / A_0);
	DEL = FLATCON * SIN_P*COS_P / (1.0 - FLATCON * COS_P*COS_P);
	DEL_LAT = RAD_P * DEL / r;
	PHI = atan2(SIN_P, COS_P);
	GLAT = PHI + DEL_LAT;
	XLON = atan2(R.y, R.x);
	XALT = (r - RAD_P) / (1.0 - 0.5*DEL*DEL_LAT);
}

void DMP::EGRT()
{
	VECTOR3 RC, XYZEA, RGEF, RG, HACEF, RCCEF, VNORM;
	double XWP2, XNEP, RVEHMG, T3, T4, BARVEH, SINB, T5, T6, BARCC, T7, CTHVC, STHVC, CTVWP1, SBARCR, A2, DVEWP1, T8, BARWP1, A3, DARC, RNGWP1, RNGWP2;

	//Find the center of heading alignment circle in runway coordinates
	XWP2 = 0.0;
	XNEP = -(DBARR - XWP2);
	RC.x = XNEP;
	RC.z = 0.0;
	XYZEA = XYZEN;
	RGEF = XYZEA - RLS;

	//Compute a topodetic position vector
	RG = mul(REC, RGEF);
	RC.y = SIGN(RTURNN, RG.y);
	HACEF = tmul(REC, RC);

	//Find bear vehicle
	RCCEF = RLS + HACEF;
	VNORM = crossp(XYZEA, RCCEF);
	RVEHMG = length(XYZEA);
	T3 = VNORM.z*RVEHMG;
	T4 = VNORM.x*XYZEA.y - VNORM.y*XYZEA.x;
	BARVEH = atan2(T3, T4);
	RCHMAG = length(RCCEF);
	SINB = RTURNN / RCHMAG;

	//Find bear vehicle to the center of heading alignment circle
	T5 = VNORM.z*RCHMAG;
	T6 = RCCEF.y*VNORM.x - VNORM.y*RCCEF.x;
	BARCC = atan2(T5, T6);
	if (BARCC < 0)
	{
		BARCC += PI2;
	}

	//Find cosine(THETA)
	T7 = dotp(XYZEA, RCCEF);
	CTHVC = T7 / (RVEHMG*RCHMAG);

	//Find the distance from vehicle to way point 1
	STHVC = sqrt(1.0 - CTHVC * CTHVC);
	CTVWP1 = CTHVC + 0.5*CTHVC*SINB*SINB;
	SBARCR = SINB / STHVC;
	A2 = acos(RLIMIT(CTVWP1*SBARCR, -1.0, 1.0));
	DVEWP1 = acos(RLIMIT(CTVWP1, -1.0, 1.0))*RCHMAG;

	//Find the distance from vehicle to normal entry point
	T8 = asin(RLIMIT(SBARCR, -1.0, 1.0));
	BARWP1 = BARVEH - SIGN(T8, RC.y);
	A3 = PI05 - A2 + SIGN(opt.RAZ - BARCC, RC.y);
	if (A3 < -0.003)
	{
		A3 += PI2; //Only in MCC memo
	}
	DARC = A3 * RTURNN;
	RNGWP1 = (DVEWP1 + DARC) / AMILE;
	RNGWP2 = RNGWP1 + (XWP2 - XNEP) / AMILE;
	TRANG = RNGWP2;
	double PSI = atan2(VRHOTD.y, VRHOTD.x);
	DELAZ = PSI - BARWP1;
	if (abs(DELAZ) > PI)
	{
		DELAZ = DELAZ - PI2 * SIGN(1.0, DELAZ);
	}
	if (BARWP1 < 0)
	{
		BARWP1 += PI2;
	}
	if (PSI < 0)
	{
		PSI += PI2;
	}
	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "EGRT: TRANG %lf DELAZ %lf", TRANG, DELAZ);
		oapiWriteLog(DebugBuffer);
	}
}

void DMP::DTT14()
{
	if (IPRNG)
	{
		ETGRNG = opt.RNGIP;
	}
	else
	{
		double VEIM;

		WBO = opt.WT - DWPR;
		if (IFINAL)
		{
			if (ICALL)
			{
				IPRETG = 1;
				VEIM = length(VTAA);
				FVE(VEIM, THELSD, GAMETG);
				ETGRNG = THELSD * RCHMAG;
				if (IC2FLG)
				{
					UpdateC1C2(VEIM, CA[0], CA[1]);
				}
				else
				{
					CA[0] = VEIM * sin(GAMETG) - CA[1] * cos(GAMETG);
				}
			}
		}
		else
		{
			if (ICALL)
			{
				IPRETG = 0;
				VEIM = length(VTAA);
				FVE(VEIM, THELSD, GAMETG);
				ETGRNG = THELSD * RCHMAG;
				if (IC2FLG)
				{
					UpdateC1C2(VEIM, CA[0], CA[1]);
				}
				else
				{
					CA[0] = VEIM * sin(GAMETG) - CA[1] * cos(GAMETG);
				}
			}
		}
		if (opt.IPOUT == 2)
		{
			sprintf_s(DebugBuffer, "DTT14: VEIM %lf GAMETG %lf THELSD %lf C1 %lf C2 %lf", VEIM, GAMETG, THELSD, CA[0], CA[1]); //TEIFVE?
			oapiWriteLog(DebugBuffer);
		}
	}
	//ICALL = false; //Don't do this for now
	THELSD = ETGRNG / RCHMAG;
}

void DMP::UpdateC1C2(double VE, double &C1, double &C2)
{
	double VV1, VV2, VH1, VH2, DTAE, FPAE;

	//Calculate data at desired velocity
	FVE(VE, DTAE, FPAE);
	VV1 = VE * sin(FPAE);
	VH1 = VE * cos(FPAE);

	//Bias velocity
	VE += 0.5;

	//Calculate data at bias velocity
	FVE(VE, DTAE, FPAE);
	VV2 = VE * sin(FPAE);
	VH2 = VE * cos(FPAE);

	//Calculate C1 and C2
	C1 = (VV2*VH1 - VV1 * VH2) / (VH1 - VH2);
	C2 = (VV1 - C1) / VH1;
}

void DMP::FVE(double VE, double &DTAE, double &FPAE)
{
	double RAN, X;
	int i, j;

	FPAE = 0.0;
	RAN = 0.0;

	if (VE < FVECON_VEL[0] || VE > FVECON_VEL[FVECON_LENGTH - 1])
	{
		//TBD: Error message "Entry velocity out of range in FVE"
	}
	/*if (VE < FVECON_VEL[0])
	{
		VE = FVECON_VEL[0];
	}
	else if (VE > FVECON_VEL[FVECON_LENGTH - 1])
	{
		VE = FVECON_VEL[FVECON_LENGTH - 1];
	}*/

	for (j = 0;j < FVECON_LENGTH;j++)
	{
		X = 1.0;
		for (i = 0;i < FVECON_LENGTH;i++)
		{
			if (i != j)
			{
				X = X * (VE - FVECON_VEL[i]) / (FVECON_VEL[j] - FVECON_VEL[i]);
			}
		}
		RAN = RAN + X * FVECON_RNGG[j];
		FPAE = FPAE + X * FVECON_FPA[j];
	}
	DTAE = RAN / RCHMAG;
}

void DMP::DTT15()
{
	if (IBTR == 1)
	{
		IBTR = 0;
	}
	else
	{
		opt.TIG = opt.TIG + (THETLS - THELSD) / THETDR;
	}
}

void DMP::DTT16()
{
	if (IBTR == 1)
	{
		IBTR = 0;
	}
	else
	{
		THETEI = THETEI + THETLS - THELSD;
	}
}

void DMP::DTT17()
{
	//Save EI vector
	TEIS = TT;
	REIS = RTA;
	VEIS = VTAA;

	//Reset flags
	IBTR = 1;
	NSYS = 2;
}

void DMP::DTT18()
{
	double DTIG;

	//Increase TIG to increase free fall time
	DTIG = DTCOST - opt.TFFMIN - TFFTOL / 2.0;
	opt.TIG = opt.TIG + DTIG;
	//Compute THETEI for new TIG
	THETEI = THETEI - DTIG * THETD;
}

void DMP::DTT21()
{
	if (NCNT1 != 0)
	{
		opt.TIG = opt.TIG + PI2 / THETD;
		CA[0] = opt.C1IP;
		CA[1] = opt.C2IP;
		IPRNG = true;
		INCR = false;
	}
	NCNT1++;
	if (NCNT1 >= NCMAX)
	{
		MITER = 9;
		IMSG = 1;
	}
}

void DMP::DTT22()
{
	double VDR, VM, RM, GAMS;

	//Compute a conic gamma from the PEG EI vector
	VDR = dotp(VEIS, REIS);
	VM = length(VEIS);
	RM = length(REIS);
	GAMS = asin(VDR / VM / RM);

	//Compute conic vertical and horizontal EI velocities
	VVS = VM * sin(GAMS);
	VHS = VM * cos(GAMS);
}

void DMP::DTT23()
{
	double VENKM, RENKM, VDR, GAMENK, VVENK, VHENK, VVCOR, GAMCOR;

	//Compute EI gamma from vector integrated to EI
	VENKM = length(VENK);
	RENKM = length(RENK);
	VDR = dotp(VENK, RENK);
	GAMENK = asin(VDR / VENKM / RENKM);
	//Compute vertical and horizontal velocities at EI
	VVENK = VENKM * sin(GAMENK);
	VHENK = VENKM * cos(GAMENK);
	//Compute required gamma to lie on the VV-VH target line described by C1 and C2
	VVCOR = VVS + CA[1] * (VHENK - VHS);
	GAMCOR = asin(VVCOR / VENKM);
	//Compute gamma error and C1 correction
	DGAM = abs(GAMENK - GAMCOR);
	DC11 = VVCOR - VVENK;
}

/*void DMP::SUPRJ(VECTOR3 &RC2, VECTOR3 &VC2, double TGO)
{
	VECTOR3 DVS, GPREV, GFINAL;
	double TI, J2, DTSTEP, DTAVG;
	int NSTEPS;

	DVS = _V(0, 0, 0);
	TI = 0.0;
	J2 = 1.0;
	DTAVG = 2.0;
	NSTEPS = (int)(abs(TGO - TI) / DTAVG) + 1;
	DTSTEP = (TGO - TI) / (double)NSTEPS;

	GPREV = GRAVJ(RC2, J2);

	for (int i = 0;i < NSTEPS;i++)
	{
		RC2 = RC2 + (VC2 + GPREV * 0.5*DTSTEP)*DTSTEP;
		GFINAL = GRAVJ(RC2, J2);
		VC2 = VC2 + (GPREV + GFINAL)*0.5 + DVS;
		RC2 = RC2 + (GFINAL - GPREV)*DTSTEP*DTSTEP / 6.0 + DVS * 0.5*DTSTEP;
		GPREV = GFINAL;
	}
	if (opt.IPOUT == 2)
	{
		//TBD: Print RC2, VC2, TGO, DTAVG
	}
}*/

VECTOR3 DMP::GRAVJ(VECTOR3 RF, double J2)
{
	VECTOR3 UR, GRAV, GJ2;
	double RFMAG, SLAT, CK, CK1;

	//Compute magnitude and direction unit vector of the input vector
	RFMAG = length(RF);
	UR = unit(RF);
	//Compute intermediate variables for J2 GRAV update
	SLAT = dotp(POLE, UR);
	CK = XJ2 * XMU*EQRAD*EQRAD / pow(RFMAG, 4);
	CK1 = 0.5*(15.0*SLAT*SLAT - 3.0);
	//Compute spherical Earth gravity vector
	GRAV = -UR * XMU / pow(RFMAG, 2);
	//Compute gravity vector with J2 effects included
	GJ2 = (UR*CK1 - POLE * SLAT*3.0)*CK;
	GRAV = GRAV + GJ2 * J2;
	return GRAV;
}

void DMP::DTT24()
{
	VECTOR3 UREI, H, UM, ULS, E, UR;
	double REA, THETDT, T, LAMDAI, THETA, ES, DELTAT, TSLP, TSLPP, ESPAST, TDEN, TPAST;

	//Compute local horizontal at EI
	UREI = unit(RTA);
	H = crossp(VTAA, RTA);
	H = unit(H);
	UM = crossp(UREI, H);
	UM = unit(UM);
	//Compute orbital rate
	REA = length(RTA);
	THETDT = sqrt(XMU / pow(REA, 3));
	NCNT = 0;
	T = THETLS / THETDT;
	//Predict time of closest approach
	TC = TT + T;
	//Predict landing site location at time of closest approach
	LAMDAI = opt.TLONG + EARTHR * TC + ERAI;
	ULS.x = cos(TLATC)*cos(LAMDAI);
	ULS.y = cos(TLATC)*sin(LAMDAI);
	ULS.z = sin(TLATC);
	//Compute closest approach test criteria
	THETA = THETDT * T;
	E = -UM * cos(THETA) + UREI * sin(THETA);
	ES = dotp(ULS, E);
	while (abs(ES) >= CRTOL)
	{
		if (NCNT == 0)
		{
			//First pass
			DELTAT = DTCR0;
			TSLPP = DTCR0 / ES;
		}
		else
		{
			TDEN = ES - ESPAST;
			if (abs(TDEN) < TDENMN)
			{
				TSLP = TSLPP;
			}
			else
			{
				TSLP = (T - TPAST) / (ES - ESPAST);
				TSLPP = TSLP;
			}
			DELTAT = -ES * TSLP;
		}
		//Save data for next pass
		TPAST = T;
		ESPAST = ES; //Document has -ES
		T = T + DELTAT;
		NCNT++;
		//Predict time of closest approach
		TC = TT + T;
		//Predict landing site location at time of closest approach
		LAMDAI = opt.TLONG + EARTHR * TC + ERAI;
		ULS.x = cos(TLATC)*cos(LAMDAI);
		ULS.y = cos(TLATC)*sin(LAMDAI);
		ULS.z = sin(TLATC);
		//Compute closest approach test criteria
		THETA = THETDT * T;
		E = -UM * cos(THETA) + UREI * sin(THETA);
		ES = dotp(ULS, E);
		if (NCNT >= NCMAX) break;
	}
	//Compute crossrange and downrange
	CRSRNG = asin(dotp(ULS, H))*RCHMAG / AMILE;
	UR = UREI * cos(THETA) + UM * sin(THETA);
	DNRNG = acos(dotp(UR, UREI))*RCHMAG / AMILE;
	if (opt.IPOUT == 2)
	{
		sprintf_s(DebugBuffer, "DTT24: TC %lf THETLS %lf THETDT %lf DELTAT %lf DTCR0 %lf T %lf CRSRNG %lf", TC, THETLS*DEG, THETDT, DELTAT, DTCR0, T, CRSRNG);
		oapiWriteLog(DebugBuffer);
	}
}

void DMP::DTMOT(DMPResults &res)
{
	//EI state vector
	res.sv_EI.R = RTA;
	res.sv_EI.V = VTAA;
	res.sv_EI.GMT = TT;
	res.sv_EI.mass = opt.WT;
}

void DMP::DTMPR(DMPResults &res)
{
	res.TIG = opt.TIG;
	res.C1 = CA[0] / 0.3048;
	res.C2 = CA[1];
	res.THETEI = THETEI * DEG;
	res.EIALT = EIALT / 1852.0;

	res.DVPR = DVPR / 0.3048;

	res.VEI = length(VTAA) / 0.3048;
	res.cEI = GAMETG * DEG; //Correct one?
	res.REI = ETGRNG / 1852.0;

	char Left;

	if (CRSRNG < 0)
	{
		Left = 'L';
	}
	else
	{
		Left = 'R';
	}

	char Buffer[128];

	sprintf(Buffer, "%c%.0lf", Left, abs(CRSRNG));
	res.XR.assign(Buffer);

	res.OOP = PSIPR*DEG;
	res.TFF = TFFPR;

	double r_apo, r_peri;
	OrbMech::periapo(RTA, VTAA, XMU, r_apo, r_peri);
	res.HP = (r_peri - OrbMech::EARTH_RADIUS_EQUATOR) / 1852.0;

	res.VGO = VGO / 0.3048;
	res.DW = DWPR / OrbMech::LBM2KG;

	res.WCG = opt.WCGOMS*SBD / OrbMech::LBM2KG;
}