/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2023 Niklas Beug

  PEG4 Targeting

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

#include "PEG4.h"

PEG4::PEG4()
{

}

bool PEG4::OMSBurnPrediction(VECTOR3 RGD, VECTOR3 VGD, double TGD, VECTOR3 RLS_TEG, double C1, double C2, double HTGT, double THETA, double FT, double VEX, double M)
{
	this->RGD = RGD;
	this->VGD = VGD;
	this->TGD = TGD;
	this->RLS_TEG = RLS_TEG;
	this->C1 = C1;
	this->C2 = C2;
	this->HTGT = HTGT;
	this->THETA = THETA;
	this->FT = FT;
	this->VEX = VEX;
	this->M = M;

	RT = HTHETA_M50_TGT_TSK();

	RP = RGD;
	VP = VGD;
	TP = TGD;
	THETA_DOT = sqrt(EARTH_MU / (pow(length(RGD), 3)));
	LAMDXZ = THETA_DOT * K_LAMDXZ;
	TGO = 0;
	VGO = _V(0, 0, 0);

	SCONV = false;

	//Init
	if (VelocityToBeGainedSubtask()) return true;

	ATR = FT / M;

	while (SCONV == false)
	{
		VGOMAG = length(VGO);
		TimeToGoSubtask();
		ThrustIntegralSubtask();
		ReferenceThrustVectorsSubtask();
		BurnoutStateVectorPredictionSubtask();
		if (VelocityToBeGainedSubtask()) return true;
		ConvergenceCheckSubtask();
	}

	VECTOR3 RJ2, VJ2, RC3, VC3;
	ENTRY_PRECISE_PREDICTOR(RC1, VC1, TGD, TGD + TGO, 2.0, 2, RC3, VC3);
	VJ2 = VC3 - VC2;
	RJ2 = RC3 - RC2;
	RP = RP + RJ2;
	VD = VD + VJ2;
	MBO = M - FT / VEX * TGO;
	return false;
}

void PEG4::GetOutput(VECTOR3 &RP, VECTOR3 &VD, VECTOR3 &VGO, double &TGO, double &MBO)
{
	RP = this->RP;
	VD = this->VD;
	VGO = this->VGO;
	TGO = this->TGO;
	MBO = this->MBO;
}

VECTOR3 PEG4::HTHETA_M50_TGT_TSK()
{
	VECTOR3 IDR, IR, IRT, RT;
	double THETA_LS, DTHETA, RTMAG;

	IDR = unit(crossp(RGD, crossp(VGD, RGD)));
	IR = unit(RGD);
	THETA_LS = atan2(-dotp(RLS_TEG, IDR), dotp(RLS_TEG, IR));
	if (THETA_LS < 0)
	{
		THETA_LS = THETA_LS + PI2;
	}
	DTHETA = THETA - THETA_LS;
	IRT = IR * cos(DTHETA) + IDR * sin(DTHETA);
	RTMAG = HTGT + RADIUS_EARTH_EQUATOR;
	RT = IRT * RTMAG;
	return RT;
}

bool PEG4::VelocityToBeGainedSubtask()
{
	double DTCOAST, RHOMAG;

	IY = unit(crossp(VP, RP));
	//RT = RT - IY * dot(RT, IY);
	if (LTVC(C1, C2, RP, RT, IY, THETA, VD)) return true;
	DTCOAST = THETA / THETA_DOT;
	RHOMAG = 1.0 / (1.0 + 0.75*TGO / DTCOAST);
	VMISS = VP - VD;
	VGO = VGO - VMISS * RHOMAG;
	return false;
}

void PEG4::TimeToGoSubtask()
{
	VRATIO = VGOMAG / (6.0 * VEX);
	TGO = VGOMAG / (ATR*(1.0 + 3.0 * VRATIO + 3.0 * VRATIO*VRATIO));
	TP = TGD + TGO;
}

void PEG4::ThrustIntegralSubtask()
{
	JOL = 0.5*TGO*(1.0 + VRATIO);
	S = 0.5*VGOMAG*TGO*(1.0 - VRATIO);
	QPRIME = VGOMAG * TGO*TGO / 12.0;
}

void PEG4::ReferenceThrustVectorsSubtask()
{
	LAM = VGO / VGOMAG;
	LAMD = crossp(LAM, IY)*LAMDXZ;
}

void PEG4::BurnoutStateVectorPredictionSubtask()
{
	VECTOR3 RGO, RGRAV, VGRAV;

	RGO = LAMD * QPRIME + LAM * S;
	RC1 = RGD - RGO / 10.0 - VGO * TGO / 30.0;
	VC1 = VGD + RGO * 6.0 / 5.0 / TGO - VGO / 10.0;
	ENTRY_PRECISE_PREDICTOR(RC1, VC1, TGD, TP, 2.0, 0, RC2, VC2);
	VGRAV = VC2 - VC1;
	RGRAV = RC2 - RC1 - VC1 * TGO;
	RP = RGD + VGD * TGO + RGRAV + RGO;
	VP = VGD + VGRAV + VGO;
}

void PEG4::ConvergenceCheckSubtask()
{
	VGOMAG = length(VGO);
	if (length(VMISS) <= KMISS * VGOMAG)
	{
		SCONV = true;
	}
	else
	{
		SCONV = false;
	}
}

bool PEG4::LTVC(double C1, double C2, VECTOR3 RP, VECTOR3 RT, VECTOR3 IY, double &THETA, VECTOR3 &VD)
{
	double R0_MAG, R1_MAG, K, Z, W, A, B, C, D, VH1, VR1;

	R0_MAG = length(RP);
	R1_MAG = length(RT);
	K = (R1_MAG - R0_MAG) / R0_MAG;
	Z = R0_MAG * R1_MAG - dotp(RP, RT);

	if (Z <= R0_MAG*R1_MAG*EP_TRANSFER)
	{
		return true;
	}

	W = dotp(crossp(RT, RP), IY) / Z;
	THETA = PI + atan2(-2.0 * W, 1.0 - W * W);
	A = K * (1.0 + W * W) + 2.0 * (1.0 - C2 * W);
	B = C1 * W;
	C = 2 * EARTH_MU / R1_MAG;
	D = B * B + A * C;

	VH1 = C / (sqrt(D) - B);
	VR1 = C1 + C2 * VH1;
	VD = (RP*(K*W*VH1 - VR1) + crossp(RP, IY)*(1.0 + K)*VH1) / R0_MAG;
	return false;
}

void PEG4::ENTRY_PRECISE_PREDICTOR(VECTOR3 R_INIT, VECTOR3 V_INIT, double T_INIT, double T_FINAL, double DT_MAX, int GMD_PRED, VECTOR3 &R_FINAL, VECTOR3 &V_FINAL)
{
	VECTOR3 G_PREVIOUS, G_FINAL;
	double STEP_SIZE, T, R_INV;
	int NUMBER_STEPS;

	R_FINAL = R_INIT;
	V_FINAL = V_INIT;

	NUMBER_STEPS = (int)max(round(abs(T_FINAL - T_INIT) / DT_MAX), 1);
	STEP_SIZE = (T_FINAL - T_INIT) / NUMBER_STEPS;
	T = T_INIT;

	if (GMD_PRED == 0)
	{
		CENTRAL(R_FINAL, G_PREVIOUS, R_INV);
	}
	else
	{
		G_PREVIOUS = ACCEL_ENTRY(R_FINAL);
	}

	for (int I = 1; I <= NUMBER_STEPS; I++)
	{
		T = T + STEP_SIZE;
		R_FINAL = R_FINAL + (V_FINAL + G_PREVIOUS * 0.5*STEP_SIZE)*STEP_SIZE;

		if (GMD_PRED == 0)
		{
			CENTRAL(R_FINAL, G_FINAL, R_INV);
		}
		else
		{
			G_FINAL = ACCEL_ENTRY(R_FINAL);
		}

		V_FINAL = V_FINAL + (G_PREVIOUS + G_FINAL)*STEP_SIZE*0.5;
		R_FINAL = R_FINAL + (G_FINAL - G_PREVIOUS)*STEP_SIZE * STEP_SIZE / 6.0;
		G_PREVIOUS = G_FINAL;
	}
}

void PEG4::CENTRAL(VECTOR3 R, VECTOR3 &ACCEL, double &R_INV)
{
	R_INV = 1.0 / length(R);
	ACCEL = -R * EARTH_MU * pow(R_INV, 3);
}

VECTOR3 PEG4::ACCEL_ENTRY(VECTOR3 R)
{
	VECTOR3 ACCEL, U_R, U_Z, G_B;
	double R_INV, cos_lat, P[4];

	CENTRAL(R, ACCEL, R_INV);

	U_R = R * R_INV;
	U_Z = _V(0, 0, 1);
	cos_lat = dotp(U_R, U_Z);

	P[0] = 3.0 * cos_lat;
	P[1] = 0.5*(15.0 * cos_lat*cos_lat - 3.0);
	P[2] = 1.0 / 3.0 * (7.0 * cos_lat*P[1] - 4.0 * P[0]);
	P[3] = 1.0 / 4.0 * (9.0 * cos_lat*P[2] - 5.0 * P[1]);

	G_B = _V(0, 0, 0);
	for (int i = 2; i <= 4; i++)
	{
		G_B = G_B + (U_R*P[i - 1] - U_Z * P[i - 2])* pow(RE*R_INV, i)*JArr[i - 1];
	}

	G_B = G_B*EARTH_MU * pow(R_INV, 2);
	ACCEL = ACCEL + G_B;
	return ACCEL;
}