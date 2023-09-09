/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2023 Niklas Beug

  PEG4 Targeting (Header)

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

#include "Orbitersdk.h"

class PEG4
{
public:
	PEG4();
	bool OMSBurnPrediction(VECTOR3 RGD, VECTOR3 VGD, double TGD, VECTOR3 RLS_TEG, double C1, double C2, double HTGT, double THETA, double FT, double VEX, double M, bool ops1);
	void GetOutputA(VECTOR3 &RP, VECTOR3 &VD, VECTOR3 &VGO, double &TGO, double &MBO);
	void GetOutputD(VECTOR3 &RP, VECTOR3 &VD, VECTOR3 &VGO, double &TGO, double &MBO, double &DT, VECTOR3 &REI, VECTOR3 &VEI, double &FWYAW, VECTOR3 &VMISS);
protected:
	VECTOR3 HTHETA_M50_TGT_TSK(VECTOR3 RLS_TEG);
	bool VelocityToBeGainedSubtask();
	void VelocityToBeGainedFuelDepletionSubtask();
	void TimeToGoSubtask();
	void ThrustIntegralSubtask();
	void ReferenceThrustVectorsSubtask();
	void BurnoutStateVectorPredictionSubtask();
	void ConvergenceCheckSubtask();

	bool LTVC(double C1, double C2, VECTOR3 RP, VECTOR3 RT, VECTOR3 IY, double &THETA, VECTOR3 &VD);
	void ENTRY_PRECISE_PREDICTOR(VECTOR3 R_INIT, VECTOR3 V_INIT, double T_INIT, double T_FINAL, double DT_MAX, int GMD_PRED, VECTOR3 &R_FINAL, VECTOR3 &V_FINAL);
	void CENTRAL(VECTOR3 R, VECTOR3 &ACCEL, double &R_INV);
	VECTOR3 ACCEL_ENTRY(VECTOR3 R);

	//INPUT
	//Ignition state vector
	VECTOR3 RGD, VGD;
	double TGD;
	//PEG target
	double C1, C2, HTGT, THETA;
	double FT, VEX, M;

	//OUTPUT
	VECTOR3 VD;
	double MBO;
	double DT; //Time from burnout to EI
	VECTOR3 V_EI; //Velocity vector at EI

	//LOCAL
	//Target position vector
	VECTOR3 RT;
	//Cutoff state vector
	VECTOR3 RP, VP;
	double TP;
	double TGO;
	VECTOR3 VGO;
	double THETA_DOT, LAMDXZ;
	bool SCONV;
	double ATR;
	VECTOR3 IY;
	VECTOR3 VMISS;
	double VGOMAG;
	double VRATIO;
	double JOL, S, QPRIME;
	VECTOR3 LAM, LAMD;
	VECTOR3 RC1, VC1, RC2, VC2;
	double SFUELD;
	VECTOR3 VGIP;

	const double K_LAMDXZ = 0.0;
	const double EARTH_MU = 398600439968871.2;
	const double RADIUS_EARTH_EQUATOR = 6378166.0;
	const double KMISS = 0.01;
	const double JArr[4] = { 0.0, 1082.6269e-6, -2.51e-6, -1.60e-6 };
	const double RE = 6.37101e6;
	const double EP_TRANSFER = 8.0*RAD;
};