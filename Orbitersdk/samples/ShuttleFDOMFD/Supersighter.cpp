/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Supersighter Display

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

#include "Supersighter.h"
#include "OrbMech.h"
#include "brent/brent.h"
#include <iostream>
#include <string> 

MATRIX3 operator+ (const MATRIX3& A, const MATRIX3 B)
{
	MATRIX3 mat = { A.m11 + B.m11, A.m12 + B.m12, A.m13 + B.m13,
				    A.m21 + B.m21, A.m22 + B.m22, A.m23 + B.m23,
				    A.m31 + B.m31, A.m32 + B.m32, A.m33 + B.m33 };
	return mat;
}

MATRIX3 operator- (const MATRIX3& A, const MATRIX3 B)
{
	MATRIX3 mat = { A.m11 - B.m11, A.m12 - B.m12, A.m13 - B.m13,
					A.m21 - B.m21, A.m22 - B.m22, A.m23 - B.m23,
					A.m31 - B.m31, A.m32 - B.m32, A.m33 - B.m33 };
	return mat;
}

SupersighterInputs::SupersighterInputs()
{
	Mode = 1;
	INMAT = "RFMT01";
	OUTMAT = "RFMT01";

	ELV = 0.0;
	ATTSense = 0;
	ATT = _V(0, 0, 0);
	IA1_A1 = IA1_A2 = IA2_A1 = IA2_A2 = IB1_A1 = IB1_A2 = IB2_A1 = IB2_A2 = 0.0;
	MGA = 0.0;
	OMI = 0.0;
	EIG = _V(0, 0, 0);
	StartTime = 0.0;

	IDT = NULL;
	IMT = NULL;
	CTF = NULL;
	GTF = NULL;
	sescnst = NULL;
	useNonSphericalGravity = false;
}

Supersighter::Supersighter()
{
	sprintf(Buffer, "");
	R_GS_EF = _V(0, 0, 0);
	TargetNumber = 0;
}

void Supersighter::RUN(const SupersighterInputs& in, SupersighterOutputs& out)
{
	inp = in;

	// Propagate state vector to start time
	double dt = GMTfromMET(inp.StartTime) - inp.sv0.GMT;
	sv_ST = OrbMech::coast_auto(inp.sv0, dt, inp.useNonSphericalGravity);

	switch (in.Mode)
	{
	case 1:
		MODE1();
		break;
	case 2:
		MODE2();
		break;
	case 3:
		MODE3();
		break;
	case 4:
		MODE4();
		break;
	case 5:
		MODE5();
		break;
	case 6:
		MODE6();
		break;
	case 7:
		MODE7();
		break;
	}

	out = outp;
}

void Supersighter::MODE1()
{
	// Find AOS/TCA/LOS for TGT 1 for the input ELV angle, using state vector and ST
	// Output input attitude

	MATRIX3 B_M50_BY;
	VECTOR3 u_TGT_TEG, u_TGT_M50, u_TGT_BY, R_M50, V_M50;
	double A1, A2, A3;
	int type, number, inst, A3Err;
	bool Limit1, Limit2;
	OrbMech::SV sv_AOS;
	double GMT_TCA, GMT_LOS;

	if (DecodeTarget(inp.TGT1, type, number))
	{
		outp.ErrorMessage = "Error: TGT1 ID invalid";
		return;
	}

	if (DecodeInstrument(inp.IA1, inst))
	{
		outp.ErrorMessage = "Error: IA1 ID invalid";
		return;
	}

	if (FindAOS(sv_ST, type, number, sv_AOS, GMT_TCA, GMT_LOS))
	{
		outp.ErrorMessage = "Error: AOS not found";
		return;
	}

	// Calculate body attitude matrix
	if (CalculateBodyMatrixFromAttitude(sv_AOS, B_M50_BY))
	{
		// TBD: Error
		return;
	}

	// Format outputs from inputs
	outp.MODE = std::to_string(inp.Mode);
	outp.INMAT = inp.INMAT;
	outp.OUTMAT = inp.OUTMAT;
	outp.ATT_SOURCE = "MED";
	outp.ATT_SENSE = FormatAttSense();
	outp.INPUT_ATT[0] = FormatAttitude(inp.ATT.x);
	outp.INPUT_ATT[1] = FormatAttitude(inp.ATT.y);
	outp.INPUT_ATT[2] = FormatAttitude(inp.ATT.z);
	outp.ELV = FormatInstrumentAngle(inp.ELV);

	// Calculate instrument pointing angles
	u_TGT_TEG = GetTargetDirection(sv_AOS, type, number);
	u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
	u_TGT_BY = mul(B_M50_BY, u_TGT_M50);
	GetInstrumentAnglesFromVector(u_TGT_BY, inst, A1, A2, Limit1, Limit2);
	R_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv_AOS.R);
	V_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv_AOS.V);
	A3Err = ComputeOmicron(R_M50, V_M50, B_M50_BY, u_TGT_M50, A3);

	// Format instrument 1 output
	outp.IA1 = inp.IA1;
	outp.IA1_A1 = FormatInstrumentAngle(A1 * DEG);
	outp.IA1_A1_LIM = FormatInstrumentLimit(Limit1);
	outp.IA1_A2 = FormatInstrumentAngle(A2 * DEG);
	outp.IA1_A2_LIM = FormatInstrumentLimit(Limit2);
	if (A3Err == false)
	{
		outp.IA1_A3 = FormatInstrumentAngle(A3 * DEG);
	}
	outp.IA1_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);

	// Format AOS/TCA/LOS
	outp.AOS_GMT = GMT2String(sv_AOS.GMT);
	outp.AOS_MET = MET2String(METfromGMT(sv_AOS.GMT));
	outp.TCA_GMT = GMT2String(GMT_TCA);
	outp.TCA_MET = MET2String(METfromGMT(GMT_TCA));
	outp.LOS_GMT = GMT2String(GMT_LOS);
	outp.LOS_MET = MET2String(METfromGMT(GMT_LOS));

	// Show more inputs
	if (type <= 3)
	{
		// Celestial
		FormatCelestialTarget(u_TGT_M50, outp.TGT1_RA, outp.TGT1_DEC);
	}
	else if (type == 4)
	{
		// Ground
		FormatGroundTarget(sv_AOS, number);
	}
	else if (type == 5)
	{
		// Vehicle
		FormatVehicleTarget(sv_AOS);
	}

	outp.TGT1 = inp.TGT1;

	// Common outputs
	CommonCalculations(sv_AOS, B_M50_BY);

	if (CalculateAttitudeFromBodyMatrix(sv_AOS, B_M50_BY, true))
	{
		// TBD: Error
		return;
	}

	// Optional second target
	if (inp.TGT2 != "")
	{
		if (DecodeTarget(inp.TGT2, type, number, false))
		{
			outp.ErrorMessage = "Error: TGT2 ID invalid";
			return;
		}

		if (DecodeInstrument(inp.IA2, inst))
		{
			outp.ErrorMessage = "Error: IA2 ID invalid";
			return;
		}

		u_TGT_TEG = GetTargetDirection(sv_AOS, type, number);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		u_TGT_BY = mul(B_M50_BY, u_TGT_M50);
		GetInstrumentAnglesFromVector(u_TGT_BY, inst, A1, A2, Limit1, Limit2);

		// Format instrument 2 output
		outp.IA2 = inp.IA2;
		outp.IA2_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IA2_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IA2_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IA2_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IA2_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);

		outp.TGT2 = inp.TGT2;
		FormatCelestialTarget(u_TGT_M50, outp.TGT2_RA, outp.TGT2_DEC);
	}
}

void Supersighter::MODE2()
{
	// Fixed-attitude/fixed line-of-sight mode

	MATRIX3 B_M50_BY;
	VECTOR3 u_BY, u_TGT_M50, u_TGT_TEG;
	int inst;
	bool Limit1, Limit2;

	if (CalculateBodyMatrixFromAttitude(sv_ST, B_M50_BY))
	{
		// TBD: Error
		return;
	}

	if (DecodeInstrument(inp.IA1, inst))
	{
		outp.ErrorMessage = "Error: IA1 ID invalid";
		return;
	}

	u_BY = GetVectorFromInstrumentAngles(inst, inp.IA1_A1 * RAD, inp.IA1_A2 * RAD);

	// Calculate inertial pointing direction
	u_TGT_M50 = tmul(B_M50_BY, u_BY);
	u_TGT_TEG = tmul(inp.sescnst->M_TEG_TO_M50, u_TGT_M50);

	// Format outputs
	outp.MODE = std::to_string(inp.Mode);
	outp.INMAT = inp.INMAT;
	outp.OUTMAT = inp.OUTMAT;
	outp.ATT_SOURCE = "MED";
	outp.ATT_SENSE = FormatAttSense();
	outp.INPUT_ATT[0] = FormatAttitude(inp.ATT.x);
	outp.INPUT_ATT[1] = FormatAttitude(inp.ATT.y);
	outp.INPUT_ATT[2] = FormatAttitude(inp.ATT.z);
	outp.IA1 = inp.IA1;
	outp.IA1_A1 = FormatInstrumentAngle(inp.IA1_A1);
	outp.IA1_A2 = FormatInstrumentAngle(inp.IA1_A2);
	outp.IA1_OCC = OccultationCalculations(sv_ST.R, sv_ST.GMT, u_TGT_TEG);
	FormatCelestialTarget(u_TGT_M50, outp.TGT1_RA, outp.TGT1_DEC);

	CommonCalculations(sv_ST, B_M50_BY);

	// Special latitude and longitude calculation
	double Lat, Lng;
	if (Mode2EarthIntersection(sv_ST.R, sv_ST.GMT, u_TGT_TEG, Lat, Lng) == 0)
	{
		outp.MODE2_LAT = FormatDeclination(Lat * DEG);
		outp.MODE2_LON = FormatLongitude(Lng * DEG);
	}
}

void Supersighter::MODE3()
{
	// Fixed Line-of-Sight Rotation Mode

	MATRIX3 B0_M50_BY, M, B_M50_BY;
	VECTOR3 e, u_TGT_TEG, u_TGT_M50, u_TGT_BY;
	int TGT_TYP, TGT_NUM, INST_NUM;

	// Calculate initial attitude
	if (CalculateBodyMatrixFromAttitude(sv_ST, B0_M50_BY))
	{
		// TBD: Error
		return;
	}

	// Calculate eigen axis
	e = CalculateEigenAxis(inp.EIG.x * RAD, inp.EIG.y * RAD);

	// Calculate rotation matrix
	M = RotationAroundAxis(e, inp.EIG.z * RAD);
	B_M50_BY = mul(M, B0_M50_BY);

	// Output formatting
	outp.MODE = std::to_string(inp.Mode);
	outp.INMAT = inp.INMAT;
	outp.OUTMAT = inp.OUTMAT;
	outp.ATT_SOURCE = "MED";
	outp.ATT_SENSE = FormatAttSense();
	outp.EIGEN_VECTOR_P = FormatAttitude(inp.EIG.x);
	outp.EIGEN_VECTOR_Y = FormatAttitude(inp.EIG.y);
	outp.EIGEN_ANG = FormatAttitude(inp.EIG.z);
	outp.INPUT_ATT[0] = FormatAttitude(inp.ATT.x);
	outp.INPUT_ATT[1] = FormatAttitude(inp.ATT.y);
	outp.INPUT_ATT[2] = FormatAttitude(inp.ATT.z);
	CommonCalculations(sv_ST, B_M50_BY);

	if (CalculateAttitudeFromBodyMatrix(sv_ST, B_M50_BY, true))
	{
		// TBD
		return;
	}

	// Optionally, calculate instrument pointing angles for IA1, TGT 1
	if (inp.IA1 != "" && inp.TGT1 != "")
	{
		double A1, A2;
		bool Limit1, Limit2;

		// Get pointing direction to TGT 1
		if (DecodeTarget(inp.TGT1, TGT_TYP, TGT_NUM))
		{
			outp.ErrorMessage = "Error: TGT1 ID invalid";
			return;
		}
		if (DecodeInstrument(inp.IA1, INST_NUM))
		{
			outp.ErrorMessage = "Error: IA1 ID invalid";
			return;
		}
		// Calculate target direction
		u_TGT_TEG = GetTargetDirection(sv_ST, TGT_TYP, TGT_NUM);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		// Convert to body
		u_TGT_BY = mul(B_M50_BY, u_TGT_BY);
		// Calculate pointing angles
		GetInstrumentAnglesFromVector(u_TGT_BY, INST_NUM, A1, A2, Limit1, Limit2);

		outp.IA1 = inp.IA1;
		outp.IA1_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IA1_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IA1_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IA1_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IA1_OCC = OccultationCalculations(sv_ST.R, sv_ST.GMT, u_TGT_TEG);

		outp.TGT1 = inp.TGT1;
		if (TGT_TYP <= 3)
		{
			// Celestial
			FormatCelestialTarget(u_TGT_M50, outp.TGT1_RA, outp.TGT1_DEC);
		}
		else if (TGT_TYP == 4)
		{
			// Ground
			FormatGroundTarget(sv_ST, TGT_NUM);
		}
		else if (TGT_TYP == 5)
		{
			// Vehicle
			FormatVehicleTarget(sv_ST);
		}
	}
	// Optionally, calculate instrument pointing angles for IA2, TGT 2
	if (inp.IA2 != "" && inp.TGT2 != "")
	{
		VECTOR3 u_TGT_M50, u_TGT_BY;
		double A1, A2;
		int TGT_TYP, TGT_NUM, INST_NUM;
		bool Limit1, Limit2;

		// Get pointing direction to TGT 1
		if (DecodeTarget(inp.TGT2, TGT_TYP, TGT_NUM, false))
		{
			outp.ErrorMessage = "Error: TGT2 ID invalid";
			return;
		}
		if (DecodeInstrument(inp.IA2, INST_NUM))
		{
			outp.ErrorMessage = "Error: IA2 ID invalid";
			return;
		}
		// Calculate target direction
		u_TGT_TEG = GetTargetDirection(sv_ST, TGT_TYP, TGT_NUM);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		// Convert to body
		u_TGT_BY = mul(B_M50_BY, u_TGT_BY);
		// Calculate pointing angles
		GetInstrumentAnglesFromVector(u_TGT_BY, INST_NUM, A1, A2, Limit1, Limit2);

		outp.IA2 = inp.IA2;
		outp.IA2_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IA2_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IA2_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IA2_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IA2_OCC = OccultationCalculations(sv_ST.R, sv_ST.GMT, u_TGT_TEG);

		outp.TGT2 = inp.TGT2;
		FormatCelestialTarget(u_TGT_M50, outp.TGT2_RA, outp.TGT2_DEC);
	}
}

void Supersighter::MODE4()
{
	// Minimum maneuver mode

	MATRIX3 B0_M50_BY, B_M50_BY;
	VECTOR3 P_BY, u_TGT_TEG, u_TGT_M50;
	double EIG_P, EIG_Y, EIG_ANG;
	int inst, TGT_TYP, TGT_NUM;
	OrbMech::SV sv_AOS;
	double GMT_TCA, GMT_LOS;

	// Calculate initial attitude
	if (CalculateBodyMatrixFromAttitude(sv_ST, B0_M50_BY))
	{
		// TBD: Error
		return;
	}

	if (DecodeInstrument(inp.IA1, inst))
	{
		outp.ErrorMessage = "Error: IA1 ID invalid";
		return;
	}

	if (DecodeTarget(inp.TGT1, TGT_TYP, TGT_NUM))
	{
		outp.ErrorMessage = "Error: TGT1 ID invalid";
		return;
	}

	if (FindAOS(sv_ST, TGT_TYP, TGT_NUM, sv_AOS, GMT_TCA, GMT_LOS))
	{
		outp.ErrorMessage = "Error: AOS not found";
		return;
	}

	// Calculate instrument direction
	P_BY = GetVectorFromInstrumentAngles(inst, inp.IA1_A1 * RAD, inp.IA1_A2 * RAD);

	// Calculate inertial direction
	u_TGT_TEG = GetTargetDirection(sv_ST, TGT_TYP, TGT_NUM);
	u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);

	// Calculate final attitude
	B_M50_BY = Mode4Attitude(B0_M50_BY, P_BY, u_TGT_M50, EIG_P, EIG_Y, EIG_ANG);

	// Output formatting
	outp.MODE = std::to_string(inp.Mode);
	outp.INMAT = inp.INMAT;
	outp.OUTMAT = inp.OUTMAT;
	outp.ATT_SOURCE = "MED";
	outp.ATT_SENSE = FormatAttSense();
	outp.INPUT_ATT[0] = FormatAttitude(inp.ATT.x);
	outp.INPUT_ATT[1] = FormatAttitude(inp.ATT.y);
	outp.INPUT_ATT[2] = FormatAttitude(inp.ATT.z);
	outp.EIGEN_VECTOR_P = FormatAttitude(EIG_P * DEG);
	outp.EIGEN_VECTOR_Y = FormatAttitude(EIG_Y * DEG);
	outp.EIGEN_ANG = FormatAttitude(EIG_ANG * DEG);
	outp.ELV = FormatInstrumentAngle(inp.ELV);

	// Format AOS/TCA/LOS
	outp.AOS_GMT = GMT2String(sv_AOS.GMT);
	outp.AOS_MET = MET2String(METfromGMT(sv_AOS.GMT));
	outp.TCA_GMT = GMT2String(GMT_TCA);
	outp.TCA_MET = MET2String(METfromGMT(GMT_TCA));
	outp.LOS_GMT = GMT2String(GMT_LOS);
	outp.LOS_MET = MET2String(METfromGMT(GMT_LOS));

	CommonCalculations(sv_ST, B_M50_BY);

	if (CalculateAttitudeFromBodyMatrix(sv_ST, B_M50_BY, true))
	{
		// TBD
		return;
	}

	outp.TGT1 = inp.TGT1;
	if (TGT_TYP <= 3)
	{
		// Celestial
		FormatCelestialTarget(u_TGT_M50, outp.TGT1_RA, outp.TGT1_DEC);
	}
	else if (TGT_TYP == 4)
	{
		// Ground
		FormatGroundTarget(sv_ST, TGT_NUM);
	}
	else if (TGT_TYP == 5)
	{
		// Vehicle
		FormatVehicleTarget(sv_ST);
	}

	// Optional instrument 2, target 2
	if (inp.IA2 != "" && inp.TGT2 != "")
	{
		VECTOR3 u_BY;
		double A1, A2;
		bool Limit1, Limit2;

		if (DecodeInstrument(inp.IA2, inst))
		{
			outp.ErrorMessage = "Error: IA2 ID invalid";
			return;
		}

		if (DecodeTarget(inp.TGT2, TGT_TYP, TGT_NUM, false))
		{
			outp.ErrorMessage = "Error: TGT2 ID invalid";
			return;
		}
		u_TGT_TEG = GetTargetDirection(sv_ST, TGT_TYP, TGT_NUM);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		u_BY = mul(B_M50_BY, u_TGT_M50);
		GetInstrumentAnglesFromVector(u_BY, inst, A1, A2, Limit1, Limit2);

		outp.IA2 = inp.IA2;
		outp.IA2_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IA2_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IA2_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IA2_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IA2_OCC = OccultationCalculations(sv_ST.R, sv_ST.GMT, u_TGT_TEG);

		FormatCelestialTarget(u_TGT_M50, outp.TGT2_RA, outp.TGT2_DEC);
	}
}

void Supersighter::MODE5()
{
	// Fixed line-of-sight/MGA Mode

	MATRIX3 B_M50_BY_A, B_M50_BY_B;
	VECTOR3 u_TGT_TEG, u_TGT_M50, u_BY, AttA, AttB;
	int inst, TGT_TYP, TGT_NUM;
	OrbMech::SV sv_AOS;
	double GMT_TCA, GMT_LOS;

	if (DecodeInstrument(inp.IA1, inst))
	{
		outp.ErrorMessage = "Error: IA1 ID invalid";
		return;
	}

	if (DecodeTarget(inp.TGT1, TGT_TYP, TGT_NUM))
	{
		outp.ErrorMessage = "Error: TGT1 ID invalid";
		return;
	}

	if (FindAOS(sv_ST, TGT_TYP, TGT_NUM, sv_AOS, GMT_TCA, GMT_LOS))
	{
		outp.ErrorMessage = "Error: AOS not found";
		return;
	}

	u_TGT_TEG = GetTargetDirection(sv_AOS, TGT_TYP, TGT_NUM);
	u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);

	u_BY = GetVectorFromInstrumentAngles(inst, inp.IA1_A1 * RAD, inp.IA1_A2 * RAD);

	// Calculate two attitude
	if (Mode5Attitude(u_BY, u_TGT_M50, inp.MGA * RAD, AttA, AttB))
	{
		outp.ErrorMessage = "Error: No att possible with input MGA";
		return;
	}

	// Calculate body matrices
	B_M50_BY_A = OrbMech::tmat(PYRAnglesToMatrix(AttA.x, AttA.y, AttA.z));
	B_M50_BY_B = OrbMech::tmat(PYRAnglesToMatrix(AttB.x, AttB.y, AttB.z));

	// Format outputs
	outp.MODE = std::to_string(inp.Mode);
	outp.INMAT = inp.INMAT;
	outp.OUTMAT = inp.OUTMAT;
	outp.MGA = FormatAttitude(inp.MGA);
	outp.ELV = FormatInstrumentAngle(inp.ELV);
	outp.ATT_SENSE = FormatAttSense();
	outp.IA1 = inp.IA1;
	outp.IA1_A1 = FormatInstrumentAngle(inp.IA1_A1);
	outp.IA1_A2 = FormatInstrumentAngle(inp.IA1_A2);
	outp.IA1_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);

	if (CalculateAttitudeFromBodyMatrix(sv_AOS, B_M50_BY_A, true))
	{
		 // Error
		return;
	}
	if (CalculateAttitudeFromBodyMatrix(sv_AOS, B_M50_BY_B, false))
	{
		// Error
		return;
	}

	outp.TGT1 = inp.TGT1;
	if (TGT_TYP <= 3)
	{
		// Celestial
		FormatCelestialTarget(u_TGT_M50, outp.TGT1_RA, outp.TGT1_DEC);
	}
	else if (TGT_TYP == 4)
	{
		// Ground
		FormatGroundTarget(sv_AOS, TGT_NUM);
	}
	else if (TGT_TYP == 5)
	{
		// Vehicle
		FormatVehicleTarget(sv_AOS);
	}

	// Format AOS/TCA/LOS
	outp.AOS_GMT = GMT2String(sv_AOS.GMT);
	outp.AOS_MET = MET2String(METfromGMT(sv_AOS.GMT));
	outp.TCA_GMT = GMT2String(GMT_TCA);
	outp.TCA_MET = MET2String(METfromGMT(GMT_TCA));
	outp.LOS_GMT = GMT2String(GMT_LOS);
	outp.LOS_MET = MET2String(METfromGMT(GMT_LOS));

	CommonCalculations(sv_AOS, B_M50_BY_A);

	// Optional outputs

	double A1, A2;
	bool Limit1, Limit2;

	if (inp.IA2 != "" && inp.TGT2 != "")
	{
		if (DecodeInstrument(inp.IA2, inst))
		{
			outp.ErrorMessage = "Error: IA2 ID invalid";
			return;
		}

		if (DecodeTarget(inp.TGT2, TGT_TYP, TGT_NUM, false))
		{
			outp.ErrorMessage = "Error: TGT2 ID invalid";
			return;
		}
		u_TGT_TEG = GetTargetDirection(sv_AOS, TGT_TYP, TGT_NUM);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		u_BY = mul(B_M50_BY_A, u_TGT_M50);
		GetInstrumentAnglesFromVector(u_BY, inst, A1, A2, Limit1, Limit2);

		outp.IA2 = inp.IA2;
		outp.IA2_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IA2_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IA2_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IA2_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IA2_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);

		outp.TGT2 = inp.TGT2;
		FormatCelestialTarget(u_TGT_M50, outp.TGT2_RA, outp.TGT2_DEC);
	}
	if (inp.IB1 != "" && inp.TGT1 != "")
	{
		if (DecodeInstrument(inp.IB1, inst))
		{
			outp.ErrorMessage = "Error: IB1 ID invalid";
			return;
		}

		if (DecodeTarget(inp.TGT1, TGT_TYP, TGT_NUM))
		{
			outp.ErrorMessage = "Error: TGT1 ID invalid";
			return;
		}
		u_TGT_TEG = GetTargetDirection(sv_AOS, TGT_TYP, TGT_NUM);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		u_BY = mul(B_M50_BY_B, u_TGT_M50);
		GetInstrumentAnglesFromVector(u_BY, inst, A1, A2, Limit1, Limit2);

		outp.IB1 = inp.IB1;
		outp.IB1_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IB1_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IB1_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IB1_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IB1_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);
	}
	if (inp.IB2 != "" && inp.TGT2 != "")
	{
		if (DecodeInstrument(inp.IB2, inst))
		{
			outp.ErrorMessage = "Error: IB2 ID invalid";
			return;
		}

		if (DecodeTarget(inp.TGT2, TGT_TYP, TGT_NUM, false))
		{
			outp.ErrorMessage = "Error: TGT2 ID invalid";
			return;
		}
		u_TGT_TEG = GetTargetDirection(sv_AOS, TGT_TYP, TGT_NUM);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		u_BY = mul(B_M50_BY_B, u_TGT_M50);
		GetInstrumentAnglesFromVector(u_BY, inst, A1, A2, Limit1, Limit2);

		outp.IB2 = inp.IB2;
		outp.IB2_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IB2_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IB2_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IB2_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IB2_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);

		outp.TGT2 = inp.TGT2;
		FormatCelestialTarget(u_TGT_M50, outp.TGT2_RA, outp.TGT2_DEC);
	}
}

void Supersighter::MODE6()
{
	// Optimum Second Line-of-Sight Mode

	MATRIX3 B_M50_BY;
	VECTOR3 P1_BY, P2_BY, T1_TEG, T1_M50, T2_TEG, T2_M50, u_BY;
	double A1, A2;
	int INST1, TGT_TYP1, TGT_NUM1, INST2, TGT_TYP2, TGT_NUM2;
	bool Limit1, Limit2;
	OrbMech::SV sv_AOS;
	double GMT_TCA, GMT_LOS;

	if (DecodeInstrument(inp.IA1, INST1))
	{
		outp.ErrorMessage = "Error: IA1 ID invalid";
		return;
	}

	if (DecodeTarget(inp.TGT1, TGT_TYP1, TGT_NUM1))
	{
		outp.ErrorMessage = "Error: TGT1 ID invalid";
		return;
	}

	if (DecodeInstrument(inp.IA2, INST2))
	{
		outp.ErrorMessage = "Error: IA2 ID invalid";
		return;
	}

	if (DecodeTarget(inp.TGT2, TGT_TYP2, TGT_NUM2, false))
	{
		outp.ErrorMessage = "Error: TGT2 ID invalid";
		return;
	}

	if (FindAOS(sv_ST, TGT_TYP1, TGT_NUM1, sv_AOS, GMT_TCA, GMT_LOS))
	{
		outp.ErrorMessage = "Error: AOS not found";
		return;
	}

	// Instrument 1 direction
	P1_BY = GetVectorFromInstrumentAngles(INST1, inp.IA1_A1 * RAD, inp.IA1_A2 * RAD);

	// Instrument 2 direction
	P2_BY = GetVectorFromInstrumentAngles(INST2, inp.IA2_A1 * RAD, inp.IA2_A2 * RAD);

	// Target 1 direction
	T1_TEG = GetTargetDirection(sv_AOS, TGT_TYP1, TGT_NUM1);
	T1_M50 = mul(inp.sescnst->M_TEG_TO_M50, T1_TEG);

	// Target 2 direction
	T2_TEG = GetTargetDirection(sv_AOS, TGT_TYP2, TGT_NUM2);
	T2_M50 = mul(inp.sescnst->M_TEG_TO_M50, T2_TEG);

	if (Mode6OptimumSecondLineOfSight(P1_BY, T1_M50, P2_BY, T2_M50, B_M50_BY))
	{
		outp.ErrorMessage = "Error: No unique solution exists";
		return;
	}

	// IA2 actual instrument angles
	u_BY = mul(B_M50_BY, T2_M50);
	GetInstrumentAnglesFromVector(u_BY, INST2, A1, A2, Limit1, Limit2);

	// Format outputs
	outp.MODE = std::to_string(inp.Mode);
	outp.INMAT = inp.INMAT;
	outp.OUTMAT = inp.OUTMAT;
	outp.ELV = FormatInstrumentAngle(inp.ELV);
	outp.IA1 = inp.IA1;
	outp.IA1_A1 = FormatInstrumentAngle(inp.IA1_A1);
	outp.IA1_A2 = FormatInstrumentAngle(inp.IA1_A2);
	outp.IA1_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, T1_TEG);

	if (CalculateAttitudeFromBodyMatrix(sv_AOS, B_M50_BY, true))
	{
		// TBD: Error
		return;
	}

	outp.TGT1 = inp.TGT1;
	if (TGT_TYP1 <= 3)
	{
		// Celestial
		FormatCelestialTarget(T1_M50, outp.TGT1_RA, outp.TGT1_DEC);
	}
	else if (TGT_TYP1 == 4)
	{
		// Ground
		FormatGroundTarget(sv_AOS, TGT_NUM1);
	}
	else if (TGT_TYP1 == 5)
	{
		// Vehicle
		FormatVehicleTarget(sv_AOS);
	}

	// Format AOS/TCA/LOS
	outp.AOS_GMT = GMT2String(sv_AOS.GMT);
	outp.AOS_MET = MET2String(METfromGMT(sv_AOS.GMT));
	outp.TCA_GMT = GMT2String(GMT_TCA);
	outp.TCA_MET = MET2String(METfromGMT(GMT_TCA));
	outp.LOS_GMT = GMT2String(GMT_LOS);
	outp.LOS_MET = MET2String(METfromGMT(GMT_LOS));

	outp.IA2 = inp.IA2;
	outp.IA2_A1 = FormatInstrumentAngle(A1 * DEG);
	outp.IA2_A1_LIM = FormatInstrumentLimit(Limit1);
	outp.IA2_A2 = FormatInstrumentAngle(A2 * DEG);
	outp.IA2_A2_LIM = FormatInstrumentLimit(Limit2);
	outp.IA2_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, T2_TEG);
	outp.TGT2 = inp.TGT2;

	outp.TGT2 = inp.TGT2;
	FormatCelestialTarget(T2_M50, outp.TGT2_RA, outp.TGT2_DEC);

	CommonCalculations(sv_AOS, B_M50_BY);
}

void Supersighter::MODE7()
{
	// Fixed Line-of-Sight Omicron Mode
	// Mode 7 computes an Orbiter attitude (output A) given a specific Omicron angle and instrument IA1 ID and instrument angles to point at target 1 for
	// AOS of target 1. Optionally, instrument pointing angles may be requested for the IA2 instrument ID. A target 2 will also be input corresponding to IA2.

	MATRIX3 B_M50_BY;
	VECTOR3 u_BY, u_TGT_TEG, u_TGT_M50, R_M50, V_M50;
	int INST, TGT_TYP, TGT_NUM;
	OrbMech::SV sv_AOS;
	double GMT_TCA, GMT_LOS;

	if (DecodeInstrument(inp.IA1, INST))
	{
		outp.ErrorMessage = "Error: IA1 ID invalid";
		return;
	}

	if (DecodeTarget(inp.TGT1, TGT_TYP, TGT_NUM))
	{
		outp.ErrorMessage = "Error: TGT1 ID invalid";
		return;
	}

	if (FindAOS(sv_ST, TGT_TYP, TGT_NUM, sv_AOS, GMT_TCA, GMT_LOS))
	{
		outp.ErrorMessage = "Error: AOS not found";
		return;
	}

	// Calculate body pointing vector
	u_BY = GetVectorFromInstrumentAngles(INST, inp.IA1_A1 * RAD, inp.IA1_A2 * RAD);

	// Calculate target pointing vector
	u_TGT_TEG = GetTargetDirection(sv_AOS, TGT_TYP, TGT_NUM);
	u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);

	// Calculate attitude
	R_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv_AOS.R);
	V_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv_AOS.V);
	B_M50_BY = Mode7Attitude(R_M50, V_M50, u_BY, u_TGT_M50, inp.OMI * RAD);

	// Format outputs
	outp.MODE = std::to_string(inp.Mode);
	outp.INMAT = inp.INMAT;
	outp.OUTMAT = inp.OUTMAT;
	outp.ELV = FormatInstrumentAngle(inp.ELV);
	outp.IA1 = inp.IA1;
	outp.IA1_A1 = FormatInstrumentAngle(inp.IA1_A1);
	outp.IA1_A2 = FormatInstrumentAngle(inp.IA1_A2);
	outp.IA1_A3 = FormatInstrumentAngle(inp.OMI);
	outp.IA1_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);

	if (CalculateAttitudeFromBodyMatrix(sv_AOS, B_M50_BY, true))
	{
		// TBD
		return;
	}

	outp.TGT1 = inp.TGT1;
	if (TGT_TYP <= 3)
	{
		// Celestial
		FormatCelestialTarget(u_TGT_M50, outp.TGT1_RA, outp.TGT1_DEC);
	}
	else if (TGT_TYP == 4)
	{
		// Ground
		FormatGroundTarget(sv_AOS, TGT_NUM);
	}
	else if (TGT_TYP == 5)
	{
		// Vehicle
		FormatVehicleTarget(sv_AOS);
	}

	// Format AOS/TCA/LOS
	outp.AOS_GMT = GMT2String(sv_AOS.GMT);
	outp.AOS_MET = MET2String(METfromGMT(sv_AOS.GMT));
	outp.TCA_GMT = GMT2String(GMT_TCA);
	outp.TCA_MET = MET2String(METfromGMT(GMT_TCA));
	outp.LOS_GMT = GMT2String(GMT_LOS);
	outp.LOS_MET = MET2String(METfromGMT(GMT_LOS));

	CommonCalculations(sv_AOS, B_M50_BY);

	// Optional IA2 calculations
	if (inp.IA2 != "" && inp.TGT2 != "")
	{
		if (DecodeInstrument(inp.IA2, INST))
		{
			outp.ErrorMessage = "Error: IA2 ID invalid";
			return;
		}

		if (DecodeTarget(inp.TGT2, TGT_TYP, TGT_NUM, false))
		{
			outp.ErrorMessage = "Error: TGT2 ID invalid";
			return;
		}

		double A1, A2;
		bool Limit1, Limit2;

		// Calculate target pointing vector
		u_TGT_TEG = GetTargetDirection(sv_AOS, TGT_TYP, TGT_NUM);
		u_TGT_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_TGT_TEG);
		u_BY = mul(B_M50_BY, u_TGT_M50);

		// Calculate instrument angles
		GetInstrumentAnglesFromVector(u_BY, INST, A1, A2, Limit1, Limit2);

		outp.IA2 = inp.IA2;
		outp.IA2_A1 = FormatInstrumentAngle(A1 * DEG);
		outp.IA2_A1_LIM = FormatInstrumentLimit(Limit1);
		outp.IA2_A2 = FormatInstrumentAngle(A2 * DEG);
		outp.IA2_A2_LIM = FormatInstrumentLimit(Limit2);
		outp.IA2_OCC = OccultationCalculations(sv_AOS.R, sv_AOS.GMT, u_TGT_TEG);

		outp.TGT2 = inp.TGT2;
		FormatCelestialTarget(u_TGT_M50, outp.TGT2_RA, outp.TGT2_DEC);
	}
}

void Supersighter::GetInstrumentAnglesFromVector(VECTOR3 u_BY, int n, double& A1, double& A2, bool& Limit1, bool& Limit2) const
{
	inp.IDT[n].BodyVectorToInstrumentAngles(inp.IMT, u_BY, A1, A2, Limit1, Limit2);
}

VECTOR3 Supersighter::GetVectorFromInstrumentAngles(int n, double A1, double A2) const
{
	return inp.IDT[n].InstrumentAnglesToBodyVector(inp.IMT, A1, A2);
}

void Supersighter::CommonCalculations(const OrbMech::SV& sv, const MATRIX3& B_M50_BY)
{
	// Phi/Theta, Pitch/Yaw to the Earth, Moon, and Sun
	// RA/DEC of the body axes +X, -Z

	MATRIX3 B_TEG_BY;
	VECTOR3 u_Earth_TEG, u_Moon_TEG, u_Sun_TEG, u_BY, u_M50;
	double Phi, Theta, Pitch, Yaw, DEC, RA;

	B_TEG_BY = mul(B_M50_BY, inp.sescnst->M_TEG_TO_M50);

	u_Earth_TEG = GetTargetDirection(sv, 0, 0);
	u_Moon_TEG = GetTargetDirection(sv, 1, 0);
	u_Sun_TEG = GetTargetDirection(sv, 2, 0);

	// Earth
	u_BY = mul(B_TEG_BY, u_Earth_TEG);
	VectorToPhiTheta(u_BY, Phi, Theta);
	VectorToPY(u_BY, Pitch, Yaw);
	outp.Pitch_E = FormatString("%.0lf", Pitch * DEG);
	outp.Yaw_E = FormatString("%.0lf", Yaw * DEG);
	outp.Theta_E = FormatString("%.0lf", Theta * DEG);
	outp.Phi_E = FormatString("%.0lf", Phi * DEG);

	// Moon
	u_BY = mul(B_TEG_BY, u_Moon_TEG);
	VectorToPhiTheta(u_BY, Phi, Theta);
	VectorToPY(u_BY, Pitch, Yaw);
	outp.Pitch_M = FormatString("%.0lf", Pitch * DEG);
	outp.Yaw_M = FormatString("%.0lf", Yaw * DEG);
	outp.Theta_M = FormatString("%.0lf", Theta * DEG);
	outp.Phi_M = FormatString("%.0lf", Phi * DEG);

	// Sun
	u_BY = mul(B_TEG_BY, u_Sun_TEG);
	VectorToPhiTheta(u_BY, Phi, Theta);
	VectorToPY(u_BY, Pitch, Yaw);
	outp.Pitch_S = FormatString("%.0lf", Pitch * DEG);
	outp.Yaw_S = FormatString("%.0lf", Yaw * DEG);
	outp.Theta_S = FormatString("%.0lf", Theta * DEG);
	outp.Phi_S = FormatString("%.0lf", Phi * DEG);

	// +X
	u_M50 = tmul(B_M50_BY, _V(1, 0, 0));
	OrbMech::latlong_from_r(u_M50, DEC, RA);
	if (RA < 0.0) RA += PI2;
	outp.RA_PX = FormatAttitude(RA * DEG);
	outp.DEC_PX = FormatDeclination(DEC * DEG);

	// -Z
	u_M50 = tmul(B_M50_BY, _V(0, 0, -1));
	OrbMech::latlong_from_r(u_M50, DEC, RA);
	if (RA < 0.0) RA += PI2;
	outp.RA_MZ = FormatAttitude(RA * DEG);
	outp.DEC_MZ = FormatDeclination(DEC * DEG);

	// Format start time in MET and GMT
	outp.ST_MET = MET2String(inp.StartTime);
	outp.ST_GMT = GMT2String(GMTfromMET(inp.StartTime));
}

void Supersighter::VectorToPY(VECTOR3 u_BY, double& P, double& Y) const
{
	P = atan2(-u_BY.z, u_BY.x);
	if (P < 0)
	{
		P = P + PI2;
	}
	Y = OrbMech::asin2(u_BY.y);
	if (Y < 0.0) Y += PI2;
}

void Supersighter::VectorToPhiTheta(VECTOR3 u_BY, double& Phi, double& Theta) const
{
	Phi = atan2(u_BY.y, -u_BY.z);
	if (Phi < 0)
	{
		Phi = Phi + PI;
	}
	Theta = OrbMech::acos2(u_BY.x);
}

int Supersighter::CalculateBodyMatrixFromAttitude(const OrbMech::SV& sv, MATRIX3& B_M50_BY) const
{
	std::string INMATType;
	VECTOR3 Att;

	Att = inp.ATT * RAD;
	INMATType = inp.INMAT.substr(0, 4);

	if (INMATType == "RLMT")
	{
		// BY = Body, SA = Sense axis, AX = +X sense
		MATRIX3 M_M50_AX, M_ATTSENSE, M_BY_SA, M_BY_AX;

		// ADI attitude
		// Take ATTSENSE into account

		if (GetRELMAT(inp.INMAT, M_M50_AX)) return 1;

		// Get body to ADI (att sense) matrix
		M_BY_SA = PYRAnglesToMatrix(Att.x, Att.y, Att.z);

		// Convert att sense
		M_ATTSENSE = ADIAttSenseConversion(inp.ATTSense, 0);
		M_BY_AX = mul(M_BY_SA, M_ATTSENSE);

		// Calculate M50 to body matrix
		B_M50_BY = mul(OrbMech::tmat(M_BY_AX), M_M50_AX);
	}
	else if (INMATType == "RFMT")
	{
		// IMU attitude
		MATRIX3 M_M50_IMU, M_BY_IMU;
		if (GetREFSMMAT(inp.INMAT, M_M50_IMU)) return 1;
		M_BY_IMU = PYRAnglesToMatrix(Att.x, Att.y, Att.z);

		B_M50_BY = mul(OrbMech::tmat(M_BY_IMU), M_M50_IMU);
	}
	else if (INMATType == "LPYR" || INMATType == "LYPR")
	{
		// LVLH attitude
		MATRIX3 M_BY_LVLH, M_M50_LVLH, M_ATTSENSE;
		VECTOR3 R_M50, V_M50;

		M_ATTSENSE = ADIAttSenseConversion(inp.ATTSense, 0);

		if (INMATType == "LPYR")
		{
			M_BY_LVLH = PYRAnglesToMatrix(Att.x, Att.y, Att.z);
		}
		else
		{
			M_BY_LVLH = YPRAnglesToMatrix(Att.x, Att.y, Att.z);
		}
		M_BY_LVLH = mul(M_BY_LVLH, M_ATTSENSE);

		R_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv.R);
		V_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv.V);
		M_M50_LVLH = LVLH_Matrix(R_M50, V_M50);

		B_M50_BY = mul(OrbMech::tmat(M_BY_LVLH), M_M50_LVLH);

		// TBD: Bias matrix
	}
	else return 1;

	return 0;
}

int Supersighter::CalculateAttitudeFromBodyMatrix(const OrbMech::SV& sv, MATRIX3 B_M50_BY, bool IsAttitudeA)
{
	// Input attitude is for M50, output can be ADI or LVLH
	// Pitch, Yaw, Roll sequence but output is RPY

	std::string Att[6][3];
	std::string INMATType, OUTMATType, AttRef;
	MATRIX3 M_M50_LVLH, M_LVLH_BY, M_AX_BY;
	VECTOR3 R_M50, V_M50, AttTemp;
	unsigned int i, j;
	bool PrintOtherAttSenses;

	// Determine output att sense
	INMATType = inp.INMAT.substr(0, 4);
	OUTMATType = inp.OUTMAT.substr(0, 4);
	if (INMATType == "RFMT")
	{
		PrintOtherAttSenses = false;
	}
	else
	{
		PrintOtherAttSenses = true;
	}
	if (OUTMATType == "RFMT")
	{
		AttRef = "IMU";
	}
	else if (OUTMATType == "RLMT")
	{
		AttRef = "ADI";
	}
	else return 1;

	R_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv.R);
	V_M50 = mul(inp.sescnst->M_TEG_TO_M50, sv.V);
	M_M50_LVLH = LVLH_Matrix(R_M50, V_M50);
	M_LVLH_BY = mul(B_M50_BY, OrbMech::tmat(M_M50_LVLH));

	if (OUTMATType == "RFMT")
	{
		// RFMT
		MATRIX3 M_M50_IMU, M_IMU_BY;

		if (GetREFSMMAT(inp.OUTMAT, M_M50_IMU)) return 1;

		M_IMU_BY = mul(B_M50_BY, OrbMech::tmat(M_M50_IMU));
		M_AX_BY = M_IMU_BY;
	}
	else
	{
		// RLMT
		MATRIX3 M_M50_ADI, M_ADI_BY;

		if (GetRELMAT(inp.OUTMAT, M_M50_ADI)) return 1;

		M_ADI_BY = mul(B_M50_BY, OrbMech::tmat(M_M50_ADI));
		M_AX_BY = M_ADI_BY;
	}

	AttTemp = MatrixToPYRAngles(M_AX_BY);
	Att[0][0] = FormatAttitude(AttTemp.x * DEG);
	Att[0][1] = FormatAttitude(AttTemp.y * DEG);
	Att[0][2] = FormatAttitude(AttTemp.z * DEG);

	AttTemp = MatrixToPYRAngles(M_LVLH_BY);
	Att[3][0] = FormatAttitude(AttTemp.x * DEG);
	Att[3][1] = FormatAttitude(AttTemp.y * DEG);
	Att[3][2] = FormatAttitude(AttTemp.z * DEG);

	if (PrintOtherAttSenses)
	{
		MATRIX3 M_TEMP;

		// Convert to IMU/ADI -X
		M_TEMP = mul(ADIAttSenseConversion(1, 0), M_AX_BY);
		AttTemp = MatrixToPYRAngles(M_TEMP);
		Att[1][0] = FormatAttitude(AttTemp.x * DEG);
		Att[1][1] = FormatAttitude(AttTemp.y * DEG);
		Att[1][2] = FormatAttitude(AttTemp.z * DEG);
		
		// Convert to IMU/ADI -Z
		M_TEMP = mul(ADIAttSenseConversion(2, 0), M_AX_BY);
		AttTemp = MatrixToPYRAngles(M_TEMP);
		Att[2][0] = FormatAttitude(AttTemp.x * DEG);
		Att[2][1] = FormatAttitude(AttTemp.y * DEG);
		Att[2][2] = FormatAttitude(AttTemp.z * DEG);

		// Convert to LVLH -X
		M_TEMP = mul(ADIAttSenseConversion(1, 0), M_LVLH_BY);
		AttTemp = MatrixToPYRAngles(M_TEMP);
		Att[4][0] = FormatAttitude(AttTemp.x * DEG);
		Att[4][1] = FormatAttitude(AttTemp.y * DEG);
		Att[4][2] = FormatAttitude(AttTemp.z * DEG);

		// Convert to LVLH -Z
		M_TEMP = mul(ADIAttSenseConversion(2, 0), M_LVLH_BY);
		AttTemp = MatrixToPYRAngles(M_TEMP);
		Att[5][0] = FormatAttitude(AttTemp.x * DEG);
		Att[5][1] = FormatAttitude(AttTemp.y * DEG);
		Att[5][2] = FormatAttitude(AttTemp.z * DEG);
	}

	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 3; j++)
		{
			if (IsAttitudeA)
			{
				outp.OUTPUT_A_ATT[i][j] = Att[i][j];
			}
			else
			{
				outp.OUTPUT_B_ATT[i][j] = Att[i][j];
			}
		}
	}

	// Output attitude reference
	if (IsAttitudeA)
	{
		outp.OUTPUT_A_ATT_REF = AttRef;
	}
	else
	{
		outp.OUTPUT_B_ATT_REF = AttRef;
	}

	return 0;
}

int Supersighter::Mode2EarthIntersection(VECTOR3 R_C, double GMT, VECTOR3 u_M, double& Lat, double& Lng) const
{
	// Calculate the latitude and longitude if the instrument LOS intersects the Earth.

	// INPUTS: 
	// R_C: Shuttle position vector in TEG coordinates
	// GMT: Time of position vector
	// u_M: Unit pointing direction in TEG coordinates

	MATRIX3 M_TEG_EF;
	VECTOR3 R_L, R_L_EF;
	double r0, r_C, C, rho;

	r0 = OrbMech::EARTH_RADIUS_ORBITER;

	r_C = length(R_C);
	C = -dotp(u_M, R_C) / r_C;

	if (C < 0.0)
	{
		// No intersection
		return 1;
	}

	rho = (r0 * r0) / (r_C * r_C) - 1.0 + C * C;

	if (rho < 0.0)
	{
		// No intersection
		return 1;
	}
	rho = r_C * (C - sqrt(rho));

	// Calculate intersection point
	R_L = R_C + u_M * rho;
	// Convert to Earth-fixed
	M_TEG_EF = OrbMech::TEG_to_EF_Matrix(OrbMech::w_Earth, GMT);
	R_L_EF = mul(M_TEG_EF, R_L);
	// Calculate latitude and longitude
	OrbMech::latlong_from_r(R_L_EF, Lat, Lng);
	return 0;
}

MATRIX3 Supersighter::Mode4Attitude(const MATRIX3& B0_M50_BY, VECTOR3 P, VECTOR3 T, double& EIG_P, double& EIG_Y, double& EIG_ANG) const
{
	// INPUTS:
	// B_M50_BY: Initial body matrix (M50 to body)
	// P: Unit pointing vector (instrument line of sight) in body frame
	// T: Unit target vector in M50 (unit vector along orbiter-target line of sight)
	// OUTPUTS:
	// B_M50_BY: Final attitude matrix

	MATRIX3 B_M50_BY;
	VECTOR3 T_BY, a, e;
	double theta;

	T_BY = mul(B0_M50_BY, T);

	if (abs(dotp(P, T_BY)) >= (1.0 - eps))
	{
		a.x = 0.0;
		if (abs(P.y) < abs(P.z))
		{
			a.y = 1.0;
			a.z = 0.0;
		}
		else
		{
			a.y = 0.0;
			a.z = 0.0;
		}
		e = unit(crossp(a, T_BY));
	}
	else
	{
		e = unit(crossp(P, T_BY));
	}
	theta = acos(dotp(P, T_BY));

	B_M50_BY = mul(RotationAroundAxis(e, theta), B0_M50_BY);

	// Eigen angles
	EIG_ANG = theta;
	CalculateEigenAxisPY(e, EIG_P, EIG_Y);

	return B_M50_BY;
}

int Supersighter::Mode5Attitude(VECTOR3 u_BY, VECTOR3 u_SM, double MGA, VECTOR3& Att1, VECTOR3& Att2) const
{
	// Given a specific middle gimbal angle (MGA) and instrument and target directions calculate the two possible attitudes
	double a, b, f, OGA1, OGA2, IGA1, IGA2;

	a = u_BY.y;
	b = -u_BY.z;
	f = (u_SM.y - sin(MGA) * u_BY.x) / cos(MGA);

	if (HarmonicAdditionSolver(a, b, f, OGA1, OGA2)) return 1;

	IGA1 = CalculateIGA(MGA, OGA1, u_BY, u_SM);
	IGA2 = CalculateIGA(MGA, OGA2, u_BY, u_SM);

	Att1 = _V(OGA1, IGA1, MGA);
	Att2 = _V(OGA2, IGA2, MGA);
	return 0;
}

int Supersighter::Mode6OptimumSecondLineOfSight(VECTOR3 P1, VECTOR3 T1, VECTOR3 P2, VECTOR3 T2, MATRIX3& B_M50_BY) const
{
	if (abs(dotp(T1, T2)) >= (1.0 - eps))
	{
		// No unique solution exists
		return 1;
	}
	if (abs(dotp(P1, P2)) >= (1.0 - eps))
	{
		// No unique solution exists
		return 1;
	}

	VECTOR3 u1, u2, u3, u1_apo, u2_apo, u3_apo;

	u1 = T1;
	u2 = unit(crossp(T1, T2));
	u3 = crossp(u1, u2);
	u1_apo = P1;
	u2_apo = unit(crossp(P1, P2));
	u3_apo = crossp(u1_apo, u2_apo);

	B_M50_BY = mul(_M(u1_apo.x, u2_apo.x, u3_apo.x, u1_apo.y, u2_apo.y, u3_apo.y, u1_apo.z, u2_apo.z, u3_apo.z), _M(u1.x, u1.y, u1.z, u2.x, u2.y, u2.z, u3.x, u3.y, u3.z));
	return 0;
}

int Supersighter::ComputeOmicron(VECTOR3 R, VECTOR3 V, const MATRIX3& B_M50_BY, VECTOR3 T, double& OMI) const
{
	// INPUTS:
	// R: Orbiter position in M50
	// V: Orbiter velocity in M50
	// B_M50_BY: Body matrix which defines the attitude of the body frame with respect to the M50 frame
	// T: Unit target vector in M50 (Body pointing vector P is assumed to coincide with T)
	// OUTPUTS: Omicron

	VECTOR3 H, N1, b, N2;

	H = unit(crossp(R, V));
	N1 = crossp(H, T);

	if (dotp(N1, N1) < eps)
	{
		// No solution
		return 1;
	}
	b = tmul(B_M50_BY, _V(0, 1.0, 0));
	if (abs(dotp(T, b)) > UP_TOL)
	{
		b = tmul(B_M50_BY, _V(0, 0, -1.0));
	}
	N2 = crossp(T, b);
	OMI = atan2(dotp(N1, b), dotp(N1, N2));
	if (OMI < 0.0) OMI += PI2;

	return 0;
}

double sign(double a)
{
	if (a >= 0.0)
	{
		return 1.0;
	}
	return -1.0;
}

MATRIX3 Supersighter::Mode7Attitude(VECTOR3 R_M50, VECTOR3 V_M50, VECTOR3 P_BY, VECTOR3 T_M50, double OMICRON) const
{
	MATRIX3 B_M50_BY;
	VECTOR3 RR_BOD, RR_M50, RRA_BOD, RRA_M50, YN, YT;
	double ROLL, DOT;

	RR_BOD = _V(0, 1, 0);
	RR_M50 = -unit(crossp(R_M50, V_M50));
	RRA_BOD = _V(0, 0, -1);

	if (abs(dotp(P_BY, RR_BOD)) > UP_TOL)
	{
		RR_BOD = RRA_BOD;
	}
	RRA_M50 = crossp(_V(0, 0, 1), RR_M50);
	ROLL = OMICRON + PI05;

	DOT = dotp(T_M50, RR_M50);
	if (abs(DOT) > UP_TOL)
	{
		RR_M50 = RRA_M50 * sign(DOT);
	}
	YN = unit(crossp(P_BY, RR_BOD));
	YT = unit(crossp(T_M50, RR_M50)) * sin(ROLL) - crossp(T_M50, unit(crossp(T_M50, RR_M50))) * cos(ROLL);

	VECTOR3 u1, u2, u3, u1_apo, u2_apo, u3_apo;

	u1 = T_M50;
	u2 = unit(crossp(T_M50, YT));
	u3 = crossp(u1, u2);
	u1_apo = P_BY;
	u2_apo = unit(crossp(P_BY, YN));
	u3_apo = crossp(u1_apo, u2_apo);

	B_M50_BY = mul(_M(u1_apo.x, u2_apo.x, u3_apo.x, u1_apo.y, u2_apo.y, u3_apo.y, u1_apo.z, u2_apo.z, u3_apo.z), _M(u1.x, u1.y, u1.z, u2.x, u2.y, u2.z, u3.x, u3.y, u3.z));
	return B_M50_BY;
}

int Supersighter::GetRELMAT(std::string relmat, MATRIX3& mat) const
{
	// RELMAT: M50 to ADI coordinate system conversion
	// TBD: Only starball RELMAT
	//mat = _M(1, 0, 0, 0, 1, 0, 0, 0, 1);
	mat = _M(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
	return 0;
}

int Supersighter::GetREFSMMAT(std::string relmat, MATRIX3& mat) const
{
	// TBD
	mat = _M(1, 0, 0, 0, 1, 0, 0, 0, 1);
	return 0;
}

int Supersighter::GetLVLHBiasMatrix(std::string relmat, MATRIX3& mat) const
{
	// TBD
	mat = _M(1, 0, 0, 0, 1, 0, 0, 0, 1);
	return 0;
}

MATRIX3 Supersighter::PYRAnglesToMatrix(double R, double P, double Y) const
{
	// Sequence: Pitch, Yaw, Roll
	// Usually body to IMU

	MATRIX3 mat;
	double o, i, m;

	o = R;
	i = P;
	m = Y;

	// o = phi, i = theta, m = psi

	mat.m11 = cos(i) * cos(m);
	mat.m12 = -cos(i) * sin(m) * cos(o) + sin(i) * sin(o);
	mat.m13 = cos(i) * sin(m) * sin(o) + sin(i) * cos(o);
	mat.m21 = sin(m);
	mat.m22 = cos(m) * cos(o);
	mat.m23 = -cos(m) * sin(o);
	mat.m31 = -sin(i) * cos(m);
	mat.m32 = sin(i) * sin(m) * cos(o) + cos(i) * sin(o);
	mat.m33 = -sin(i) * sin(m) * sin(o) + cos(i) * cos(o);

	return mat;
}

MATRIX3 Supersighter::YPRAnglesToMatrix(double R, double P, double Y) const
{
	// Sequence: Yaw, Pitch, Roll
	// Usually body to IMU

	MATRIX3 mat;
	double o, i, m;

	o = R;
	i = P;
	m = Y;

	// o = phi, i = theta, m = psi

	mat.m11 = cos(m) * cos(i);
	mat.m12 = cos(m) * sin(i) * sin(o) - sin(m) * cos(o);
	mat.m13 = cos(m) * sin(i) * cos(o) + sin(m) * sin(o);
	mat.m21 = sin(m) * cos(i);
	mat.m22 = sin(m) * sin(i) * sin(o) + cos(m) * cos(o);
	mat.m23 = sin(m) * sin(i) * cos(o) - cos(m) * sin(o);
	mat.m31 = -sin(i);
	mat.m32 = cos(i) * sin(o);
	mat.m33 = cos(i) * cos(o);

	return mat;
}

MATRIX3 Supersighter::ADIAttSenseConversion(int intype, int outtype) const
{
	//Types: 0 = +X, 1 = -X, 2 = -Z
	if ((intype == 0 && outtype == 1) || (intype == 1 && outtype == 0)) return _M(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
	else if ((intype == 0 && outtype == 2) || (intype == 2 && outtype == 0)) return _M(0.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 0.0);
	else if (intype == 1 && outtype == 2) return _M(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
	else if (intype == 2 && outtype == 1) return _M(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0);

	return _M(1, 0, 0, 0, 1, 0, 0, 0, 1);
}

VECTOR3 Supersighter::MatrixToPYRAngles(const MATRIX3& M_XXX_BY) const
{
	// INPUTS:
	// M_XXX_BY: Matrix from X coordinate system to body
	// OUTPUTS:
	// Att: Attitude (output is RPY in Pitch, Yaw, Roll extraction sequence), radians

	VECTOR3 Att;

	Att.x = atan2(-M_XXX_BY.m32, M_XXX_BY.m22);
	Att.y = atan2(-M_XXX_BY.m13, M_XXX_BY.m11);
	Att.z = OrbMech::asin2(M_XXX_BY.m12);

	if (Att.x < 0.0) Att.x += PI2;
	if (Att.y < 0.0) Att.y += PI2;
	if (Att.z < 0.0) Att.z += PI2;

	return Att;
}

VECTOR3 Supersighter::MatrixToYPRAngles(const MATRIX3& M_XXX_BY) const
{
	// INPUTS:
	// M_XXX_BY: Matrix from X coordinate system to body
	// OUTPUTS:
	// Att: Attitude (output is RPY in Yaw, Pitch, Roll extraction sequence), radians

	VECTOR3 Att;

	Att.x = atan2(M_XXX_BY.m23, M_XXX_BY.m33);
	Att.y = asin(-M_XXX_BY.m13);
	Att.z = atan2(M_XXX_BY.m12, M_XXX_BY.m11);

	if (Att.x < 0.0) Att.x += PI2;
	if (Att.y < 0.0) Att.y += PI2;
	if (Att.z < 0.0) Att.z += PI2;

	return Att;
}

MATRIX3 Supersighter::LVLH_Matrix(VECTOR3 R, VECTOR3 V) const
{
	// Rotation matrix from inertial to LVLH

	VECTOR3 i, j, k;
	j = unit(crossp(V, R));
	k = unit(-R);
	i = crossp(j, k);
	return _M(i.x, i.y, i.z, j.x, j.y, j.z, k.x, k.y, k.z);
}

VECTOR3 Supersighter::CalculateEigenAxis(double P, double Y) const
{
	return _V(cos(P) * cos(Y), sin(Y), -sin(P) * cos(Y));
}

void Supersighter::CalculateEigenAxisPY(VECTOR3 e, double& P, double& Y) const
{
	Y = OrbMech::asin2(e.y);
	P = atan2(-e.z, e.x);
}

MATRIX3 Supersighter::RotationAroundAxis(VECTOR3 e, double theta) const
{
	MATRIX3 H, I;

	H = _M(0.0, -e.z, e.y, e.z, 0.0, -e.x, -e.y, e.x, 0.0);
	I = _M(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

	return I + mul(H, H) * (1.0 - cos(theta)) - H * sin(theta);
}

int Supersighter::HarmonicAdditionSolver(double a, double b, double f, double& theta1, double& theta2) const
{
	// Solves the general equation: f(theta) = a*cos(theta) + b*sin(theta) for theta

	double c, sin_delta, cos_delta, delta, temp;

	c = sqrt(a * a + b * b);

	// Solution possible?
	if (c == 0.0)
	{
		// No
		theta1 = theta2 = 0.0;
		return 1;
	}

	sin_delta = -b / c;
	cos_delta = a / c;
	delta = atan2(sin_delta, cos_delta);

	temp = f / c;

	// Solution possible?
	if (abs(temp) >= 1.0)
	{
		// No
		theta1 = theta2 = 0.0;
		return 1;
	}

	theta1 = acos(temp) - delta;
	if (theta1 < 0.0) theta1 += PI2;

	// Second solution
	c = -c;
	sin_delta = -b / c;
	cos_delta = a / c;
	delta = atan2(sin_delta, cos_delta);
	temp = f / c;
	theta2 = acos(temp) - delta;
	if (theta2 < 0.0) theta2 += PI2;

	return 0;
}

double Supersighter::CalculateIGA(double MGA, double OGA, VECTOR3 u_BY, VECTOR3 u_SM) const
{
	double a, b, c, d, e, f, ang;

	// Cos term in equation 1
	a = cos(MGA) * u_BY.x - sin(MGA) * cos(OGA) * u_BY.y + sin(MGA) * sin(OGA) * u_BY.z;
	// Sine term in equation 1
	b = sin(OGA) * u_BY.y + cos(OGA) * u_BY.z;
	// Constant term
	c = u_SM.x;
	// Cos term in equation 2
	d = sin(OGA) * u_BY.y + cos(OGA) * u_BY.z;
	// Sin term in equation 2
	e = -cos(MGA) * u_BY.x + sin(MGA) * cos(OGA) * u_BY.y - sin(MGA) * sin(OGA) * u_BY.z;
	// Constant term in equation 2
	f = u_SM.z;
	ang = TwoSineCosineEquations(a, b, c, d, e, f);
	if (ang < 0)
	{
		ang = ang + PI2;
	}
	return ang;
}

double Supersighter::TwoSineCosineEquations(double a, double b, double c, double d, double e, double f) const
{
	// a * cos(ang) + b * sin(ang) = c
	// d * cos(ang) + e * sin(ang) = f
	return atan2((a * f - c * d) / (a * e - b * d), (c * e - b * f) / (a * e - b * d));
}

VECTOR3 Supersighter::GetTargetDirection(const OrbMech::SV& sv, int type, int number) const
{
	// INPUTS:
	// type: 0 = center of Earth, 1 = center of Moon, 2 = center of Sun
	//OUTPUTS:
	// return value: Unit direction vector to target in TEG coordinates

	if (type == 0)
	{
		// Center of Earth
		return unit(-sv.R);
	}
	else if (type == 1)
	{
		// Center of Moon
		return unit(MOON(sv.GMT) - sv.R);
	}
	else if (type == 2)
	{
		// Center of Sun
		return unit(SUN(sv.GMT) - sv.R);
	}
	else if (type == 3)
	{
		// Celestial
		VECTOR3 u_M50 = inp.CTF->stars[number].u_vec;
		return tmul(inp.sescnst->M_TEG_TO_M50, u_M50);
	}
	else if (type == 4)
	{
		// Ground target
		VECTOR3 R_TEG = GroundTargetTEG(sv.GMT, number);
		return unit(R_TEG - sv.R);
	}
	else if (type == 5)
	{
		// Ephemeris
		OrbMech::SV sv_T_temp = OrbMech::coast_auto(inp.sv_T, sv.GMT - inp.sv_T.GMT, inp.useNonSphericalGravity);
		return unit(sv_T_temp.R - sv.R);
	}

	// Error really
	return _V(0, 0, 1);
}

VECTOR3 Supersighter::SUN(double GMT) const
{
	// Sun direction in TEG coordinates
	return OrbMech::SUN(inp.sescnst->GMTBASE, GMT, inp.sescnst->M_TEG_TO_M50);
}

VECTOR3 Supersighter::MOON(double GMT) const
{
	// Moon direction in TEG coordinates
	return OrbMech::MOON(inp.sescnst->GMTBASE, GMT, inp.sescnst->M_TEG_TO_M50);
}

VECTOR3 Supersighter::GroundTargetTEG(double GMT, int number) const
{
	// Calculation position vector of ground target in TEG coordinates
	VECTOR3 R_EF = OrbMech::r_from_latlong(inp.GTF->targets[number].Lat * RAD, inp.GTF->targets[number].Lng * RAD, (OrbMech::EARTH_RADIUS_ORBITER + inp.GTF->targets[number].Alt * OrbMech::FPS2MPS));
	MATRIX3 M_TEG_EF = OrbMech::TEG_to_EF_Matrix(OrbMech::w_Earth, GMT);
	return tmul(M_TEG_EF, R_EF);
}

double Supersighter::GroundTargetRange(const OrbMech::SV& sv, int number) const
{
	// Range to ground target

	VECTOR3 R_TEG;

	R_TEG = GroundTargetTEG(sv.GMT, number);

	return length(R_TEG - sv.R);
}

std::string Supersighter::OccultationCalculations(VECTOR3 R, double GMT, VECTOR3 u_LOS)
{
	// Earth, Moon, Sun, airglow

	// INPUTS:
	// R: Position vector (Earth relative)
	// GMT: Time of position vector
	// u_LOS: Instrument line-of-sight in same coordinate system as R

	VECTOR3 R_EM, R_ES, R_EC, R_MC, R_SC;
	bool los_E, los_M, los_S, los_A;

	// TBD: Put somewhere else later
	double R_E_mean = 6371000.0;
	double R_Moon = 1737400.0;
	double R_Sun = 695700000.0;

	// Get vectors to Sun and Moon
	R_ES = SUN(GMT);
	R_EM = MOON(GMT);

	// Build vectors from Sun and Moon to spacecraft
	R_EC = R;
	R_MC = R_EC - R_EM;
	R_SC = R_EC - R_ES;

	// Check line-of-sight
	los_E = OrbMech::LineOfSight(R_EC, u_LOS, R_E_mean);
	los_M = OrbMech::LineOfSight(R_MC, u_LOS, R_Moon);
	los_S = OrbMech::LineOfSight(R_SC, u_LOS, R_Sun);
	los_A = OrbMech::LineOfSight(R_EC, u_LOS, R_E_mean + 400000.0 * OrbMech::FPS2MPS);

	return FormatOcculation(!los_E, !los_M, !los_S, !los_A);
}

int Supersighter::FindAOS(const OrbMech::SV& sv, int type, int number, OrbMech::SV& sv_AOS, double &GMT_TCA, double &GMT_LOS)
{
	// Find AOS for all targets
	// Limit to 10 hours from initial state vector
	// If in AOS at input time, use that time

	// Center of Earth is always in AOS
	if (type == 0)
	{
		sv_AOS = sv;
		GMT_TCA = sv.GMT;
		GMT_LOS = sv.GMT + 10.0 * 3600.0;
		return 0;
	}
	else if (type == 5)
	{
		// Ephemeris
		// TBD
		sv_AOS = sv;
		GMT_TCA = sv.GMT;
		GMT_LOS = sv.GMT + 10.0 * 3600.0;
		return 0;
	}

	double tol, T_P, jump, range, GMT_pass, max_elevation, GMT_AOS;
	int max_iterations, i;

	// Initial settings
	max_iterations = 20;
	tol = 1.0;
	sv_temp = sv;
	TargetType = type;
	TargetNumber = number;
	T_P = OrbMech::REVTIM(sv.R, sv.V, inp.useNonSphericalGravity);
	jump = 1.0 * T_P;
	range = 0.25 * T_P;

	if (type == 4)
	{
		// Ground
		double Lat, Lng, Radius;

		Lat = inp.GTF->targets[number].Lat * RAD;
		Lng = inp.GTF->targets[number].Lng * RAD;
		Radius = inp.GTF->targets[number].Alt * OrbMech::FPS2MPS + OrbMech::EARTH_RADIUS_ORBITER;

		R_GS_EF = OrbMech::r_from_latlong(Lat, Lng, Radius);
	}

	// Find initial pass
	if (FindAOSInitialPass(sv, T_P, GMT_pass, max_elevation)) return 1;

	// Set up search for AOS
	double GeneralElevationCalcPointer(void* data, double var);
	double(*fptr)(void*, double) = &GeneralElevationCalcPointer;

	for (i = 0; i < max_iterations; i++) { //search for elevation above minimumElevation

		// Conditions for break
		if (max_elevation > 0.0)
		{
			GMT_TCA = GMT_pass;

			// Calculate AOS/LOS
			// AOS
			range = 0.5 * T_P;
			GMT_AOS = zbrent(fptr, GMT_pass, GMT_pass - range, tol, this);
			if (GMT_AOS < 0.0) return 1;

			// LOS
			GMT_LOS = zbrent(fptr, GMT_pass, GMT_pass + range, tol, this);
			if (GMT_LOS < 0.0) return 1;

			// Check if LOS is before input SV time. If not we have found our solution.
			if (GMT_LOS > sv.GMT)
			{
				break;
			}
		}

		GMT_pass += jump;
		max_elevation = -brentmin(GMT_pass - range, GMT_pass, GMT_pass + range, fptr, tol, &GMT_pass, this);
	}

	// Found within limit?
	if (i >= max_iterations) return 1;

	// Make sure times are in order and after start time
	if (GMT_AOS < sv.GMT) GMT_AOS = sv.GMT;
	if (GMT_TCA < GMT_AOS) GMT_TCA = GMT_AOS;
	if (GMT_LOS < GMT_TCA) GMT_LOS = GMT_TCA;

	// State vector to AOS time
	sv_AOS = OrbMech::coast_auto(sv_temp, GMT_AOS - sv_temp.GMT, inp.useNonSphericalGravity);

	return 0;
}

int Supersighter::FindAOSInitialPass(const OrbMech::SV& sv, double T_P, double& GMT_pass, double& maxElevation)
{
	double startpoint;
	double a, b, c, fa, fb, fc, tol;
	int i, MAX_itter;

	MAX_itter = 30;
	tol = 1.0;

	startpoint = sv.GMT;

	c = startpoint;
	fc = GeneralElevationCalc(c);
	b = startpoint - 0.166 * T_P;
	fb = GeneralElevationCalc(b);
	a = startpoint - 0.322 * T_P;
	fa = GeneralElevationCalc(a);

	for (i = 0; i < MAX_itter && (fb > fa || fb > fc); i++)
	{
		fc = fb;
		fb = fa;
		c = b;
		b = a;
		a = startpoint - 0.166 * (i + 3) * T_P;
		fa = GeneralElevationCalc(a);
	}
	if (i >= MAX_itter - 1) {
		return 1;
	}

	double GMT_min;

	double GeneralElevationCalcPointer(void* data, double var);
	double(*fptr)(void*, double) = &GeneralElevationCalcPointer;

	maxElevation = -brentmin(a, b, c, fptr, tol, &GMT_min, this);
	GMT_pass = GMT_min;

	return 0;
}

double GeneralElevationCalcPointer(void* data, double var)
{
	return ((Supersighter*)data)->GeneralElevationCalc(var);
}

double Supersighter::GeneralElevationCalc(double GMT)
{
	// Calculate elevation for all targets
	// Preset values: TargetType, R_TEG (for ground targets), TargetNumber (for celestial targets)

	double Elev;

	// Update state vector to desired time
	sv_temp = OrbMech::coast_auto(sv_temp, GMT - sv_temp.GMT, inp.useNonSphericalGravity);

	// Get target direction
	if (TargetType == 4)
	{
		// Ground
		MATRIX3 M_TEG_EF;
		VECTOR3 R_TEG, Rho_apo, N;
		M_TEG_EF = OrbMech::TEG_to_EF_Matrix(OrbMech::w_Earth, GMT);
		R_TEG = tmul(M_TEG_EF, R_GS_EF);

		Rho_apo = unit(sv_temp.R - R_TEG);
		N = unit(R_TEG);

		// Elevation angle
		Elev = asin(dotp(Rho_apo, N));
	}
	else
	{
		// Moon, Sun, Celestial

		VECTOR3 u_dir;
		double ang1, ang2;

		u_dir = GetTargetDirection(sv_temp, TargetType, TargetNumber);

		// Angle from local vertical to target
		ang1 = acos(dotp(-unit(sv_temp.R), u_dir));

		// Angle from local vertical to horizon
		ang2 = PI05 - OrbMech::acos2(OrbMech::EARTH_RADIUS_ORBITER / length(sv_temp.R));

		// Calculate elevation above horizon
		Elev = ang1 - ang2;
	}

	// Return negative of elevation (with bias from minimum elevation)
	return -Elev + inp.ELV * RAD;
}

int Supersighter::DecodeTarget(const std::string& TGT, int& type, int& number, bool IsTGT1) const
{
	if (TGT == "COE")
	{
		// Center of Earth
		type = 0;
		number = 0;
	}
	else if (TGT == "COM")
	{
		// Center of Moon
		type = 1;
		number = 0;
	}
	else if (TGT == "SUN")
	{
		// Center of Sun
		type = 2;
		number = 0;
	}
	else if (TGT.substr(0, 1) == "C")
	{
		// Celestial target
		type = 3;
		number = std::stoi(TGT.substr(1)) - 1;
		if (number < 0 || number >= 400) return 2;
	}
	else if (TGT.substr(0, 1) == "G")
	{
		// Ground target only allowed for target 1
		if (IsTGT1 == false) return 1;
		// Ground target
		type = 4;
		number = std::stoi(TGT.substr(1)) - 1;
		if (number < 0 || number >= 100) return 2;
		// No target set?
		if (inp.GTF->targets[number].Name == "") return 2;
	}
	else if (TGT.substr(0, 1) == "E")
	{
		// Ephemeris
		// TBD: No matter what else is entered, use target state vector
		// GMT = 0 means the SV is invalid
		if (inp.sv_T.GMT == 0.0) return 2;
		type = 5;
		number = 0;
	}
	else return 1;

	return 0;
}

int Supersighter::DecodeInstrument(const std::string& INST, int& number) const
{
	// Correct input format (3 characters)?
	if (INST.size() != 3) return 1;
	number = (std::stoi(INST.substr(1))) - 1;
	// Valud number?
	if (number < 0 || number > 24) return 1;
	// Is instrument initialized?
	if (inp.IDT[number].IsInitialized() == false) return 2;
	// All good
	return 0;
}

double Supersighter::GMTfromMET(double met) const
{
	return met + inp.sescnst->GMTLO;
}

double Supersighter::METfromGMT(double gmt) const
{
	return gmt - inp.sescnst->GMTLO;
}

void Supersighter::FormatGroundTarget(const OrbMech::SV& sv, int number)
{
	// Only target 1 for ground targets, so we can write to it directly

	GetGroundTargetInputs(number, outp.TGT1_LAT, outp.TGT1_LON, outp.TGT1_ALT);
	double RNG = GroundTargetRange(sv, number);
	outp.TGT1_RNG = FormatString("%.1lf", RNG / OrbMech::NM2M);
}

void Supersighter::GetGroundTargetInputs(int number, std::string& LAT, std::string& LNG, std::string& ALT)
{
	LAT = FormatDeclination(inp.GTF->targets[number].Lat);
	LNG = FormatLongitude(inp.GTF->targets[number].Lng);
	ALT = FormatString("%.0lf", inp.GTF->targets[number].Alt);
}

void Supersighter::FormatCelestialTarget(VECTOR3 u_TGT_M50, std::string& strRA, std::string& strDEC)
{
	double DEC, RA;

	OrbMech::latlong_from_r(u_TGT_M50, DEC, RA);
	if (RA < 0.0) RA += PI2;
	strRA = FormatAttitude(RA * DEG);
	strDEC = FormatDeclination(DEC * DEG);
}

void Supersighter::FormatVehicleTarget(const OrbMech::SV& sv)
{
	// Only for TGT 1

	OrbMech::SV sv_T_temp;
	std::string strRA, strDEC;

	VECTOR3 DR, u_dir_TEG, u_dir_M50;
	double range;

	// Take target to SV time
	sv_T_temp = OrbMech::coast_auto(inp.sv_T, sv.GMT - inp.sv_T.GMT, inp.useNonSphericalGravity);
	// Calculate relative position
	DR = sv_T_temp.R - sv.R;
	// Calculate range
	range = length(DR);
	// Calculate direction vector
	u_dir_TEG = unit(DR);
	u_dir_M50 = mul(inp.sescnst->M_TEG_TO_M50, u_dir_TEG);

	FormatCelestialTarget(u_dir_M50, strRA, strDEC);

	outp.VEH_RANGE = FormatString("%.1lf", range / OrbMech::NM2M);
	outp.TGT1_RA = strRA;
	outp.TGT1_DEC = strDEC;
}

std::string Supersighter::FormatString(const char* _Format, double Val)
{
	std::string str;

	sprintf_s(Buffer, _Format, Val);
	str.assign(Buffer);
	return str;
}

std::string Supersighter::FormatAttitude(double Att)
{
	if (Att >= 359.995)
	{
		Att = 0.0;
	}

	return FormatString("%.2lf", Att);
}

std::string Supersighter::FormatInstrumentAngle(double Ang)
{
	return FormatString("%+.2lf", Ang);
}

std::string Supersighter::FormatInstrumentLimit(bool Limit)
{
	if (Limit)
	{
		return "*";
	}
	return " ";
}

std::string Supersighter::FormatAttSense()
{
	if (inp.ATTSense == 0) return "+X";
	else if (inp.ATTSense == 1) return "-X";
	else return "-Z";
}

std::string Supersighter::FormatOcculation(bool E, bool M, bool S, bool A)
{
	// true = is occulted

	std::string temp;

	if (E) temp.append("E");
	else temp.append(" ");
	if (M) temp.append("M");
	else temp.append(" ");
	if (S) temp.append("S");
	else temp.append(" ");
	if (A) temp.append("A");
	else temp.append(" ");
	
	return temp;
}

std::string Supersighter::FormatDeclination(double DEC)
{
	std::string temp;

	temp = FormatString("%05.2lf", abs(DEC));
	if (DEC >= 0.0)
	{
		temp.append("N");
	}
	else
	{
		temp.append("S");
	}
	return temp;
}

std::string Supersighter::FormatLongitude(double LNG)
{
	std::string temp;

	temp = FormatString("%06.2lf", abs(LNG));
	if (LNG >= 0.0)
	{
		temp.append("E");
	}
	else
	{
		temp.append("W");
	}
	return temp;
}

std::string Supersighter::MET2String(double MET)
{
	// Format: DDD:HH:MM:SS
	MET = round(MET);
	sprintf_s(Buffer, "%03.0f:%02.0f:%02.0f:%02.0f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
	std::string temp(Buffer);
	return temp;
}

std::string Supersighter::GMT2String(double GMT)
{
	// Format: DDD:HH:MM:SS
	GMT = round(GMT);
	sprintf_s(Buffer, "%03.0f:%02.0f:%02.0f:%02.0f", floor(GMT / 86400.0) + (double)inp.sescnst->DayOfYear, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
	std::string temp(Buffer);
	return temp;
}