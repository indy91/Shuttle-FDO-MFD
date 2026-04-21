/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) Niklas Beug

  Supersighter Display (Header)

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
#include "OrbMech.h"
#include "InstrumentDefinitionTable.h"

struct CelestialTargetFileEntry
{
	char Name[32];
	VECTOR3 u_vec;
	double MAG;
};

struct CelestialTargetFile
{
	CelestialTargetFileEntry stars[400];
};

struct GroundTargetFileEntry
{
	std::string Name;
	double Lat = 0.0; // Degrees
	double Lng = 0.0; // Degrees
	double Alt = 0.0; // Feet
};

struct GroundTargetFile
{
	void Set(int num, std::string name, double lat, double lng, double alt)
	{
		if (num < 1 || num > 100) return;
		targets[num - 1].Name = name;
		targets[num - 1].Lat = lat;
		targets[num - 1].Lng = lng;
		targets[num - 1].Alt = alt;
	}

	GroundTargetFileEntry targets[100];
};

struct SupersighterInputs
{
	SupersighterInputs();

	// 1 = Moveable line-of-sight, 2 = Fixed attitude/fixed line-of-sight, 3 = Fixed line-of-sight rotation, 4 = Minimum maneuver
	// 5 = Fixed line-of-sight/MGA input, 6 = Dual line-of-sight, 7 = Fixed line-of-sight/omicron input
	int Mode;
	// Input matrix id. Valid inputs are: RLMTxx  = ADI reference matrix (RELMAT), RFMTxx = IMU reference matrix (REFSMMAT)
	// LPYR = Input ATT is LVLH with a P, Y, R Euler sequence, LYPR = Input ATT is LVLH with a Y, P, R Euler sequence
	std::string INMAT;
	std::string OUTMAT;
	// Elevation angle, degrees
	double ELV;
	// Target 1 ID
	std::string TGT1;
	// Target 2 ID
	std::string TGT2;
	// Instrument ID for output A attitude and target 1
	std::string IA1;
	// Instrument ID for output A attitude and target 2
	std::string IA2;
	// Instrument ID for output B attitude and target 1
	std::string IB1;
	// Instrument ID for output B attitude and target 2
	std::string IB2;
	// 0 = +X, 1 = -X, 2 = -Z
	int ATTSense;
	// Input/Angles
	VECTOR3 ATT;
	// Instrument angles, degrees
	double IA1_A1, IA1_A2;
	double IA2_A1, IA2_A2;
	double IB1_A1, IB1_A2;
	double IB2_A1, IB2_A2;
	// Middle gimbal angle (mode 5), degrees
	double MGA;
	// Omicron
	double OMI;
	VECTOR3 EIG;
	double StartTime;
	// TBD: Ephemeris or vector
	// The source bias matrix used with the source Relative Matrix (RELMAT) for local vertical local horizontal (LVLH) offset attitude reference.
	// This matrix ID will define whether the input LVLH attitude is offset and either a P, Y, R or Y, P, R Euler angle sequence.
	std::string IP_LVLH_BIAS;
	// The desired bias matrix used with the desired reference matrix for LVLH offset attitude reference.
	// This matrix ID will define whether the output A&B LVLH attitudes are either a P, Y, R or Y, P, R Euler angle sequence.
	std::string OP_LVLH_BIAS;

	// State vector or ephemeris
	OrbMech::SV sv0;
	// Target state vector
	OrbMech::SV sv_T;

	// Pointers to tables

	// Instrument Definitions
	InstrumentDefinitionTable* IDT;
	// Instrument Mount Matrix Table
	InstrumentMountMatrix* IMT;
	// Celestial targets
	CelestialTargetFile* CTF;
	// Ground targets
	GroundTargetFile* GTF;
	// RELMAT, REFSMMAT, BIAS
	OrbMech::SessionConstants *sescnst;

	// Gravit selection
	bool useNonSphericalGravity;
};

struct SupersighterOutputs
{
	std::string INMAT;
	std::string OUTMAT;
	std::string ATT_SOURCE;
	std::string EPH;
	std::string VID;
	std::string IN_LVLH_BIAS;
	std::string OUT_LVLH_BIAS;
	std::string MODE;
	std::string EIGEN_VECTOR_P;
	std::string EIGEN_VECTOR_Y;
	std::string EIGEN_ANG;
	std::string ELV;
	std::string ATT_SENSE;
	std::string INPUT_ATT[3];
	std::string MGA;
	std::string VEH_RANGE;
	std::string MODE2_LAT;
	std::string MODE2_LON;
	std::string OUTPUT_A_ATT[6][3];
	std::string OUTPUT_A_ATT_REF;
	std::string OUTPUT_B_ATT[6][3];
	std::string OUTPUT_B_ATT_REF;
	std::string TGT1;
	std::string TGT1_RA;
	std::string TGT1_DEC;
	std::string TGT1_LAT;
	std::string TGT1_LON;
	std::string TGT1_ALT;
	std::string TGT1_RNG;
	std::string TGT2_OCC;
	std::string TGT2;
	std::string TGT2_RA;
	std::string TGT2_DEC;
	std::string IA1, IA2, IB1, IB2;
	std::string IA1_OCC, IA2_OCC, IB1_OCC, IB2_OCC;
	std::string IA1_A1, IA1_A1_LIM, IA1_A2, IA1_A2_LIM, IA1_A3;
	std::string IA2_A1, IA2_A1_LIM, IA2_A2, IA2_A2_LIM;
	std::string IB1_A1, IB1_A1_LIM, IB1_A2, IB1_A2_LIM;
	std::string IB2_A1, IB2_A1_LIM, IB2_A2, IB2_A2_LIM;

	std::string ST_MET, ST_GMT;
	std::string AOS_MET, AOS_GMT;
	std::string TCA_MET, TCA_GMT;
	std::string LOS_MET, LOS_GMT;

	std::string RA_PX, DEC_PX, RA_MZ, DEC_MZ;
	std::string Pitch_E, Yaw_E, Theta_E, Phi_E;
	std::string Pitch_M, Yaw_M, Theta_M, Phi_M;
	std::string Pitch_S, Yaw_S, Theta_S, Phi_S;

	std::string ErrorMessage;
};

class Supersighter
{
public:
	Supersighter();

	void RUN(const SupersighterInputs& in, SupersighterOutputs &out);

	// Functions for class pointers
	double GeneralElevationCalc(double GMT);
private:
	// Move-able Line-of-Sight Mode
	void MODE1();
	// Fixed Attitude/Fixed Line-of-Sight Mode
	void MODE2();
	// Fixed Line-of-Sight Rotation Mode
	void MODE3();
	// Minimum Maneuver Mode
	void MODE4();
	// Fixed line-of-sight/MGA Mode
	void MODE5();
	// Optimum Second Line-of-Sight Mode
	void MODE6();
	// Fixed Line-of-Sight Omicron Mode
	void MODE7();

	// SUBROUTINES

	// Instrument conversions
	void GetInstrumentAnglesFromVector(VECTOR3 u_BY, int n, double& A1, double& A2, bool &Limit1, bool& Limit2) const;
	VECTOR3 GetVectorFromInstrumentAngles(int n, double A1, double A2) const;

	// Common calculations
	void CommonCalculations(const OrbMech::SV& sv, const MATRIX3& B_M50_BY);

	// P/Y and R/P sequences for display
	void VectorToPY(VECTOR3 u_BY, double& P, double& Y) const;
	void VectorToPhiTheta(VECTOR3 u_BY, double &Phi, double &Theta) const;

	// Mode specific calculations
	int Mode2EarthIntersection(VECTOR3 R_TEG, double GMT, VECTOR3 u_M, double &Lat, double &Lng) const;
	MATRIX3 Mode4Attitude(const MATRIX3& B0_M50_BY, VECTOR3 P, VECTOR3 T, double& EIG_P, double& EIG_Y, double& EIG_ANG) const;
	int Mode5Attitude(VECTOR3 u_BY, VECTOR3 u_SM, double MGA, VECTOR3& Att1, VECTOR3& Att2) const;
	int Mode6OptimumSecondLineOfSight(VECTOR3 P1, VECTOR3 T1, VECTOR3 P2, VECTOR3 T2, MATRIX3 & B_M50_BY) const;
	int ComputeOmicron(VECTOR3 R, VECTOR3 V, const MATRIX3& B_M50_BY, VECTOR3 T, double &OMI) const;
	MATRIX3 Mode7Attitude(VECTOR3 R_M50, VECTOR3 V_M50, VECTOR3 P_BY, VECTOR3 T_M50, double OMICRON) const;

	// Input and output attitude
	int CalculateBodyMatrixFromAttitude(const OrbMech::SV& sv, MATRIX3 &B_M50_BY) const;
	int CalculateAttitudeFromBodyMatrix(const OrbMech::SV& sv, MATRIX3 B_M50_BY, bool IsAttitudeA);

	// More utility functions
	int GetRELMAT(std::string relmat, MATRIX3 &mat) const;
	int GetREFSMMAT(std::string relmat, MATRIX3& mat) const;
	int GetLVLHBiasMatrix(std::string relmat, MATRIX3& mat) const;

	// Attitude conversions
	MATRIX3 PYRAnglesToMatrix(double R, double P, double Y) const;
	MATRIX3 YPRAnglesToMatrix(double R, double P, double Y) const;
	MATRIX3 ADIAttSenseConversion(int intype, int outtype) const;
	VECTOR3 MatrixToPYRAngles(const MATRIX3& M_XXX_BY) const;
	VECTOR3 MatrixToYPRAngles(const MATRIX3& M_XXX_BY) const;
	MATRIX3 LVLH_Matrix(VECTOR3 R, VECTOR3 V) const;

	// Eigen angle
	VECTOR3 CalculateEigenAxis(double P, double Y) const;
	void CalculateEigenAxisPY(VECTOR3 e, double& P, double& Y) const;
	MATRIX3 RotationAroundAxis(VECTOR3 e, double theta) const;

	// Mode 5 helper functions
	int HarmonicAdditionSolver(double a, double b, double f, double &theta1, double &theta2) const;
	double CalculateIGA(double MGA, double OGA, VECTOR3 u_BY, VECTOR3 u_SM) const;
	double TwoSineCosineEquations(double a, double b, double c, double d, double e, double f) const;

	// Target calculations
	VECTOR3 GetTargetDirection(const OrbMech::SV& sv, int type, int number) const;
	VECTOR3 SUN(double GMT) const;
	VECTOR3 MOON(double GMT) const;
	VECTOR3 GroundTargetTEG(double GMT, int number) const;
	double GroundTargetRange(const OrbMech::SV& sv, int number) const;

	// AOS/LOS and occultation
	std::string OccultationCalculations(VECTOR3 R, double GMT, VECTOR3 u_LOS);
	int FindAOS(const OrbMech::SV& sv, int type, int number, OrbMech::SV& sv_AOS, double &GMT_TCA, double &GMT_LOS);
	int FindAOSInitialPass(const OrbMech::SV& sv, double T_P, double &GMT_pass, double &maxElevation);

	// Decode inputs
	int DecodeTarget(const std::string &TGT, int &type, int &number, bool IsTGT1 = true) const;
	int DecodeInstrument(const std::string& INST, int& number) const;

	// Time conversions
	double GMTfromMET(double met) const;
	double METfromGMT(double gmt) const;

	// Output formatting
	void FormatGroundTarget(const OrbMech::SV& sv, int number);
	void GetGroundTargetInputs(int number, std::string& LAT, std::string& LNG, std::string& ALT);
	void FormatCelestialTarget(VECTOR3 u_TGT_M50, std::string& strRA, std::string& strDEC);
	void FormatVehicleTarget(const OrbMech::SV& sv);
	std::string FormatString(const char* _Format, double Val);
	std::string FormatAttitude(double Att);
	std::string FormatInstrumentAngle(double Ang);
	std::string FormatInstrumentLimit(bool Limit);
	std::string FormatAttSense();
	std::string FormatOcculation(bool E, bool M, bool S, bool A);
	std::string FormatDeclination(double DEC);
	std::string FormatLongitude(double LNG);
	std::string MET2String(double MET);
	std::string GMT2String(double GMT);

	SupersighterInputs inp;
	SupersighterOutputs outp;

	// State vector at start time
	OrbMech::SV sv_ST;
	// Temporary variables
	char Buffer[128];
	// For AOS/TCA/LOS calculations
	// Temporary state vector
	OrbMech::SV sv_temp;
	// Earth-fixed position of ground station
	VECTOR3 R_GS_EF;
	// Type of target (1 = Moon, 2 = Sun, 3 = Celestial, 4 = Ground)
	int TargetType;
	int TargetNumber;

	// CONSTANTS
	// General tolerance of dot product between two vectors
	const double eps = 0.00001;
	// Universal Pointing tolerance (cosine of one degree)
	const double UP_TOL = 0.999848;
};