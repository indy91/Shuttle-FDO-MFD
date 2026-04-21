#pragma once

#include "Orbitersdk.h"

struct InstrumentMountMatrix
{
	std::string Comment;
	MATRIX3 MAT = identity(); // Rotation matrix from the Orbiter body coordinate system to the instrument mount coordinate system
};

struct InstrumentDefinitionTableEntry
{
	InstrumentDefinitionTableEntry();

	int FormatInstrumentType() const;

	// Description of instrument
	char Comment[17];
	// Instrument type (1 = X, 2 = Y, 3 = Z, negative sign = negative axis)
	int e[3];
	// ID of mount (in mount matrix table)
	int Mount;
	// First Euler angle rotation to coordinate system of instrument null orientation (X-axis rotation), degrees
	double Phi1;
	// Second Euler angle rotation to coordinate system of instrument null orientation (Y-axis rotation), degrees
	double Theta;
	// Third Euler angle rotation to coordinate system of instrument null orientation (X-axis rotation), degrees
	double Phi2;
	// Angle limits, degrees
	double A1_MIN, A1_MAX, A2_MIN, A2_MAX;
	// Program control parameter designating a reticle pattern which requires mathematical techniques different from the standard (if non-zero)
	int RET_ID;
};

class InstrumentDefinitionTable
{
public:
	InstrumentDefinitionTable();

	int BuildInstrumentData(const std::string& comment, int INSTR_TYPE, int Mount, double Phi1, double Theta, double Phi2, double A1_MIN, double A1_MAX, double A2_MIN, double A2_MAX, int RET_ID);

	// Conversions
	void BodyVectorToInstrumentAngles(InstrumentMountMatrix *MT, VECTOR3 u_BY, double &A1, double &A2, bool &Limit1, bool &Limit2) const;
	VECTOR3 InstrumentAnglesToBodyVector(InstrumentMountMatrix* MT, double A1, double A2) const;

	bool IsInitialized() const;

	InstrumentDefinitionTableEntry GetInputs() const;
private:

	// Instrument defining data
	struct InstrumentDefinitionTableVariable
	{
		MATRIX3 N;
		int I, J, K, L, SI, SJ, SK, S2;
	};

	// INTERNAL FUNCTIONS

	// Vector in instrument coordinates to instrument angles
	void InstrumentVectorToInstrumentAngles(const InstrumentDefinitionTableVariable& var, VECTOR3 u_IN, double& A1, double& A2) const;
	void InstrumentVectorToInstrumentAnglesEuler(const InstrumentDefinitionTableVariable& var, VECTOR3 u_IN, double& A1, double& A2) const;
	void InstrumentVectorToInstrumentAnglesIndep(const InstrumentDefinitionTableVariable& var, VECTOR3 u_IN, double& A1, double& A2) const;
	// Instrument angles to instrument coordinate system
	VECTOR3 InstrumentAnglesToInstrumentCoordinates(const InstrumentDefinitionTableVariable& var, double A1, double A2) const;
	VECTOR3 InstrumentAnglesToInstrumentCoordinatesIndep(const InstrumentDefinitionTableVariable& var, double A1, double A2) const;

	// Convert inputs to common variables
	void PreliminaryCalculations(InstrumentDefinitionTableVariable &var) const;

	// Rotation around X, Y, Z axis
	MATRIX3 RotX(double alpha) const;
	MATRIX3 RotY(double alpha) const;
	MATRIX3 RotZ(double alpha) const;

	// Set when the instrument table has been successfully built
	int Initialized;

	// INPUT DATA
	InstrumentDefinitionTableEntry Inputs;
};
