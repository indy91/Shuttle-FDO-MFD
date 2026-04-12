#pragma once

#include "Orbitersdk.h"

class InstrumentDefinitionTable
{
public:
	InstrumentDefinitionTable();

	int BuildInstrumentData(const std::string& comment, int INSTR_TYPE, double Phi1, double Theta, double Phi2, double A1_MIN, double A1_MAX, double A2_MIN, double A2_MAX, bool RET_ID, MATRIX3 MT_MAT);
	
	struct InstrumentDefinitionTableInputs
	{
		InstrumentDefinitionTableInputs();
		std::string Comment;
		int INSTR_TYPE;
		double Phi1;
		double Theta;
		double Phi2;
		double A1_MIN, A1_MAX, A2_MIN, A2_MAX;
		bool RET_ID;
	};

	// Conversions
	void BodyVectorToInstrumentAngles(VECTOR3 u_BY, double &A1, double &A2, bool &Limit1, bool &Limit2) const;
	VECTOR3 InstrumentAnglesToBodyVector(double A1, double A2) const;

	bool IsInitialized() const;

	InstrumentDefinitionTableInputs GetInputs() const;
private:

	// INTERNAL FUNCTIONS

	// Vector in instrument coordinates to instrument angles
	void InstrumentVectorToInstrumentAngles(VECTOR3 u_IN, double &A1, double &A2) const;
	void InstrumentVectorToInstrumentAnglesEuler(VECTOR3 u_IN, double& A1, double& A2) const;
	void InstrumentVectorToInstrumentAnglesIndep(VECTOR3 u_IN, double& A1, double& A2) const;
	// Instrument angles to instrument coordinate system
	VECTOR3 InstrumentAnglesToInstrumentCoordinates(double A1, double A2) const;
	VECTOR3 InstrumentAnglesToInstrumentCoordinatesIndep(double A1, double A2) const;

	// Rotation around X, Y, Z axis
	MATRIX3 RotX(double alpha) const;
	MATRIX3 RotY(double alpha) const;
	MATRIX3 RotZ(double alpha) const;
	// Compute indices for conversions
	void ComputeINtoICConversion();

	// Set when the instrument table has been successfully built
	int Initialized;

	// INTERNAL DATA
	double A1_MIN, A1_MAX, A2_MIN, A2_MAX;
	int e[3];
	bool RET_ID; // 0 = no reticle, 1 = reticle (hor, vert)
	// Mount matrix
	MATRIX3 M, N;
	int I, J, K, L, SI, SJ, SK, S2;

	// INPUT DATA
	InstrumentDefinitionTableInputs Inputs;
};
