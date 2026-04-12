#include "InstrumentDefinitionTable.h"

InstrumentDefinitionTable::InstrumentDefinitionTableInputs::InstrumentDefinitionTableInputs()
{
	INSTR_TYPE = 231;
	Phi1 = Theta = Phi2 = 0.0;
	A1_MIN = A2_MIN = 0.0;
	A1_MAX = 360.0;
	A2_MAX = 180.0;
	RET_ID = false;
}

InstrumentDefinitionTable::InstrumentDefinitionTable()
{
	Initialized = false;
	e[0] = e[1] = e[2] = 0;
	RET_ID = false;
	A1_MIN = A1_MAX = A2_MIN = A2_MAX = 0.0;
	M = N = _M(0, 0, 0, 0, 0, 0, 0, 0, 0);
	I = J = K = L = SI = SJ = SK = S2 = 0;
}

int InstrumentDefinitionTable::BuildInstrumentData(const std::string& comment, int INSTR_TYPE, double Phi1, double Theta, double Phi2, double A1_MIN, double A1_MAX, double A2_MIN, double A2_MAX, bool RET_ID, MATRIX3 MT_MAT)
{
	// INPUTS:
	// INSTR_TYPE: A three-digit number ABC that defines the rotation sequence of the instrument and the null axis of the instrument
	// A - axis for 1st rotation
	// B - axis for 2nd rotation
	// C - null axis of the instrument
	// Phi1: Euler angle of rotation about X mount axis to the X', Y', Z' coordinate system, degrees
	// Theta: Euler angle of rotation about Y' axis to the X'', Y'', Z'' coordinate system, degrees
	// Phi2: Euler angle of rotation about X'' axis to the instrument coordinate system
	// OUTPUTS:
	// Return value = error if non-zero

	int TYPE[3];

	TYPE[0] = INSTR_TYPE / 100;
	TYPE[1] = (INSTR_TYPE - TYPE[0] * 100) / 10;
	TYPE[2] = INSTR_TYPE - TYPE[0] * 100 - TYPE[1] * 10;

	// Check if all are between 1-6
	for (int i = 0; i < 3; i++)
	{
		if (TYPE[i] < 1 || TYPE[i] > 6)
		{
			// Error
			return 1;
		}

		// Build e array
		if (TYPE[i] >= 4)
		{
			e[i] = -(TYPE[i] - 3);
		}
		else
		{
			e[i] = TYPE[i];
		}
	}

	// TBD: Additional error checks (illegal type)

	this->RET_ID = RET_ID;

	// Degrees to radians
	this->A1_MIN = A1_MIN * RAD;
	this->A1_MAX = A1_MAX * RAD;
	this->A2_MIN = A2_MIN * RAD;
	this->A2_MAX = A2_MAX * RAD;

	// Matrix from Orbiter body coordinate system to instrument mount coordinate system
	M = MT_MAT;
	// Matrix from mount system to instrument system
	N = mul(RotX(Phi2 * RAD), mul(RotY(Theta * RAD), RotX(Phi1 * RAD)));
	// Compute parameters for the conversion between instrument computation frame and instrument null frame
	ComputeINtoICConversion();

	// Store inputs
	Inputs.Comment = comment;
	Inputs.INSTR_TYPE = INSTR_TYPE;
	Inputs.Phi1 = Phi1;
	Inputs.Theta = Theta;
	Inputs.Phi2 = Phi2;
	Inputs.A1_MIN = A1_MIN;
	Inputs.A1_MAX = A1_MAX;
	Inputs.A2_MIN = A2_MIN;
	Inputs.A2_MAX = A2_MAX;
	Inputs.RET_ID = RET_ID;

	Initialized = true;
	return 0;
}

void InstrumentDefinitionTable::BodyVectorToInstrumentAngles(VECTOR3 u_BY, double& A1, double& A2, bool& Limit1, bool& Limit2) const
{
	// INPUTS:
	// OUTPUTS:

	VECTOR3 u_IN;

	// Convert vector from body coordinates to instrument cordinates
	u_IN = mul(mul(N, M), u_BY);
	InstrumentVectorToInstrumentAngles(u_IN, A1, A2);

	// Check limits
	if (A1 < A1_MIN || A1 > A1_MAX)
	{
		Limit1 = true;
	}
	else
	{
		Limit1 = false;
	}
	if (A2 < A2_MIN || A2 > A2_MAX)
	{
		Limit2 = true;
	}
	else
	{
		Limit2 = false;
	}
}

VECTOR3 InstrumentDefinitionTable::InstrumentAnglesToBodyVector(double A1, double A2) const
{
	// INPUTS:
	// A1: Instrument angle 1, radians
	// A2: Instrument angle 2, radians
	// OUTPUTS:
	// u_BY: Unit direction vector in body coordinates

	VECTOR3 u_IN, u_BY;

	u_IN = InstrumentAnglesToInstrumentCoordinates(A1, A2);
	u_BY = tmul(mul(N, M), u_IN);

	return u_BY;
}

void InstrumentDefinitionTable::InstrumentVectorToInstrumentAngles(VECTOR3 u_IN, double& A1, double& A2) const
{
	// INPUTS:
	// u_IN: Unit vector in instrument coordinate system
	// OUTPUTS:
	// A1: Instrument angle 1, radians (always 0-360°)
	// A2: Instrument angle 2, radians (either 0-180° or -90° to 90°)

	// TBD: Euler vs independent
	InstrumentVectorToInstrumentAnglesEuler(u_IN, A1, A2);
}

void InstrumentDefinitionTable::InstrumentVectorToInstrumentAnglesEuler(VECTOR3 u_IN, double& A1, double& A2) const
{
	// Euler sequences
	if (I == L)
	{
		A1 = atan2(u_IN.data[K - 1] * (double)(S2 * SK), u_IN.data[J - 1] * (double)SJ);
		A2 = acos(u_IN.data[I - 1] * (double)SI);
	}
	else
	{
		A1 = atan2(u_IN.data[J - 1] * (double)SJ, u_IN.data[I - 1] * (double)SI);
		A2 = asin(-u_IN.data[K - 1] * (double)(S2 * SK));
	}
}

void InstrumentDefinitionTable::InstrumentVectorToInstrumentAnglesIndep(VECTOR3 u_IN, double& A1, double& A2) const
{
	// Independent sequences
	A1 = atan2(u_IN.data[J - 1] * (double)SJ, u_IN.data[I - 1] * (double)SI);
	A2 = atan2(u_IN.data[K - 1] * (double)SK, u_IN.data[I - 1] * (double)SI);
}

VECTOR3 InstrumentDefinitionTable::InstrumentAnglesToInstrumentCoordinates(double A1, double A2) const
{
	// INPUTS:
	// A1: Instrument angle 1, radians
	// A2: Instrument angle 2, radians
	// OUTPUTS:
	// return value: unit pointing vector in instrument coordinates

	VECTOR3 u_IN;
	double SA1, CA1, SA2, CA2;

	SA1 = sin(A1);
	CA1 = cos(A1);
	SA2 = sin(A2);
	CA2 = cos(A2);

	if (I == L)
	{
		u_IN.data[I - 1] = CA2 * (double)SI;
		u_IN.data[J - 1] = CA1 * SA2 * (double)SJ;
		u_IN.data[K - 1] = SA1 * SA2 * (double)(S2 * SK);
	}
	else
	{
		u_IN.data[I - 1] = CA1 * CA2 * (double)SI;
		u_IN.data[J - 1] = SA1 * CA2 * (double)SJ;
		u_IN.data[K - 1] = -SA2 * (double)(S2 * SK);
	}

	return u_IN;
}

VECTOR3 InstrumentDefinitionTable::InstrumentAnglesToInstrumentCoordinatesIndep(double A1, double A2) const
{
	// Independent instrument rotation sequence

	// INPUTS:
	// A1: Instrument angle 1, radians
	// A2: Instrument angle 2, radians
	// OUTPUTS:
	// return value: unit pointing vector in instrument coordinates

	VECTOR3 u_IN;

	u_IN.data[I - 1] = (double)(SI);
	u_IN.data[J - 1] = tan(A1) * (double)(SJ);
	u_IN.data[K - 1] = tan(A2) * (double)(SK);

	return unit(u_IN);
}

bool InstrumentDefinitionTable::IsInitialized() const
{
	return Initialized;
}

InstrumentDefinitionTable::InstrumentDefinitionTableInputs InstrumentDefinitionTable::GetInputs() const
{
	return Inputs;
}

MATRIX3 InstrumentDefinitionTable::RotX(double alpha) const
{
	double CA, SA;

	CA = cos(alpha);
	SA = sin(alpha);

	return _M(1.0, 0.0, 0.0, 0.0, CA, SA, 0.0, -SA, CA);
}

MATRIX3 InstrumentDefinitionTable::RotY(double alpha) const
{
	double CA, SA;

	CA = cos(alpha);
	SA = sin(alpha);

	return _M(CA, 0.0, -SA, 0.0, 1.0, 0.0, SA, 0.0, CA);
}

MATRIX3 InstrumentDefinitionTable::RotZ(double alpha) const
{
	double CA, SA;

	CA = cos(alpha);
	SA = sin(alpha);

	return _M(CA, SA, 0.0, -SA, CA, 0.0, 0.0, 0.0, 1.0);
}

int sign(int val)
{
	if (val >= 0) return 1;
	else return -1;
}

void InstrumentDefinitionTable::ComputeINtoICConversion()
{
	int S, SL;

	I = abs(e[2]);
	K = L = abs(e[0]);
	SI = sign(e[2]);
	SK = SL = sign(e[0]);
	S = sign(e[1]);
	if (I == L)
	{
		K = abs(e[1]);
		SK = S;
	}
	J = 6 - I - K;
	SJ = SI * SK;
	if (I != (K + 1))
	{
		if (I != (K - 2))
		{
			SJ = -SJ;
		}
	}
	if (I == L)
	{
		S2 = SL * SI;
	}
	else
	{
		S2 = SJ * S;
	}
}