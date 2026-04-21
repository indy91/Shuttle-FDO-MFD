#include "InstrumentDefinitionTable.h"

InstrumentDefinitionTableEntry::InstrumentDefinitionTableEntry()
{
	sprintf(Comment, "");
	e[0] = e[1] = e[2] = 0;
	Mount = 0;
	Phi1 = 0.0;
	Theta = 0.0;
	Phi2 = 0.0;
	A1_MIN = 0.0;
	A1_MAX = 0.0;
	A2_MIN = 0.0;
	A2_MAX = 0.0;
	RET_ID = 0;
}

int InstrumentDefinitionTableEntry::FormatInstrumentType() const
{
	// Format e vector (1-3 and -3 to -1) to original instrument type (1-6)
	int ID, ITEMP;

	ID = 0;

	for (int i = 0; i < 3; i++)
	{
		if (e[i] > 0)
		{
			ITEMP = e[i];
		}
		else
		{
			ITEMP = 3 - e[i];
		}
		if (i == 0)
		{
			ID += ITEMP * 100;
		}
		else if (i == 1)
		{
			ID += ITEMP * 10;
		}
		else
		{
			ID += ITEMP;
		}
	}
	return ID;
}

InstrumentDefinitionTable::InstrumentDefinitionTable()
{
	Initialized = false;
}

int InstrumentDefinitionTable::BuildInstrumentData(const std::string& comment, int INSTR_TYPE, int Mount, double Phi1, double Theta, double Phi2, double A1_MIN, double A1_MAX, double A2_MIN, double A2_MAX, int RET_ID)
{
	// INPUTS:
	// INSTR_TYPE: A three-digit number ABC that defines the rotation sequence of the instrument and the null axis of the instrument
	// A - axis for 1st rotation
	// B - axis for 2nd rotation
	// C - null axis of the instrument
	// Mount: ID of mount matrix (typically 1-16)
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
			Inputs.e[i] = -(TYPE[i] - 3);
		}
		else
		{
			Inputs.e[i] = TYPE[i];
		}
	}

	// Additional error checks (TBD: illegal type)
	if (Mount < 1 || Mount > 16) return 1;

	// Store other inputs
	strncpy(Inputs.Comment, comment.c_str(), sizeof(Inputs.Comment));
	Inputs.Mount = Mount;
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

void InstrumentDefinitionTable::BodyVectorToInstrumentAngles(InstrumentMountMatrix* MT, VECTOR3 u_BY, double& A1, double& A2, bool& Limit1, bool& Limit2) const
{
	// INPUTS:
	// OUTPUTS:

	InstrumentDefinitionTableVariable var;
	VECTOR3 u_IN;

	// Preliminary calculations
	PreliminaryCalculations(var);

	// Convert vector from body coordinates to instrument cordinates
	u_IN = mul(mul(var.N, MT[Inputs.Mount - 1].MAT), u_BY);
	InstrumentVectorToInstrumentAngles(var, u_IN, A1, A2);

	// Check limits
	if (A1 < Inputs.A1_MIN * RAD || A1 > Inputs.A1_MAX * RAD)
	{
		Limit1 = true;
	}
	else
	{
		Limit1 = false;
	}
	if (A2 < Inputs.A2_MIN * RAD || A2 > Inputs.A2_MAX * RAD)
	{
		Limit2 = true;
	}
	else
	{
		Limit2 = false;
	}
}

VECTOR3 InstrumentDefinitionTable::InstrumentAnglesToBodyVector(InstrumentMountMatrix* MT, double A1, double A2) const
{
	// INPUTS:
	// A1: Instrument angle 1, radians
	// A2: Instrument angle 2, radians
	// OUTPUTS:
	// u_BY: Unit direction vector in body coordinates

	InstrumentDefinitionTableVariable var;
	VECTOR3 u_IN, u_BY;

	// Preliminary calculations
	PreliminaryCalculations(var);

	u_IN = InstrumentAnglesToInstrumentCoordinates(var, A1, A2);
	u_BY = tmul(mul(var.N, MT[Inputs.Mount - 1].MAT), u_IN);

	return u_BY;
}

void InstrumentDefinitionTable::InstrumentVectorToInstrumentAngles(const InstrumentDefinitionTableVariable& var, VECTOR3 u_IN, double& A1, double& A2) const
{
	// INPUTS:
	// u_IN: Unit vector in instrument coordinate system
	// OUTPUTS:
	// A1: Instrument angle 1, radians (always 0-360°)
	// A2: Instrument angle 2, radians (either 0-180° or -90° to 90°)

	// TBD: Euler vs independent
	InstrumentVectorToInstrumentAnglesEuler(var, u_IN, A1, A2);
}

void InstrumentDefinitionTable::InstrumentVectorToInstrumentAnglesEuler(const InstrumentDefinitionTableVariable& var, VECTOR3 u_IN, double& A1, double& A2) const
{
	// Euler sequences
	if (var.I == var.L)
	{
		A1 = atan2(u_IN.data[var.K - 1] * (double)(var.S2 * var.SK), u_IN.data[var.J - 1] * (double)var.SJ);
		A2 = acos(u_IN.data[var.I - 1] * (double)var.SI);
	}
	else
	{
		A1 = atan2(u_IN.data[var.J - 1] * (double)var.SJ, u_IN.data[var.I - 1] * (double)var.SI);
		A2 = asin(-u_IN.data[var.K - 1] * (double)(var.S2 * var.SK));
	}
}

void InstrumentDefinitionTable::InstrumentVectorToInstrumentAnglesIndep(const InstrumentDefinitionTableVariable& var, VECTOR3 u_IN, double& A1, double& A2) const
{
	// Independent sequences
	A1 = atan2(u_IN.data[var.J - 1] * (double)var.SJ, u_IN.data[var.I - 1] * (double)var.SI);
	A2 = atan2(u_IN.data[var.K - 1] * (double)var.SK, u_IN.data[var.I - 1] * (double)var.SI);
}

VECTOR3 InstrumentDefinitionTable::InstrumentAnglesToInstrumentCoordinates(const InstrumentDefinitionTableVariable& var, double A1, double A2) const
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

	if (var.I == var.L)
	{
		u_IN.data[var.I - 1] = CA2 * (double)var.SI;
		u_IN.data[var.J - 1] = CA1 * SA2 * (double)var.SJ;
		u_IN.data[var.K - 1] = SA1 * SA2 * (double)(var.S2 * var.SK);
	}
	else
	{
		u_IN.data[var.I - 1] = CA1 * CA2 * (double)var.SI;
		u_IN.data[var.J - 1] = SA1 * CA2 * (double)var.SJ;
		u_IN.data[var.K - 1] = -SA2 * (double)(var.S2 * var.SK);
	}

	return u_IN;
}

VECTOR3 InstrumentDefinitionTable::InstrumentAnglesToInstrumentCoordinatesIndep(const InstrumentDefinitionTableVariable& var, double A1, double A2) const
{
	// Independent instrument rotation sequence

	// INPUTS:
	// A1: Instrument angle 1, radians
	// A2: Instrument angle 2, radians
	// OUTPUTS:
	// return value: unit pointing vector in instrument coordinates

	VECTOR3 u_IN;

	u_IN.data[var.I - 1] = (double)(var.SI);
	u_IN.data[var.J - 1] = tan(A1) * (double)(var.SJ);
	u_IN.data[var.K - 1] = tan(A2) * (double)(var.SK);

	return unit(u_IN);
}

bool InstrumentDefinitionTable::IsInitialized() const
{
	return Initialized;
}

InstrumentDefinitionTableEntry InstrumentDefinitionTable::GetInputs() const
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

void InstrumentDefinitionTable::PreliminaryCalculations(InstrumentDefinitionTableVariable& var) const
{
	int S, SL;

	var.N = mul(RotX(Inputs.Phi2 * RAD), mul(RotY(Inputs.Theta * RAD), RotX(Inputs.Phi1 * RAD)));

	var.I = abs(Inputs.e[2]);
	var.K = var.L = abs(Inputs.e[0]);
	var.SI = sign(Inputs.e[2]);
	var.SK = SL = sign(Inputs.e[0]);
	S = sign(Inputs.e[1]);
	if (var.I == var.L)
	{
		var.K = abs(Inputs.e[1]);
		var.SK = S;
	}
	var.J = 6 - var.I - var.K;
	var.SJ = var.SI * var.SK;
	if (var.I != (var.K + 1))
	{
		if (var.I != (var.K - 2))
		{
			var.SJ = -var.SJ;
		}
	}
	if (var.I == var.L)
	{
		var.S2 = SL * var.SI;
	}
	else
	{
		var.S2 = var.SJ * S;
	}
}