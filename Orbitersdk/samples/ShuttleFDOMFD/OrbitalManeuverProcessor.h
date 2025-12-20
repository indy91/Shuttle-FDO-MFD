/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2025 Niklas Beug

  Orbital Maneuver Processor (Header)

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

namespace OMP
{
	// CONSTANTS
	const unsigned MAXMANEUVERS = 40U;
	const unsigned MAXMANEUVERNAMELENGTH = 10U;
	const unsigned MAXSECONDARIES = 9U;

	class OMPDefs
	{
	public:
		typedef enum { NOMAN, APSO, CIRC, DVPY, DVYP, EXDV, HA, HASH, LSDV, NOSH, PC, NC, NCC, NH, NHRD, NPC, NS, NSR, SOI, SOM, SOR, TPF, TPI, TPM } MANTYPE;
		typedef enum { NOTHR, THRES_APS, THRES_CAN, THRES_DLT, THRES_DT, THRES_DTL, THRES_M, THRES_REV, THRES_T, THRES_N, THRES_WT } THRESHOLD;
		typedef enum {
			NOSEC, A, ALT, APO, SEC_APS, ARG, ASC, CN, DEC, DSC, EL, LAT, LON, N, NA, NP, OPT, P, PER, RAS, TGTA, TGTP, U,
			LITI, LITM, LITO, NITI, NITM, NITO, CXYZ, DH, DNOD, DPC, DR, DV, DVLS, DVLV, HD, ITSR, MREV, SEC_NULL, PHA, PIT, VFIL, WEDG, YAW
		} SECONDARIES;
		typedef enum { NOTHRU, PX4, PX3, PX2, MXL, YL, MYL, ZH, ZL, MZH, MZL, M1, M2, OL, OR, OBP } THRUSTERS;
		typedef enum { NOGUID, M50, P7 } GUID;
	};

	// CONVERSIONS
	// MANTYPE to string
	std::string GetOPMManeuverType(OMPDefs::MANTYPE type);
	// String to MANTYPE
	OMPDefs::MANTYPE GetOPMManeuverType(std::string buf);
	// THRESHOLD to string
	std::string GetOPMManeuverThreshold(OMP::OMPDefs::THRESHOLD type);
	// String to THRESHOLD
	OMP::OMPDefs::THRESHOLD GetOPMThresholdType(std::string buf);
	// SECONDARIES to string
	std::string GetSecondaryName(OMP::OMPDefs::SECONDARIES sec);
	// String to SECONDARIES
	OMP::OMPDefs::SECONDARIES GetSecondaryType(std::string buf);

	struct SecData
	{
		OMPDefs::SECONDARIES type = OMPDefs::SECONDARIES::NOSEC;
		double value = 0.0;
	};

	struct ManeuverConstraintsInput
	{
		std::string Name;			//Maneuver name
		std::string Type;			//Maneuver type
		std::string Threshold;		//Threshold type
		std::string ThresholdValue; //Value associated with threshold
		std::string Secondary[9];
		std::string SecondaryValues[9];
	};

	struct ManeuverConstraints
	{
		std::string name;
		OMPDefs::MANTYPE type;
		OMPDefs::THRESHOLD threshold;
		double thresh_num = 0.0;	//time, delta time or revs
		std::vector<SecData> secondaries;
	};

	struct ManeuverConstraintsTableHeader
	{
		ManeuverConstraintsTableHeader();

		std::string Name;		// Name of Maneuver Constraints Table
		std::string Comment;	// Comment about MCT
	};

	// Table in string format
	struct ManeuverConstraintsTableInput
	{
		ManeuverConstraintsTableHeader Header;
		std::vector<ManeuverConstraintsInput> Table;
	};

	// Table in internal format
	struct ManeuverConstraintsTable
	{
		ManeuverConstraintsTableHeader Header;
		std::vector<ManeuverConstraints> Table;
	};

	struct ITERCONSTR
	{
		int type = 0;			//type of iterator (1 = NC, 2 = NH, 3 = NPC)
		unsigned man = 0;		//maneuver that is applying the DV
		unsigned constr = 0;	//maneuver for which the constraint is applied
		int constrtype = 0;		//For NC: 1 = DR, 2 = PHA
		double value = 0.0;		//Value of the constraint
	};

	struct MANEUVER
	{
		std::string name;
		double TIG_GMT;
		VECTOR3 dV_LVLH;
		OMPDefs::MANTYPE type;
	};

	struct MANEVALDATA
	{
		MANEVALDATA();

		//Display data
		std::string type;
		std::string name;
		double DVMag;
		double GMTIG;
		double METIG;
		double DT;
		VECTOR3 DV;
		double HA;
		double HP;
		double DH;
		double RANGE;
		double PHASE;
		bool noon;
		double TTN;
		double Y;
		double Ydot;
		bool sunrise;
		double TTS;

		//Additional data
		bool ChaserMan; //true = chaser, false = target
		OrbMech::SV sv_before;
		VECTOR3 V_after;
	};

	struct MANEVALTABLE
	{
		MANEVALTABLE();

		// Header

		// Name of Maneuver Evaluation Table
		std::string Name;		
		// Comment about MET
		std::string Comment;
		//Chaser state vector GMT
		double GMT_C;
		//Targer state vector GMT
		double GMT_T;
		//Total accumulated chaser DV
		VECTOR3 dv_C;
		//Total accumulated chaser DV
		VECTOR3 dv_T;

		std::vector<MANEVALDATA> Maneuvers;
	};

	struct OMPInputs
	{
		//Input state vectors
		OrbMech::SV CHASER;
		OrbMech::SV TARGET;
		//Gravity setting
		bool useNonSphericalGravity;
		//Table with maneuver constraints
		ManeuverConstraintsTable MCT;
		//Data for the evaluation table
		std::string OMPChaserFile, OMPTargetFile, OMPMCTFile;
		//Write output print
		bool PRINT = true;
		//Write output plot
		bool PLOT = false;
	};

	struct OMPOutputs
	{
		MANEVALTABLE ManeuverEvaluationTable;
		int Error = 0;
		std::string ErrorMessage;
		std::vector<std::vector<std::string>> OutputPrint;
	};

	struct OMPVariablesTable
	{
		OMPVariablesTable();

		double ISP[5];
		double RangeTolerance;
	};

	class OrbitalManeuverProcessor
	{
	public:
		OrbitalManeuverProcessor(OrbMech::SessionConstants& scnst);
		//Parse input to internal format
		bool ParseManeuverConstraintsTable(const std::vector< ManeuverConstraintsInput>& tab_in, std::vector < ManeuverConstraints>& tab_out, std::string& errormessage) const;
		//Run OMP
		void Calculate(const OMPInputs& in, OMPOutputs& out);
	protected:
		int CalculateOMPPlan(const OMPInputs& in);
		void Init(const OMPInputs& in);
		int ProcessIterators();
		int ProcessManeuverConstraints();
		int ProcessTIGModifiers();
		int UpdateToThreshold();
		int UpdateToModfiedTIG();
		int RunIterators();
		int ApplyManeuver();
		bool IsOMPConverged() const;
		void CalculateManeuverEvalTable(OrbMech::SV sv_A0, OrbMech::SV sv_P0);
		void PrintManeuverEvaluationTable();
		void GetOMPError(int err, std::string& buf, unsigned int i = 0, unsigned int j = 0) const;

		// TRAJECTORY PROPAGATION
		
		// Propagate through DT
		int coast_auto(OrbMech::SV sv0, double dt, OrbMech::SV& sv1) const;
		// Propagate through M orbits
		int DeltaOrbitsAuto(OrbMech::SV sv0, double M, OrbMech::SV& sv1) const;
		// Interface to function for propagation to desired time, mean anomaly, argument of latitude or maneuver line
		int GeneralTrajectoryPropagation(OrbMech::SV sv0, int opt, double param, double DN, OrbMech::SV& sv1) const;
		// Propagate to upcoming apoapsis
		int timetoapo_auto(OrbMech::SV sv_A, double revs, OrbMech::SV& sv_out) const;
		// Propagate to upcoming periapsis
		int timetoperi_auto(OrbMech::SV sv_A, double revs, OrbMech::SV& sv_out) const;
		// Find time to common node
		int FindCommonNode(OrbMech::SV sv_A, OrbMech::SV sv_P, VECTOR3& u_d, double& dt) const;
		// Propagate to Nth apsidal crossing
		int FindNthApsidalCrossingAuto(OrbMech::SV sv0, double N, OrbMech::SV& sv_out) const;
		// Propagate to optimum node shift position
		int FindOptimumNodeShiftPoint(OrbMech::SV sv0, double dh, OrbMech::SV& sv_out) const;
		//Computation of the travel angle to a desired latitude
		bool TLAT(VECTOR3 R, VECTOR3 V, double lat, int C, double& K_AD, double& dtheta) const;
		//Computation of the travel angle to a desired longitude
		double TLON(VECTOR3 R, VECTOR3 V, double t, double lng, int C, double w_E) const;
		//Computation of the travel angle to a desired altitude
		bool TALT(VECTOR3 R, VECTOR3 V, double rad, int C, double mu, double& dtheta) const;
		//Search for maneuver time routine
		int SEARMT(OrbMech::SV sv0, int opt, double val, OrbMech::SV& sv1) const;
		int PositionMatch(OrbMech::SV sv_A, OrbMech::SV sv_P, OrbMech::SV& sv_P2) const;

		//TIG modifiers
		int Sunrise(OrbMech::SV sv0, bool rise, bool midnight, OrbMech::SV& sv1) const;
		int FindOrbitalSunriseRelativeTime(OrbMech::SV sv0, bool sunrise, double dt1, OrbMech::SV& sv_out);
		int FindOrbitalMidnightRelativeTime(OrbMech::SV sv0, bool midnight, double dt1, OrbMech::SV& sv_out);

		//Maneuvers
		bool HeightManeuverAuto(OrbMech::SV sv_A, double r_D, bool horizontal, VECTOR3& DV, double dv_guess = 0.0);
		int Lambert(OrbMech::SV sv_A1, VECTOR3 RP2_off, double dt, VECTOR3& V_A1_apo);
		int SOIManeuver(OrbMech::SV sv_A1, OrbMech::SV sv_P, double dt, VECTOR3 off, VECTOR3& DV);
		int SORManeuver(OrbMech::SV sv_A1, OrbMech::SV sv_P, VECTOR3 off, VECTOR3& DV);
		VECTOR3 NPCManeuver(OrbMech::SV sv_A, VECTOR3 H_P) const;
		double CalculateYDot(VECTOR3 V_A, VECTOR3 R_P, VECTOR3 V_P) const;
		VECTOR3 NSRManeuver(OrbMech::SV sv_A, OrbMech::SV sv_P) const;
		VECTOR3 NodeShiftManeuver(OrbMech::SV sv0, double dh_D) const;
		VECTOR3 PlaneChangeManeuver(OrbMech::SV sv0, double dw_D) const;

		//Utilities
		double GMTfromGET(double get) const;
		double GETfromGMT(double gmt) const;
		OrbMech::SV ApplyLVLHManeuver(OrbMech::SV sv0, VECTOR3 DV_LVLH, int ThrustProfile, bool forwards = true) const;
		int GetOMPThresholdValue(std::string buf, OMP::OMPDefs::THRESHOLD type, double& val) const;

		// INPUTS
		OrbMech::SV sv_chaser;
		OrbMech::SV sv_target;
		ManeuverConstraintsTable MCT;
		std::string ProjectFolder, OMPChaserFile, OMPTargetFile, OMPMCTFile;

		// INTERNAL

		struct OMPManeuverArray
		{
			OMPManeuverArray()
			{
				ChaserMan = true;
				ThrustProfile = 0;
				dv_table = add_constraint = _V(0, 0, 0);
			}

			//Active vehicle for the maneuver
			bool ChaserMan;
			//Thrust profile (0-4) for the maneuver
			int ThrustProfile;
			//Chaser state vector at threshold time
			OrbMech::SV sv_A_threshold;
			//Target state vector at threshold time
			//Chaser state vector at TIG, before maneuver
			OrbMech::SV sv_A_bef_table;
			//Chaser state vector at TIG, after maneuver
			OrbMech::SV sv_A_aft_table;
			//Target state vector at TIG, before maneuver
			OrbMech::SV sv_P_bef_table;
			//Target state vector at TIG, after maneuver
			OrbMech::SV sv_P_aft_table;
			//LVLH Delta V of maneuver
			VECTOR3 dv_table;
			//The secondary constraint for TIG
			SecData tigmodifiers;
			//Additional constraints for the maneuver
			VECTOR3 add_constraint;
		};
		//Error number
		int Error;
		//Gravity setting
		bool useNonSphericalGravity;
		//Current maneuver counter
		unsigned int CurMan;
		//Number of maneuvers
		size_t TAB;
		//Vector of iterators (NC, NH, NPC)
		std::vector<ITERCONSTR> iterators;
		OrbMech::SV sv_cur, sv_P_cur;
		std::vector<OMPManeuverArray> ManeuverData;

		//Phantom state vector for the NPC maneuver
		OrbMech::SV sv_phantom;
		//Vector of iterations
		std::vector<OrbMech::ITERSTATE> iterstate;
		//Flag set by RunIterators if the iteration has to step back to an earlier maneuver
		bool recycle;
		//Table of maneuver definitions for generating the evaluation table
		std::vector<MANEUVER> ManeuverTable;
		std::vector<std::vector<std::string>> OutputPrint;

		// OUTPUTS
		MANEVALTABLE ManeuverEvaluationTable;

		// CONSTANTS
		OrbMech::SessionConstants& sesconst;
		OMPVariablesTable ompvariables;
	};
}