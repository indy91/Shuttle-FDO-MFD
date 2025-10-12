/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2025 Niklas Beug

  Orbital Maneuver Processor

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

#include "OrbitalManeuverProcessor.h"
#include <string>

namespace OMP
{
	MANEVALDATA::MANEVALDATA()
	{
		DVMag = 0.0;
		GMTIG = 0.0;
		METIG = 0.0;
		DT = 0.0;
		DV = _V(0, 0, 0);
		HA = 0.0;
		HP = 0.0;
		DH = 0.0;
		RANGE = 0.0;
		PHASE = 0.0;
		noon = false;
		TTN = 0.0;
		Y = 0.0;
		Ydot = 0.0;
		sunrise = false;
		TTS = 0.0;

		ChaserMan = false;
		V_after = _V(0, 0, 0);
	}

	MANEVALTABLE::MANEVALTABLE()
	{
		GMT_C = 0.0;
		GMT_T = 0.0;
		dv_C = _V(0, 0, 0);
		dv_T = _V(0, 0, 0);
	}

	OMPVariablesTable::OMPVariablesTable()
	{
		for (int i = 0; i < 5; i++)
		{
			ISP[i] = 3000.0;
		}
		RangeTolerance = 10.0;
	}

	OrbitalManeuverProcessor::OrbitalManeuverProcessor(OrbMech::SessionConstants& scnst) : sesconst(scnst)
	{
		Error = 0;
		TAB = 0U;
		CurMan = 0;
		recycle = false;
		useNonSphericalGravity = false;
	}

	void OrbitalManeuverProcessor::Init(const OMPInputs& in)
	{
		sv_chaser = in.CHASER;
		sv_target = in.TARGET;
		useNonSphericalGravity = in.useNonSphericalGravity;
		ManeuverConstraintsTable = in.ManeuverConstraintsTable;
		OMPChaserFile = in.OMPChaserFile;
		OMPTargetFile = in.OMPTargetFile;
		OMPMCTFile = in.OMPMCTFile;

		Error = 0;
		recycle = false;

		iterators.clear();
		iterstate.clear();

		TAB = ManeuverConstraintsTable.size();

		ManeuverData.clear();
		ManeuverData.resize(TAB);

		OutputPrint.clear();
	}

	bool OrbitalManeuverProcessor::ParseManeuverConstraintsTable(const std::vector<ManeuverConstraintsInput>& tab_in, std::vector <ManeuverConstraints>& tab_out, std::string& errormessage) const
	{
		//Convert pure text to internal type definitions

		ManeuverConstraints temp;
		SecData sectemp;
		unsigned int i, j;

		tab_out.clear();

		i = j = 0;

		for (i = 0; i < tab_in.size(); i++)
		{
			temp.secondaries.clear();

			//Name
			if (tab_in[i].Name.size() > 10)
			{
				GetOMPError(2001, errormessage, i); //Error 2001: Error parsing MCT, maneuver name too long
				return true;
			}
			temp.name = tab_in[i].Name;

			//Type
			temp.type = GetOPMManeuverType(tab_in[i].Type.c_str());
			if (temp.type == OMPDefs::MANTYPE::NOMAN)
			{
				GetOMPError(2002, errormessage, i); //Error 2002: Error parsing MCT, maneuver type illegal
				return true;
			}

			//Threshold
			temp.threshold = GetOPMThresholdType(tab_in[i].Threshold);
			if (temp.threshold == OMPDefs::THRESHOLD::NOTHR)
			{
				GetOMPError(2003, errormessage, i); //Error 2003: Error parsing MCT, threshold type illegal
				return true;
			}

			//Threshold Value
			if (GetOMPThresholdValue(tab_in[i].ThresholdValue, temp.threshold, temp.thresh_num))
			{
				GetOMPError(2004, errormessage, i); //Error 2004: Error parsing MCT, threshold value illegal
				return true;
			}

			//Secondaries
			for (j = 0; j < 9; j++)
			{
				//Empty?
				if (tab_in[i].Secondary[j] == "") continue;
				//Check if it exists
				sectemp.type = GetSecondaryType(tab_in[i].Secondary[j]);
				if (sectemp.type == OMPDefs::SECONDARIES::NOSEC)
				{
					GetOMPError(2005, errormessage, i); //Error 2005: Error parsing MCT, secondary type illegal
					return true;
				}
				if (sscanf_s(tab_in[i].SecondaryValues[j].c_str(), "%lf", &sectemp.value) != 1)
				{
					GetOMPError(2006, errormessage, i); //Error 2006: Error parsing MCT, secondary value illegal
					return true;
				}
				temp.secondaries.push_back(sectemp);
			}
			tab_out.push_back(temp);
		}
		return false;
	}

	void OrbitalManeuverProcessor::Calculate(const OMPInputs& in, OMPOutputs& out)
	{
		Error = CalculateOMPPlan(in);

		std::string Buffer;
		GetOMPError(Error, Buffer);

		out.ManeuverEvaluationTable = ManeuverEvaluationTable;
		out.Error = Error;
		out.ErrorMessage = Buffer;
		out.OutputPrint = OutputPrint;
	}

	int OrbitalManeuverProcessor::CalculateOMPPlan(const OMPInputs& in)
	{
		Init(in);

		//Initial error checks
		if (sv_target.GMT == 0.0) return 100; //Error 100: Target not defined
		if (ManeuverConstraintsTable.size() < 1) return 1;	//Error 1: No maneuvers in constraint table
		if (ManeuverConstraintsTable[0].threshold != OMPDefs::THRESHOLD::THRES_T) return 2;	//Error 2: First maneuver needs a T as threshold

		//SET UP ITERATORS and CHECK THAT THRESHOLDS EXIST
		Error = ProcessIterators();
		if (Error) return Error;

		iterstate.resize(iterators.size());

		//CHECK THAT MANEUVER CONSTRAINTS EXIST
		Error = ProcessManeuverConstraints();
		if (Error) return Error;

		//CHECK AND LOAD TIG MODIFIERS
		Error = ProcessTIGModifiers();
		if (Error) return Error;

		//Set up loop
		CurMan = 0;

		do
		{
			recycle = false;
			if (CurMan == 0)
			{
				sv_cur = sv_chaser;
				sv_P_cur = sv_target;
			}
			else
			{
				sv_cur = ManeuverData[CurMan - 1].sv_A_aft_table;
				sv_P_cur = ManeuverData[CurMan - 1].sv_P_aft_table;
			}

			//THRESHOLD (sv_cur ->sv_A_bef_table)
			Error = UpdateToThreshold();
			if (Error) return Error;

			//Save state vector at threshold
			ManeuverData[CurMan].sv_A_threshold = ManeuverData[CurMan].sv_A_bef_table;
			//Take target to threshold
			Error = coast_auto(sv_P_cur, ManeuverData[CurMan].sv_A_bef_table.GMT - sv_P_cur.GMT, ManeuverData[CurMan].sv_P_bef_table);
			if (Error) return Error;

			//TIG MODIFICATION
			Error = UpdateToModfiedTIG();
			if (Error) return Error;

			//TIG has been calculated, now get target SV at TIG
			Error = coast_auto(sv_P_cur, ManeuverData[CurMan].sv_A_bef_table.GMT - sv_P_cur.GMT, ManeuverData[CurMan].sv_P_bef_table);
			//Fix this:
			sv_P_cur = ManeuverData[CurMan].sv_P_bef_table;
			if (Error) return Error;

			//ITERATORS
			Error = RunIterators();
			if (Error) return Error;

			//Recycle due to iterator not converged?
			if (recycle) continue;

			//MANEUVER
			Error = ApplyManeuver();
			if (Error) return Error;

			CurMan++;
			if (CurMan == TAB && IsOMPConverged() == false)
			{
				CurMan = 0;
			}
		} while (CurMan < TAB || IsOMPConverged() == false);

		MANEUVER man;

		ManeuverTable.clear();

		for (unsigned int i = 0; i < TAB; i++)
		{
			man.dV_LVLH = ManeuverData[i].dv_table;
			man.name = ManeuverConstraintsTable[i].name;
			man.TIG_GMT = ManeuverData[i].sv_A_bef_table.GMT;
			man.type = ManeuverConstraintsTable[i].type;

			ManeuverTable.push_back(man);
		}

		CalculateManeuverEvalTable(sv_chaser, sv_target);

		if (in.PRINT) PrintManeuverEvaluationTable();

		return 0;
	}

	int OrbitalManeuverProcessor::ProcessIterators()
	{
		ITERCONSTR con;
		unsigned int i, j, k;
		int found;
		bool npcflag = false;

		for (i = 0; i < TAB; i++)
		{
			found = 0;

			if (ManeuverConstraintsTable[i].threshold == OMPDefs::THRESHOLD::NOTHR) return 5;	//Error 5: Maneuver doesn't have a threshold

			if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NC)
			{
				for (k = i + 1; k < ManeuverConstraintsTable.size(); k++)
				{
					for (j = 0; j < ManeuverConstraintsTable[k].secondaries.size(); j++)
					{
						if (ManeuverConstraintsTable[k].secondaries[j].type == OMPDefs::DR)
						{
							found = 1;
						}
						else if (ManeuverConstraintsTable[k].secondaries[j].type == OMPDefs::PHA)
						{
							found = 2;
						}
						if (found) break;
					}
					if (found) break;
				}

				if (!found) return 7;	//Error 7: didn't find NC constraint

				con.man = i;
				con.type = 1;
				con.constr = k;
				con.constrtype = found;
				if (found == 1)
				{
					//DR
					con.value = ManeuverConstraintsTable[k].secondaries[j].value * 1852.0;
				}
				else
				{
					//PHA or ELA
					con.value = ManeuverConstraintsTable[k].secondaries[j].value * RAD;
				}

				iterators.push_back(con);
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NH || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NHRD)
			{
				for (k = i + 1; k < ManeuverConstraintsTable.size(); k++)
				{
					for (j = 0; j < ManeuverConstraintsTable[k].secondaries.size(); j++)
					{
						if (ManeuverConstraintsTable[k].secondaries[j].type == OMPDefs::DH)
						{
							found = 1;
						}
						if (found) break;
					}
					if (found) break;
				}

				if (!found) return 8;	//Error 8: didn't find NH constraint

				con.man = i;
				con.type = 2;
				con.constr = k;
				con.value = ManeuverConstraintsTable[k].secondaries[j].value * 1852.0;

				iterators.push_back(con);
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NPC)
			{
				for (k = i + 1; k < ManeuverConstraintsTable.size(); k++)
				{
					for (j = 0; j < ManeuverConstraintsTable[k].secondaries.size(); j++)
					{
						if (ManeuverConstraintsTable[k].secondaries[j].type == OMPDefs::WEDG)
						{
							found = 1;
						}
						if (found) break;
					}
					if (found) break;
				}

				if (npcflag) return 22;	//More than one NPC maneuver found

				//Only set up iterator if constraint was found
				if (found)
				{
					npcflag = true;

					con.man = i;
					con.type = 3;
					con.constr = k;
					con.value = ManeuverConstraintsTable[k].secondaries[j].value * 1852.0;
					iterators.push_back(con);
				}
			}
		}

		return 0;
	}

	int OrbitalManeuverProcessor::ProcessManeuverConstraints()
	{
		unsigned int i, j;
		int found;

		for (i = 0; i < TAB; i++)
		{
			found = 0;
			if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HA || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::HASH)
			{
				for (j = 0; j < ManeuverConstraintsTable[i].secondaries.size(); j++)
				{
					if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::HD)
					{
						ManeuverData[i].add_constraint.x = ManeuverConstraintsTable[i].secondaries[j].value * 1852.0;
						found++;
					}
				}
				if (found != 1) return 6;	//Didn't find HD constraints for HA maneuver
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::EXDV)
			{
				for (j = 0; j < ManeuverConstraintsTable[i].secondaries.size(); j++)
				{
					if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::DVLV)
					{
						if (found > 2) return 4;													//Error 4: too many DV components specified
						ManeuverData[i].dv_table.data[found] = ManeuverConstraintsTable[i].secondaries[j].value * 0.3048;
						found++;
					}
				}
				if (found < 3) return 3;													//Error 3: Not enough DV components specified
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOI)
			{
				if (i + 1 >= TAB) return 11; //Error 11: No maneuver after SOI/NCC
				if (ManeuverConstraintsTable[i + 1].type != OMPDefs::MANTYPE::SOR) return 12;	//Error 12: Wrong maneuver after SOI
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::SOR)
			{
				for (j = 0; j < ManeuverConstraintsTable[i].secondaries.size(); j++)
				{
					if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::CXYZ)
					{
						if (found > 2) return 9; //Error 9: too many CXYZ components specified
						ManeuverData[i].add_constraint.data[found] = ManeuverConstraintsTable[i].secondaries[j].value * 1852.0;
						found++;
					}
				}
				if (found < 3) return 10; //Error 10: Not enough CXYZ components specified
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::TPI)
			{
				if (i + 1 >= TAB) return 11; //Error 11: No maneuver after SOI/NCC/TPI
				if (ManeuverConstraintsTable[i + 1].type != OMPDefs::MANTYPE::TPF) return 12;	//Error 12: Wrong maneuver after SOI/TPI
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NCC)
			{
				if (i + 1 >= TAB) return 11; //Error 11: No maneuver after SOI/NCC

				for (j = 0; j < ManeuverConstraintsTable[i + 1].secondaries.size(); j++)
				{
					if (ManeuverConstraintsTable[i + 1].secondaries[j].type == OMPDefs::CXYZ)
					{
						if (found > 2) return 9; //Error 9: too many CXYZ components specified
						ManeuverData[i + 1].add_constraint.data[found] = ManeuverConstraintsTable[i + 1].secondaries[j].value * 1852.0;
						found++;
					}
					else if (ManeuverConstraintsTable[i + 1].secondaries[j].type == OMPDefs::DH)
					{
						if (found > 2) return 9; //Error 9: too many CXYZ components specified
						ManeuverData[i + 1].add_constraint.z = ManeuverConstraintsTable[i + 1].secondaries[j].value * 1852.0;
						found++;
					}
					else if (ManeuverConstraintsTable[i + 1].secondaries[j].type == OMPDefs::DR)
					{
						if (found > 2) return 9; //Error 9: too many CXYZ components specified
						ManeuverData[i + 1].add_constraint.x = ManeuverConstraintsTable[i + 1].secondaries[j].value * 1852.0;
						found++;
					}
					else if (ManeuverConstraintsTable[i + 1].secondaries[j].type == OMPDefs::WEDG)
					{
						if (found > 2) return 9; //Error 9: too many CXYZ components specified
						ManeuverData[i + 1].add_constraint.y = ManeuverConstraintsTable[i + 1].secondaries[j].value * 1852.0;
						found++;
					}
				}
				if (found < 3) return 10; //Error 10: Not enough CXYZ components specified
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::DVPY || ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::DVYP)
			{
				for (j = 0; j < ManeuverConstraintsTable[i].secondaries.size(); j++)
				{
					if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::DV)
					{
						ManeuverData[i].add_constraint.x = ManeuverConstraintsTable[i].secondaries[j].value * 0.3048;
					}
					else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::PIT)
					{
						ManeuverData[i].add_constraint.y = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
					}
					else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::YAW)
					{
						ManeuverData[i].add_constraint.z = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
					}
				}
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::NOSH)
			{
				for (j = 0; j < ManeuverConstraintsTable[i].secondaries.size(); j++)
				{
					if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::DNOD)
					{
						ManeuverData[i].add_constraint.x = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
						found++;
					}
				}
				if (found != 1) return 24;	//Didn't find DNOD constraint for NOSH maneuver
			}
			else if (ManeuverConstraintsTable[i].type == OMPDefs::MANTYPE::PC)
			{
				for (j = 0; j < ManeuverConstraintsTable[i].secondaries.size(); j++)
				{
					if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::DPC)
					{
						ManeuverData[i].add_constraint.x = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
						found++;
					}
				}
				if (found != 1) return 27;	//Didn't find DPC constraint for PC maneuver
			}
		}

		return 0;
	}

	int OrbitalManeuverProcessor::ProcessTIGModifiers()
	{
		unsigned int i, j;

		for (i = 0; i < TAB; i++)
		{
			for (j = 0; j < ManeuverConstraintsTable[i].secondaries.size(); j++)
			{
				//Maneuver at apogee
				if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::APO)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::APO;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				//Maneuver at perigee
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::PER)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::PER;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				//Maneuver at target apogee
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::TGTA)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::TGTA;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				//Maneuver at target perigee
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::TGTP)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::TGTP;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				//Initial guess
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::DV)
				{
					ManeuverData[i].dv_table = _V(ManeuverConstraintsTable[i].secondaries[j].value * 0.3048, 0, 0);
				}
				//Common Node
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::CN)
				{
					if (ManeuverConstraintsTable[i].type != OMPDefs::MANTYPE::NPC) return 14;	//Error 14: CN secondary only applies to NPC
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::CN;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				//Maneuver at nth upcoming apsis
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::SEC_APS)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::SEC_APS;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::LITI)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::LITI;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * 60.0;
				}
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::LITM)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::LITM;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * 60.0;
				}
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::LITO)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::LITO;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * 60.0;
				}
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::NITI)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::NITI;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * 60.0;
				}
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::NITM)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::NITM;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * 60.0;
				}
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::NITO)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::NITO;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * 60.0;
				}
				//Optimum node shift
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::OPT)
				{
					if (ManeuverConstraintsTable[i].type != OMPDefs::MANTYPE::NOSH) return 25;	//Error 25: OPT secondary only applies to NOSH
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::OPT;
				}
				//Angle from apogee
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::A)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::A;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
				}
				//Angle from perigee
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::P)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::P;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
				}
				//Argument of latitude
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::U)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::U;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
				}
				//Ascending Node
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::ASC)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::ASC;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				//Descending Node
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::DSC)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::DSC;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value;
				}
				//Latitude
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::LAT)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::LAT;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
				}
				//Longitude
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::LON)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::LON;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
				}
				//Declination
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::DEC)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::DEC;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * RAD;
				}
				//Latitude
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::ALT)
				{
					ManeuverData[i].tigmodifiers.type = OMPDefs::SECONDARIES::ALT;
					ManeuverData[i].tigmodifiers.value = ManeuverConstraintsTable[i].secondaries[j].value * 1852.0;
				}
				//Maneuver vehicle and thruster
				else if (ManeuverConstraintsTable[i].secondaries[j].type == OMPDefs::SECONDARIES::VFIL)
				{
					//Only allows target maneuver for EXDV for now
					int Veh = (int)(ManeuverConstraintsTable[i].secondaries[j].value) / 10;
					int Thr = (int)(ManeuverConstraintsTable[i].secondaries[j].value) - Veh * 10;

					if (Veh < 1 || Veh > 2) return 32; //Error 32: Wrong vehicle code in VFIL secondary
					if (Thr < 1 || Thr > 5) return 33; //Error 33: Wrong thruster code in VFIL secondary

					if (Veh == 1)
					{
						ManeuverData[i].ChaserMan = true;
					}
					else
					{
						ManeuverData[i].ChaserMan = false;
					}
					ManeuverData[i].ThrustProfile = Thr - 1;
				}
			}
		}
		return 0;
	}

	int OrbitalManeuverProcessor::UpdateToThreshold()
	{
		if (ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_T)
		{
			double dt;

			dt = GMTfromGET(ManeuverConstraintsTable[CurMan].thresh_num) - sv_cur.GMT;
			Error = coast_auto(sv_cur, dt, ManeuverData[CurMan].sv_A_bef_table);
		}
		else if (ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_M || ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_REV ||
			ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_APS || ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_N)
		{
			double mult;

			if (ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_M || ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_REV)
			{
				mult = 1.0;
			}
			else
			{
				mult = 0.5;
			}

			if (CurMan > 0 && ManeuverConstraintsTable[CurMan - 1].type == OMPDefs::MANTYPE::NPC)
			{
				//Special NPC logic. Propagate threshold state vector for desired orbits, then propagate current state vector to same time. Assumes plane change doesn't affect orbital period
				Error = DeltaOrbitsAuto(ManeuverData[CurMan - 1].sv_A_threshold, mult * ManeuverConstraintsTable[CurMan].thresh_num, ManeuverData[CurMan].sv_A_bef_table);
				if (Error) return Error;
				Error = coast_auto(sv_cur, ManeuverData[CurMan].sv_A_bef_table.GMT - sv_cur.GMT, ManeuverData[CurMan].sv_A_bef_table);
			}
			else
			{
				Error = DeltaOrbitsAuto(sv_cur, mult * ManeuverConstraintsTable[CurMan].thresh_num, ManeuverData[CurMan].sv_A_bef_table);
			}
		}
		else if (ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_DT)
		{
			//Special NPC logic. DT from threshold time, not current time
			double ddt;
			if (CurMan > 0 && ManeuverConstraintsTable[CurMan - 1].type == OMPDefs::MANTYPE::NPC)
			{
				ddt = ManeuverConstraintsTable[CurMan].thresh_num - (sv_cur.GMT - ManeuverData[CurMan - 1].sv_A_threshold.GMT);
			}
			else
			{
				ddt = ManeuverConstraintsTable[CurMan].thresh_num;
			}
			Error = coast_auto(sv_cur, ddt, ManeuverData[CurMan].sv_A_bef_table);
		}
		else if (ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_CAN || ManeuverConstraintsTable[CurMan].threshold == OMPDefs::THRESHOLD::THRES_WT)
		{
			Error = DeltaOrbitsAuto(sv_cur, ManeuverConstraintsTable[CurMan].thresh_num / PI2, ManeuverData[CurMan].sv_A_bef_table);
		}
		if (Error) return Error;
		return 0;
	}

	int OrbitalManeuverProcessor::UpdateToModfiedTIG()
	{
		if (ManeuverData[CurMan].tigmodifiers.type != OMPDefs::SECONDARIES::NOSEC)
		{
			if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::APO)
			{
				Error = timetoapo_auto(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].tigmodifiers.value, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::PER)
			{
				Error = timetoperi_auto(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].tigmodifiers.value, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::TGTA)
			{
				//Take target to the apogee
				OrbMech::SV svtemp;
				Error = timetoapo_auto(ManeuverData[CurMan].sv_P_bef_table, ManeuverData[CurMan].tigmodifiers.value, svtemp);
				if (Error) return Error;
				//Then take chaser to phase match
				Error = PositionMatch(svtemp, ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::TGTP)
			{
				//Take target to the perigee
				OrbMech::SV svtemp;
				Error = timetoperi_auto(ManeuverData[CurMan].sv_P_bef_table, ManeuverData[CurMan].tigmodifiers.value, svtemp);
				if (Error) return Error;
				//Then take chaser to phase match
				Error = PositionMatch(svtemp, ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::CN)
			{
				double dt;

				//First iteration
				if (sv_phantom.GMT == 0.0)
				{
					OrbMech::SV sv_P1;

					Error = coast_auto(sv_P_cur, ManeuverData[CurMan].sv_A_bef_table.GMT - sv_P_cur.GMT, sv_P1);
					if (Error) return Error;

					Error = FindCommonNode(ManeuverData[CurMan].sv_A_bef_table, sv_P1, ManeuverData[CurMan].add_constraint, dt);
				}
				else
				{
					Error = FindCommonNode(ManeuverData[CurMan].sv_A_bef_table, sv_phantom, ManeuverData[CurMan].add_constraint, dt);
				}
				if (Error) return 31;

				Error = coast_auto(ManeuverData[CurMan].sv_A_bef_table, dt, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::SEC_APS)
			{
				Error = FindNthApsidalCrossingAuto(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].tigmodifiers.value, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::ARG)
			{
				OrbMech::OELEMENTS coe;
				double u_b, DN, u_d;

				coe = OrbMech::coe_from_sv(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V, OrbMech::mu_Earth);
				u_b = fmod(coe.TA + coe.w, PI2);
				u_d = ManeuverData[CurMan].tigmodifiers.value * RAD;
				if (u_b > u_d)
				{
					DN = 1.0;
				}
				else
				{
					DN = 0.0;
				}

				Error = GeneralTrajectoryPropagation(ManeuverData[CurMan].sv_A_bef_table, 2, u_d, DN, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::LITI || ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::NITO)
			{
				Error = FindOrbitalSunriseRelativeTime(ManeuverData[CurMan].sv_A_bef_table, true, ManeuverData[CurMan].tigmodifiers.value, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::LITM)
			{
				Error = FindOrbitalMidnightRelativeTime(ManeuverData[CurMan].sv_A_bef_table, false, ManeuverData[CurMan].tigmodifiers.value, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::LITO || ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::NITI)
			{
				Error = FindOrbitalSunriseRelativeTime(ManeuverData[CurMan].sv_A_bef_table, false, ManeuverData[CurMan].tigmodifiers.value, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::NITM)
			{
				Error = FindOrbitalMidnightRelativeTime(ManeuverData[CurMan].sv_A_bef_table, true, ManeuverData[CurMan].tigmodifiers.value, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::OPT)
			{
				Error = FindOptimumNodeShiftPoint(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].add_constraint.x, ManeuverData[CurMan].sv_A_bef_table);
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::LAT || ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::LON || ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::DEC || ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::ALT)
			{
				OrbMech::SV sv_temp;

				Error = SEARMT(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].tigmodifiers.type, ManeuverData[CurMan].tigmodifiers.value, sv_temp);
				if (Error) return 26; //Failure to find maneuver time
				ManeuverData[CurMan].sv_A_bef_table = sv_temp;
			}
			else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::ASC || ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::DSC || ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::U)
			{
				double U, crossings, U_D;

				//Calculate current argument of latitude
				U = OrbMech::ArgLat(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V);

				if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::ASC)
				{
					U_D = 0.0;
					crossings = round(ManeuverData[CurMan].tigmodifiers.value) - 1.0;
				}
				else if (ManeuverData[CurMan].tigmodifiers.type == OMPDefs::SECONDARIES::DSC)
				{
					U_D = PI;
					crossings = round(ManeuverData[CurMan].tigmodifiers.value) - 1.0;
				}
				else
				{
					U_D = ManeuverData[CurMan].tigmodifiers.value;
					crossings = 0.0;
				}

				if (U > U_D)
				{
					crossings = crossings + 1.0;
				}

				Error = GeneralTrajectoryPropagation(ManeuverData[CurMan].sv_A_bef_table, 2, U_D, crossings, ManeuverData[CurMan].sv_A_bef_table);
			}
		}

		if (Error) return Error;
		return 0;
	}

	int OrbitalManeuverProcessor::RunIterators()
	{
		VECTOR3 u;
		unsigned int l;

		for (l = 0; l < iterators.size(); l++)
		{
			if (iterators[l].constr == CurMan)
			{
				//NC Manever Iterator
				if (iterators[l].type == 1)
				{
					bool notconverged;
					if (iterators[l].constrtype == 1)
					{
						//Downrange position (DR)

						VECTOR3 R_REL, V_REL;
						OrbMech::SV SV_ACON = ManeuverData[CurMan].sv_A_bef_table;
						OrbMech::SV SV_PCON = ManeuverData[CurMan].sv_P_bef_table;
						OrbMech::REL_COMP(true, SV_PCON.R, SV_PCON.V, SV_ACON.R, SV_ACON.V, R_REL, V_REL);

						iterstate[l].err = iterators[l].value - R_REL.x;

						notconverged = (abs(iterstate[l].err) > ompvariables.RangeTolerance);
					}
					else
					{
						//Phase angle (PH)

						double phase;

						phase = OrbMech::PHSANG(ManeuverData[CurMan].sv_P_bef_table.R, ManeuverData[CurMan].sv_P_bef_table.V, ManeuverData[CurMan].sv_A_bef_table.R);
						iterstate[l].err = iterators[l].value - phase;

						notconverged = (abs(iterstate[l].err) > 0.001 * RAD);
					}

					if (notconverged)
					{
						iterstate[l].converged = false;
						iterstate[l].dv = ManeuverData[iterators[l].man].dv_table.x;
						OrbMech::ITER(iterstate[l].c_I, iterstate[l].s_F, iterstate[l].err, iterstate[l].p_H, iterstate[l].dv, iterstate[l].erro, iterstate[l].dvo);

						if (iterstate[l].s_F) return 20;	//Error 20: Too many iterations

						ManeuverData[iterators[l].man].dv_table.x = iterstate[l].dv;

						//return to maneuver
						CurMan = iterators[l].man;
						recycle = true;
						break;
					}
					else
					{
						iterstate[l].converged = true;
						iterstate[l].c_I = 0;
					}
				}
				//NH Maneuver Iterator
				if (iterators[l].type == 2)
				{
					//Calculate error
					VECTOR3 Rtemp, Vtemp;
					OrbMech::SV SV_ACON = ManeuverData[CurMan].sv_A_bef_table;
					OrbMech::SV SV_PCON = ManeuverData[CurMan].sv_P_bef_table;

					u = unit(crossp(SV_PCON.R, SV_PCON.V));
					SV_ACON.R = unit(SV_ACON.R - u * dotp(SV_ACON.R, u)) * length(SV_ACON.R);
					OrbMech::RADUP(SV_PCON.R, SV_PCON.V, SV_ACON.R, OrbMech::mu_Earth, Rtemp, Vtemp);

					iterstate[l].err = length(Rtemp) - length(SV_ACON.R) - iterators[l].value;

					if (abs(iterstate[l].err) > 10.0)
					{
						iterstate[l].converged = false;
						iterstate[l].dv = ManeuverData[iterators[l].man].dv_table.x;
						OrbMech::ITER(iterstate[l].c_I, iterstate[l].s_F, iterstate[l].err, iterstate[l].p_H, iterstate[l].dv, iterstate[l].erro, iterstate[l].dvo);

						if (iterstate[l].s_F) return 20;	//Error 20: Too many iterations

						ManeuverData[iterators[l].man].dv_table.x = iterstate[l].dv;

						//return to maneuver
						CurMan = iterators[l].man;
						recycle = true;
						break;
					}
					else
					{
						iterstate[l].converged = true;
						iterstate[l].c_I = 0;
					}
				}
				//NPC Iterator
				if (iterators[l].type == 3)
				{
					OrbMech::SV SV_ACON = ManeuverData[CurMan].sv_A_bef_table;
					OrbMech::SV SV_PCON = ManeuverData[CurMan].sv_P_bef_table;
					VECTOR3 H_A = crossp(SV_ACON.R, SV_ACON.V);
					VECTOR3 H_P = crossp(SV_PCON.R, SV_PCON.V);

					iterstate[l].err = OrbMech::acos2(dotp(H_A, H_P) / length(H_A) / length(H_P)) - iterators[l].value;

					if (abs(iterstate[l].err) > 0.0005 * RAD)
					{
						OrbMech::SV sv_PH;
						iterstate[l].converged = false;

						//Generate phantom plane
						u = unit(H_P);
						sv_PH = SV_ACON;
						sv_PH.R = unit(SV_ACON.R - u * dotp(SV_ACON.R, u)) * length(SV_ACON.R);
						sv_PH.V = unit(SV_ACON.V - u * dotp(SV_ACON.V, u)) * length(SV_ACON.V);

						//Iterate backwards to NPC TIG
						for (unsigned m = iterators[l].constr - 1; m >= iterators[l].man; m--)
						{
							Error = coast_auto(sv_PH, ManeuverData[m].sv_A_bef_table.GMT - sv_PH.GMT, sv_PH);
							if (Error) return Error;
							if (m > iterators[l].man)
							{
								sv_PH = ApplyLVLHManeuver(sv_PH, ManeuverData[m].dv_table, ManeuverData[m].ThrustProfile, false);
							}
							else break;
						}

						sv_phantom = sv_PH;

						iterstate[l].c_I += 1.0;
						if (iterstate[l].c_I > 15) return 20; //Error 20: Too many iterations

						//return to maneuver
						/*i = iterators[l].man;
						recycle = true;
						break;*/
					}
					else
					{
						iterstate[l].converged = true;
						iterstate[l].c_I = 0;
					}
				}
			}
		}

		return 0;
	}

	int OrbitalManeuverProcessor::ApplyManeuver()
	{
		//Get state vector at maneuver TIG
		OrbMech::SV sv_maneuver;

		if (ManeuverData[CurMan].ChaserMan)
		{
			sv_maneuver = ManeuverData[CurMan].sv_A_bef_table;
		}
		else
		{
			sv_maneuver = ManeuverData[CurMan].sv_P_bef_table;
		}

		if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::HA)
		{
			VECTOR3 DV;
			if (HeightManeuverAuto(ManeuverData[CurMan].sv_A_bef_table, OrbMech::EARTH_RADIUS_EQUATOR + ManeuverData[CurMan].add_constraint.x, true, DV, ManeuverData[CurMan].dv_table.x))
			{
				return 28;	//HA maneuver failed to converge
			}
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V), DV);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::HASH)
		{
			VECTOR3 DV;
			if (HeightManeuverAuto(sv_maneuver, OrbMech::EARTH_RADIUS_EQUATOR + ManeuverData[CurMan].add_constraint.x, false, DV))
			{
				return 29;	//HASH maneuver failed to converge
			}
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(sv_maneuver.R, sv_maneuver.V), DV);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::EXDV || ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NC || ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NH)
		{
			//Nothing to do
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NHRD)
		{
			double r_dot = dotp(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V) / length(ManeuverData[CurMan].sv_A_bef_table.R);
			ManeuverData[CurMan].dv_table.z = -r_dot;
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::SOI || ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::TPI)
		{
			VECTOR3 DV, offset;
			double ddt;
			if (ManeuverConstraintsTable[CurMan + 1].threshold == OMPDefs::THRESHOLD::THRES_T)
			{
				ddt = GMTfromGET(ManeuverConstraintsTable[CurMan + 1].thresh_num) - ManeuverData[CurMan].sv_A_bef_table.GMT;
			}
			else if (ManeuverConstraintsTable[CurMan + 1].threshold == OMPDefs::THRESHOLD::THRES_DT)
			{
				ddt = ManeuverConstraintsTable[CurMan + 1].thresh_num;
			}
			else if (ManeuverConstraintsTable[CurMan + 1].threshold == OMPDefs::THRESHOLD::THRES_WT)
			{
				ddt = OrbMech::time_theta(sv_P_cur.R, sv_P_cur.V, ManeuverConstraintsTable[CurMan + 1].thresh_num, OrbMech::mu_Earth);
			}
			else
			{
				return 23;	//No valid threshold for SOI/NCC/TPI
			}

			if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::SOI)
			{
				offset = ManeuverData[CurMan + 1].add_constraint;
			}
			else
			{
				offset = _V(0, 0, 0);
			}

			Error = SOIManeuver(ManeuverData[CurMan].sv_A_bef_table, sv_P_cur, ddt, offset, DV);
			if (Error) return Error;
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V), DV);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::SOR || ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::TPF)
		{
			VECTOR3 DV, offset;

			if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::SOR)
			{
				offset = ManeuverData[CurMan].add_constraint;
			}
			else
			{
				offset = _V(0, 0, 0);
			}

			Error = SORManeuver(ManeuverData[CurMan].sv_A_bef_table, sv_P_cur, offset, DV);
			if (Error) return Error;
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V), DV);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NPC)
		{
			OrbMech::SV sv_P1;
			VECTOR3 H_P, DV;
			//First iteration
			if (ManeuverData[CurMan].add_constraint.x == 0.0)
			{
				Error = coast_auto(sv_P_cur, ManeuverData[CurMan].sv_A_bef_table.GMT - sv_P_cur.GMT, sv_P1);
				if (Error) return Error;
				H_P = crossp(sv_P1.R, sv_P1.V);
			}
			else
			{
				H_P = ManeuverData[CurMan].add_constraint;
			}

			DV = NPCManeuver(ManeuverData[CurMan].sv_A_bef_table, H_P);
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V), DV);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NCC)
		{
			VECTOR3 DV;
			double ddt;
			if (ManeuverConstraintsTable[CurMan + 1].threshold == OMPDefs::THRESHOLD::THRES_T)
			{
				ddt = GMTfromGET(ManeuverConstraintsTable[CurMan + 1].thresh_num) - ManeuverData[CurMan].sv_A_bef_table.GMT;
			}
			else if (ManeuverConstraintsTable[CurMan + 1].threshold == OMPDefs::THRESHOLD::THRES_DT)
			{
				ddt = ManeuverConstraintsTable[CurMan + 1].thresh_num;
			}
			else
			{
				return 23;	//No valid threshold for SOI/NCC
			}

			Error = SOIManeuver(ManeuverData[CurMan].sv_A_bef_table, sv_P_cur, ddt, ManeuverData[CurMan + 1].add_constraint, DV);
			if (Error) return Error;
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V), DV);
		}
		//Apsidal Shift
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::APSO)
		{
			double r_dot;

			r_dot = dotp(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V) / length(ManeuverData[CurMan].sv_A_bef_table.R);
			ManeuverData[CurMan].dv_table = _V(0, 0, -2.0 * r_dot);
		}
		//Circularization
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::CIRC)
		{
			VECTOR3 DV;

			if (HeightManeuverAuto(sv_maneuver, length(sv_maneuver.R), false, DV))
			{
				return 30;	//CIRC maneuver failed to converge
			}
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(sv_maneuver.R, sv_maneuver.V), DV);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::DVPY || ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::DVYP)
		{
			double dv = ManeuverData[CurMan].add_constraint.x;
			double pit = ManeuverData[CurMan].add_constraint.y;
			double yaw = ManeuverData[CurMan].add_constraint.z;
			ManeuverData[CurMan].dv_table = _V(dv * cos(pit) * cos(yaw), dv * sin(yaw), -dv * sin(pit) * cos(yaw));
		}
		//Node Shift setting up common node 90° later
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NS)
		{
			double Y_A_dot = CalculateYDot(ManeuverData[CurMan].sv_A_bef_table.V, ManeuverData[CurMan].sv_P_bef_table.R, ManeuverData[CurMan].sv_P_bef_table.V);
			ManeuverData[CurMan].dv_table = _V(0, -Y_A_dot, 0);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NSR)
		{
			ManeuverData[CurMan].dv_table = NSRManeuver(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].sv_P_bef_table);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::NOSH)
		{
			VECTOR3 DV;

			DV = NodeShiftManeuver(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].add_constraint.x);
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V), DV);
		}
		else if (ManeuverConstraintsTable[CurMan].type == OMPDefs::MANTYPE::PC)
		{
			VECTOR3 DV;

			DV = PlaneChangeManeuver(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].add_constraint.x);
			ManeuverData[CurMan].dv_table = mul(OrbMech::LVLH_Matrix(ManeuverData[CurMan].sv_A_bef_table.R, ManeuverData[CurMan].sv_A_bef_table.V), DV);
		}

		//Additional secondary maneuver constraints
		for (unsigned int j = 0; j < ManeuverConstraintsTable[CurMan].secondaries.size(); j++)
		{
			if (ManeuverConstraintsTable[CurMan].secondaries[j].type == OMPDefs::SEC_NULL)
			{
				double Y_A_dot = CalculateYDot(ManeuverData[CurMan].sv_A_bef_table.V, ManeuverData[CurMan].sv_P_bef_table.R, ManeuverData[CurMan].sv_P_bef_table.V);
				ManeuverData[CurMan].dv_table.y = -Y_A_dot;
			}
		}

		//Calculate SV after the maneuver
		if (ManeuverData[CurMan].ChaserMan)
		{
			ManeuverData[CurMan].sv_A_aft_table = ApplyLVLHManeuver(ManeuverData[CurMan].sv_A_bef_table, ManeuverData[CurMan].dv_table, ManeuverData[CurMan].ThrustProfile);
			ManeuverData[CurMan].sv_P_aft_table = ManeuverData[CurMan].sv_P_bef_table;
		}
		else
		{
			ManeuverData[CurMan].sv_P_aft_table = ApplyLVLHManeuver(ManeuverData[CurMan].sv_P_bef_table, ManeuverData[CurMan].dv_table, ManeuverData[CurMan].ThrustProfile);
			ManeuverData[CurMan].sv_A_aft_table = ManeuverData[CurMan].sv_A_bef_table;
		}

		return 0;
	}

	bool OrbitalManeuverProcessor::IsOMPConverged() const
	{
		for (unsigned int i = 0; i < iterstate.size(); i++)
		{
			if (iterstate[i].converged == false) return false;
		}
		return true;
	}

	void OrbitalManeuverProcessor::CalculateManeuverEvalTable(OrbMech::SV sv_A0, OrbMech::SV sv_P0)
	{
		OrbMech::SV sv_cur, sv_P_cur, sv_temp;
		VECTOR3 u, R, Rtemp, Vtemp, R_REL, V_REL;
		double apo, peri, dt1, dt2;
		MANEVALDATA man;

		sv_cur = sv_A0;
		sv_P_cur = sv_P0;

		ManeuverEvaluationTable.Maneuvers.clear();
		ManeuverEvaluationTable.dv_C = ManeuverEvaluationTable.dv_T = _V(0, 0, 0);

		ManeuverEvaluationTable.GMT_C = sv_A0.GMT;
		ManeuverEvaluationTable.GMT_T = sv_P0.GMT;

		for (unsigned i = 0; i < ManeuverTable.size(); i++)
		{
			//Coast to TIG
			Error = coast_auto(sv_cur, ManeuverTable[i].TIG_GMT - sv_cur.GMT, sv_cur);
			if (Error) return;
			Error = coast_auto(sv_P_cur, ManeuverTable[i].TIG_GMT - sv_P_cur.GMT, sv_P_cur);
			if (Error) return;

			//Apply maneuver
			if (ManeuverData[i].ChaserMan)
			{
				man.sv_before = sv_cur;
				sv_cur = ApplyLVLHManeuver(sv_cur, ManeuverTable[i].dV_LVLH, ManeuverData[i].ThrustProfile);
				man.V_after = sv_cur.V;

				//Accumulate DV
				ManeuverEvaluationTable.dv_C.x += abs(ManeuverTable[i].dV_LVLH.x);
				ManeuverEvaluationTable.dv_C.y += abs(ManeuverTable[i].dV_LVLH.y);
				ManeuverEvaluationTable.dv_C.z += abs(ManeuverTable[i].dV_LVLH.z);
			}
			else
			{
				man.sv_before = sv_P_cur;
				sv_P_cur = ApplyLVLHManeuver(sv_P_cur, ManeuverTable[i].dV_LVLH, ManeuverData[i].ThrustProfile);
				man.V_after = sv_P_cur.V;

				//Accumulate DV
				ManeuverEvaluationTable.dv_T.x += abs(ManeuverTable[i].dV_LVLH.x);
				ManeuverEvaluationTable.dv_T.y += abs(ManeuverTable[i].dV_LVLH.y);
				ManeuverEvaluationTable.dv_T.z += abs(ManeuverTable[i].dV_LVLH.z);
			}

			//Calculate maneuver parameters
			man.type = GetOPMManeuverType(ManeuverTable[i].type);
			man.name = ManeuverTable[i].name;
			man.DVMag = length(ManeuverTable[i].dV_LVLH) / 0.3048;
			man.GMTIG = ManeuverTable[i].TIG_GMT;
			man.METIG = GETfromGMT(ManeuverTable[i].TIG_GMT);
			if (i < ManeuverTable.size() - 1)
			{
				man.DT = ManeuverTable[i + 1].TIG_GMT - ManeuverTable[i].TIG_GMT;
			}
			else
			{
				man.DT = 0.0;
			}
			man.DV = ManeuverTable[i].dV_LVLH / 0.3048;
			man.ChaserMan = ManeuverData[i].ChaserMan;
			if (ManeuverData[i].ChaserMan)
			{
				sv_temp = sv_cur;
			}
			else
			{
				sv_temp = sv_P_cur;
			}
			
			if (useNonSphericalGravity)
			{
				ApsidesMagnitudeDetermination(sv_temp, apo, peri);
			}
			else
			{
				OrbMech::periapo(sv_temp.R, sv_temp.V, OrbMech::mu_Earth, apo, peri);
			}

			man.HA = (apo - OrbMech::EARTH_RADIUS_EQUATOR) / 1852.0;
			man.HP = (peri - OrbMech::EARTH_RADIUS_EQUATOR) / 1852.0;

			//Calculate DH
			u = unit(crossp(sv_P_cur.R, sv_P_cur.V));
			R = unit(sv_cur.R - u * dotp(sv_cur.R, u)) * length(sv_cur.R);
			OrbMech::RADUP(sv_P_cur.R, sv_P_cur.V, R, OrbMech::mu_Earth, Rtemp, Vtemp);
			man.DH = (length(Rtemp) - length(R)) / 1852.0;

			//Calculate range, Y and Ydot
			OrbMech::REL_COMP(true, sv_P_cur.R, sv_P_cur.V, sv_cur.R, sv_cur.V, R_REL, V_REL);
			man.RANGE = length(sv_P_cur.R - sv_cur.R) / 1852.0;
			//man.RANGE = -R_REL.x / 1852.0;
			man.PHASE = -R_REL.x / length(sv_P_cur.R) * DEG;
			man.Y = R_REL.y / 0.3048;
			man.Ydot = V_REL.y / 0.3048;

			if (Sunrise(sv_cur, true, true, sv_temp))
			{
				return;
			}
			dt1 = sv_temp.GMT - sv_cur.GMT;

			if (Sunrise(sv_cur, false, true, sv_temp))
			{
				return;
			}
			dt2 = sv_temp.GMT - sv_cur.GMT;

			if (dt1 < dt2)
			{
				man.noon = false;
				man.TTN = dt1;
			}
			else
			{
				man.noon = true;
				man.TTN = dt2;
			}

			if (Sunrise(sv_cur, true, false, sv_temp))
			{
				return;
			}
			dt1 = sv_temp.GMT - sv_cur.GMT;

			if (Sunrise(sv_cur, false, false, sv_temp))
			{
				return;
			}
			dt2 = sv_temp.GMT - sv_cur.GMT;

			if (dt1 < dt2)
			{
				man.sunrise = true;
				man.TTS = dt1;
			}
			else
			{
				man.sunrise = false;
				man.TTS = dt2;
			}

			ManeuverEvaluationTable.Maneuvers.push_back(man);
		}

		ManeuverEvaluationTable.dv_C /= 0.3048;
		ManeuverEvaluationTable.dv_T /= 0.3048;
	}

	void OrbitalManeuverProcessor::GetOMPError(int err, std::string& buf, unsigned int i, unsigned int j) const
	{
		switch (err)
		{
		case 0:		buf = "";														break;
		case 1:		buf = "Error: No maneuvers in constraint table";				break;
		case 2:		buf = "Error: First maneuver needs a T as threshold";			break;
		case 3:		buf = "Error: Not enough DV components specified";				break;
		case 4:		buf = "Error: Too many DV components specified";				break;
		case 5:		buf = "Error: A maneuver doesn't have a threshold";				break;
		case 6:		buf = "Error: could not find HD constraint for HA maneuver";	break;
		case 7:		buf = "Error: NC maneuver has no DR constraint specified";		break;
		case 8:		buf = "Error: NH maneuver has no DH constraint specified";		break;
		case 9:		buf = "Error: too many CXYZ components specified";				break;
		case 10:	buf = "Error: not enough CXYZ components specified";			break;
		case 11:	buf = "Error: no maneuver after SOI/NCC";						break;
		case 12:	buf = "Error: wrong maneuver after SOI";						break;
		case 14:	buf = "Error: CN secondary only applies to NPC";				break;
		case 20:	buf = "Error: Too many iterations";								break;
		case 22:	buf = "Error: More than one NPC maneuver specified";			break;
		case 23:	buf = "Error: No valid threshold for SOI/NCC";					break;
		case 24:	buf = "Error: could not find DNOD constraint for NOSH maneuver"; break;
		case 25:	buf = "Error: OPT secondary only applies to NOSH";				break;
		case 26:	buf = "Error: Routine SEARMT could not find maneuver point";	break;
		case 27:	buf = "Error: could not find DPC constraint for PC maneuver";	break;
		case 28:	buf = "Error: HA maneuver failed to converge";					break;
		case 29:	buf = "Error: HASH maneuver failed to converge";				break;
		case 30:	buf = "Error: CIRC maneuver failed to converge";				break;
		case 31:	buf = "Error: Failed to converge on common node";				break;
		case 32:	buf = "Error: Wrong vehicle code in VFIL secondary";			break;
		case 100:	buf = "Error: No target vessel.";								break;
		case 1001:	buf = "Error: Trajectory became reentrant.";					break;
		case 1002:	buf = "Error: Kepler error in integrator.";						break;
		case 2003:	buf = "Error parsing MCT, threshold type of maneuver " + std::to_string(i + 1) + " illegal";		break;
		default:	buf = "Error: unknown error";									break;
		}
	}

	int OrbitalManeuverProcessor::coast_auto(OrbMech::SV sv0, double dt, OrbMech::SV& sv1) const
	{
		if (useNonSphericalGravity)
		{
			sv1 = OrbMech::coast(sv0, dt);
		}
		else
		{
			sv1 = OrbMech::coast_osc(sv0, dt, OrbMech::mu_Earth);
		}
		return 0;
	}

	int OrbitalManeuverProcessor::DeltaOrbitsAuto(OrbMech::SV sv0, double M, OrbMech::SV& sv1) const
	{
		return GeneralTrajectoryPropagation(sv0, 3, 0.0, M, sv1);
	}

	int OrbitalManeuverProcessor::GeneralTrajectoryPropagation(OrbMech::SV sv0, int opt, double param, double DN, OrbMech::SV& sv1) const
	{
		sv1 = OrbMech::GeneralTrajectoryPropagation(sv0, opt, param, DN, useNonSphericalGravity);
		return 0;
	}

	int OrbitalManeuverProcessor::timetoapo_auto(OrbMech::SV sv_A, double revs, OrbMech::SV& sv_out) const
	{
		double v_r = dotp(sv_A.R, sv_A.V) / length(sv_A.R);
		if (v_r > 0)
		{
			return GeneralTrajectoryPropagation(sv_A, 1, PI, revs - 1.0, sv_out);
		}
		else
		{
			return GeneralTrajectoryPropagation(sv_A, 1, PI, revs, sv_out);
		}
	}

	int OrbitalManeuverProcessor::timetoperi_auto(OrbMech::SV sv_A, double revs, OrbMech::SV& sv_out) const
	{
		return GeneralTrajectoryPropagation(sv_A, 1, 0.0, revs, sv_out);
	}

	int OrbitalManeuverProcessor::FindCommonNode(OrbMech::SV sv_A, OrbMech::SV sv_P, VECTOR3& u_d, double& dt) const
	{
		OrbMech::SV sv_A1, sv_P1;
		VECTOR3 H_A, H_P, C_N;
		double dtheta, ddt, eps_C;
		int C, CMAX;

		sv_A1 = sv_A;
		sv_P1 = sv_P;
		C = 0;
		dt = 0.0;

		CMAX = 20;
		eps_C = 0.0115 * RAD;

		do
		{
			//Take passive vehicle to position match
			if (PositionMatch(sv_A1, sv_P1, sv_P1))
			{
				return 1;
			}

			H_A = unit(crossp(sv_A1.R, sv_A1.V));
			H_P = unit(crossp(sv_P1.R, sv_P1.V));
			C_N = crossp(H_A, H_P);

			if (C == 0)
			{
				VECTOR3 T_EST = crossp(sv_A1.R, C_N);
				if (dotp(T_EST, H_A) < 0)
				{
					C_N = -C_N;
				}
				dtheta = -OrbMech::PHSANG(sv_A1.R, sv_A1.V, C_N);
			}
			else
			{
				double dtheta1, dtheta2;

				dtheta1 = -OrbMech::PHSANG(sv_A1.R, sv_A1.V, C_N);
				dtheta2 = -OrbMech::PHSANG(sv_A1.R, sv_A1.V, -C_N);
				if (abs(dtheta1) < abs(dtheta2))
				{
					dtheta = dtheta1;
				}
				else
				{
					dtheta = dtheta2;
				}
			}
			ddt = OrbMech::time_theta(sv_A1.R, sv_A1.V, dtheta, OrbMech::mu_Earth);
			if (coast_auto(sv_A1, ddt, sv_A1))
			{
				return 1;
			}
			dt += ddt;
			C++;
		} while (abs(dtheta) >= eps_C && C < CMAX);

		u_d = H_P;
		if (C >= CMAX)
		{
			return 1;
		}
		return 0;
	}

	int OrbitalManeuverProcessor::FindNthApsidalCrossingAuto(OrbMech::SV sv0, double N, OrbMech::SV& sv_out) const
	{
		int M = (int)N;
		bool even = (M % 2) == 0;
		double fact;
		if (even)
		{
			fact = -1.0;
		}
		else
		{
			fact = 1.0;
		}
		double v_r = dotp(sv0.R, sv0.V) / length(sv0.R);

		double DN;

		if (v_r > 0)
		{
			DN = (double)((M - 1) / 2);
		}
		else
		{
			DN = (double)(M / 2);
		}

		if (fact * v_r > 0)
		{
			//Apoapsis
			return GeneralTrajectoryPropagation(sv0, 1, PI, DN, sv_out);
		}
		else
		{
			//Periapsis
			return GeneralTrajectoryPropagation(sv0, 1, 0, DN, sv_out);
		}
	}

	int OrbitalManeuverProcessor::FindOptimumNodeShiftPoint(OrbMech::SV sv0, double dh, OrbMech::SV& sv_out) const
	{
		OrbMech::SV sv1;
		VECTOR3 H;
		double U_D, U_B, DN, i;

		//Calculate argument of latitude and inclination
		U_B = OrbMech::ArgLat(sv0.R, sv0.V);
		H = unit(crossp(sv0.R, sv0.V));
		i = acos(H.z);

		U_D = atan2(1.0 + cos(dh), -sin(dh) * cos(i));
		if (U_D < 0)
		{
			U_D += PI2;
		}

		if (U_B < U_D)
		{
			DN = 0.0;
		}
		else
		{
			if (U_B < U_D + PI)
			{
				U_D = U_D + PI;
				DN = 0.0;
			}
			else
			{
				DN = 1.0;
			}
		}

		return GeneralTrajectoryPropagation(sv0, 2, U_D, DN, sv1);
	}

	bool OrbitalManeuverProcessor::TLAT(VECTOR3 R, VECTOR3 V, double lat, int C, double& K_AD, double& dtheta) const
	{
		// INPUTS:
		// R = vehicle inertial position vector
		// V = vehicle inertial velocity vector
		// t = time of state vector
		// lat = Desired Earth-fixed latitude
		// C = iteration counter
		// OUTPUTS:
		// K_AD = nearest crossing flag
		// dtheta = travel angle to crossing
		// return value = true = no crossing, false = crossing possible

		VECTOR3 H, N;
		double i, U, Ulat;

		H = unit(crossp(R, V));
		N = unit(crossp(_V(0, 0, 1), H));
		i = acos(H.z);
		if (abs(lat) > i) return true;
		U = OrbMech::PHSANG(R, V, N);
		if (U < 0)
		{
			U += PI2;
		}
		Ulat = asin(abs(sin(lat)) / sin(i));
		if (C == 0)
		{
			K_AD = -1.0;
		}
		if (lat < 0)
		{
			U = U - PI;
		}
		U = atan2(sin(U), cos(U));
		if (U < 0)
		{
			U += PI2;
		}
		if (K_AD > 0)
		{
			Ulat = PI - Ulat;
		}
		if (C == 0)
		{
			double Ulat_pi = PI - Ulat;

			if (U >= Ulat)
			{
				if (U < Ulat_pi)
				{
					Ulat = Ulat_pi;
					K_AD = 1.0;
				}
				else
				{
					Ulat = Ulat + PI2;
				}
			}
		}
		dtheta = Ulat - U;
		if (C != 0)
		{
			if (abs(dtheta) > PI)
			{
				dtheta = dtheta - PI2 * OrbMech::sign(dtheta);
			}
		}
		return false;
	}

	double OrbitalManeuverProcessor::TLON(VECTOR3 R, VECTOR3 V, double t, double lng, int C, double w_E) const
	{
		// INPUTS:
		// R = vehicle inertial position vector
		// V = vehicle inertial velocity vector
		// t = time of state vector
		// lng = Desired Earth-fixed longitude
		// C = iteration counter
		// w_E = Rotational rate of Earth
		// OUTPUTS:
		// dtheta = travel angle to crossing

		VECTOR3 N, H;
		double alpha1, alpha2, I, NODE, U, pro, U_lng, dtheta;

		H = unit(crossp(R, V));
		N = unit(crossp(_V(0, 0, 1), H));
		I = acos(H.z);
		NODE = atan2(N.y, N.x);
		U = OrbMech::PHSANG(R, V, N);
		if (U < 0)
		{
			U += PI2;
		}
		pro = 1.0;
		if (PI05 - I < 0)
		{
			pro = -1.0;
		}

		alpha1 = pro * (lng - NODE);
		alpha2 = alpha1 + w_E * t;
		U_lng = atan2(sin(alpha2), pro * cos(alpha2) * cos(I));
		if (U_lng < 0)
		{
			U_lng += PI2;
		}
		if (C == 0)
		{
			if (U_lng < U)
			{
				U_lng = U_lng + PI2;
			}
			dtheta = U_lng - U;
		}
		else
		{
			dtheta = U_lng - U;
			if (abs(dtheta) >= PI)
			{
				dtheta = dtheta - OrbMech::sign(dtheta) * PI2;
			}
		}
		return dtheta;
	}

	bool OrbitalManeuverProcessor::TALT(VECTOR3 R, VECTOR3 V, double rad, int C, double mu, double& dtheta) const
	{
		// INPUTS:
		// R = vehicle inertial position vector
		// V = vehicle inertial velocity vector
		// rad = desired radius to cross
		// C = iteration counter
		// mu = standard gravitational parameter
		// OUTPUTS:
		// dtheta = travel angle to crossing
		// return value = true = no crossing, false = crossing possible

		double r, v, r_dot, P, a, ecosE, esinE, e, r_MIN, r_MAX, theta_D, theta;

		r = length(R);
		v = length(V);
		r_dot = dotp(V, R) / r;
		P = pow(length(crossp(R, V)), 2) / mu;
		a = r / (2.0 - v * v * r / mu);
		ecosE = (a - r) / a;
		esinE = r * r_dot / sqrt(mu * a);
		e = sqrt(esinE * esinE + ecosE * ecosE);
		r_MIN = P / (1.0 + e);
		r_MAX = 2.0 * a - r_MIN;

		//Error condition
		if (rad < r_MIN || r > r_MAX) return true;

		theta_D = acos((P / rad - 1.0) / e);
		theta = acos((P / r - 1.0) / e);

		if (C != 0)
		{
			dtheta = theta_D - theta;
			if (r_dot < 0)
			{
				dtheta = -dtheta;
			}
		}
		else
		{
			dtheta = 0.0;

			if (r_dot > 0)
			{
				if (theta_D > theta)
				{
					dtheta = theta_D - theta;
				}
				else if (theta_D < theta)
				{
					dtheta = PI2 - (theta_D + theta);
				}
			}
			else
			{
				if (theta_D > theta)
				{
					dtheta = theta + theta_D;
				}
				else if (theta_D < theta)
				{
					dtheta = theta - theta_D;
				}
			}
		}
		return false;
	}

	int OrbitalManeuverProcessor::SEARMT(OrbMech::SV sv0, int opt, double val, OrbMech::SV& sv1) const
	{
		double K_AD, dtheta, dt, l_dot;
		int C, Error;
		bool CEND;

		const int CMAX = 20;
		const double eps4 = 0.011 * RAD;

		C = 0;
		CEND = false;
		sv1 = sv0;
		l_dot = OrbMech::GetMeanMotion(sv0.R, sv0.V, OrbMech::mu_Earth);

		do
		{
			if (opt == OMPDefs::SECONDARIES::LAT || opt == OMPDefs::SECONDARIES::DEC)
			{
				if (TLAT(sv1.R, sv1.V, val, C, K_AD, dtheta))
				{
					//Error
					return 1;
				}
			}
			else if (opt == OMPDefs::SECONDARIES::ALT)
			{
				if (TALT(sv1.R, sv1.V, val + OrbMech::EARTH_RADIUS_EQUATOR, C, OrbMech::mu_Earth, dtheta))
				{
					//Error
					return 1;
				}
			}
			else
			{
				dtheta = TLON(sv1.R, sv1.V, sv1.GMT, val, C, OrbMech::w_Earth);
			}

			dt = dtheta / l_dot;
			if (C == 0)
			{
				if (dt < 0.0)
				{
					dt = dt + PI2 / l_dot;
				}
			}

			Error = coast_auto(sv1, dt, sv1);
			if (Error) return Error;
			C++;
		} while (abs(dtheta) > eps4 && C < CMAX);

		if (C >= CMAX)
		{
			return true;
		}
		return false;
	}

	int OrbitalManeuverProcessor::PositionMatch(OrbMech::SV sv_A, OrbMech::SV sv_P, OrbMech::SV& sv_P2) const
	{
		//Take target (sv_P) to same time as sv_A and then to position match

		OrbMech::SV sv_P1;
		double phase, n, ddt;

		if (coast_auto(sv_P, sv_A.GMT - sv_P.GMT, sv_P1))
		{
			return 1;
		}

		do
		{
			phase = OrbMech::PHSANG(sv_A.R, sv_A.V, sv_P1.R);
			n = OrbMech::GetMeanMotion(sv_P1.R, sv_P1.V, OrbMech::mu_Earth);
			ddt = phase / n;
			if (coast_auto(sv_P1, ddt, sv_P1))
			{
				return 1;
			}
		} while (abs(ddt) > 0.01);

		sv_P2 = sv_P1;

		return 0;
	}

	int OrbitalManeuverProcessor::Sunrise(OrbMech::SV sv0, bool rise, bool midnight, OrbMech::SV& sv1) const
	{
		//Find next environment change

		//midnight = 0-> rise=0:sunset, rise=1:sunrise
		//midnight = 1-> rise=0:midday, rise=1:midnight

		OrbMech::SV sv;
		VECTOR3 R_S, H, N;
		double R_e, r, l_dot, g_dot, cos_phi1, phi1, sin_alpha, cos_eta, sin_eta, phi2, F, phi3, dt;
		int IC;

		//Set up iteration
		R_e = OrbMech::EARTH_RADIUS_EQUATOR;
		sv = sv0;
		l_dot = OrbMech::GetMeanMotion(sv.R, sv.V, OrbMech::mu_Earth);
		g_dot = 0.0;
		IC = 0;

		do
		{
			R_S = unit(OrbMech::SUN(sesconst.GMTBASE, sv.GMT, sesconst.M_TEG_TO_M50));
			r = length(sv.R);

			H = unit(crossp(sv.R, sv.V));
			N = unit(crossp(R_S, H));
			//Rise?
			if (rise == false)
			{
				N = -N;
			}
			cos_phi1 = dotp(unit(sv.R), N);
			phi1 = acos(cos_phi1);
			sin_alpha = sqrt(1.0 - pow(R_e / r, 2));
			cos_eta = dotp(H, R_S);
			sin_eta = sqrt(1.0 - cos_eta * cos_eta);
			if (sin_eta <= sin_alpha)
			{
				//Orbit is totally in daylight
				return 1;
			}

			if (midnight)
			{
				if (rise == false)
				{
					phi2 = -PI05;
				}
				else
				{
					phi2 = PI05;
				}
			}
			else
			{
				phi2 = asin(sin_alpha / sin_eta);
			}
			F = sv.R.x * N.y - sv.R.y * N.x;
			if (rise == false)
			{
				if (F >= 0.0)
				{
					phi3 = phi1 + phi2;
				}
				else
				{
					if (cos_phi1 > 0.0)
					{
						phi3 = phi2 - phi1;
					}
					else
					{
						phi3 = PI2 - phi1 + phi2;
					}
				}
			}
			else
			{
				if (F > 0.0)
				{
					phi3 = phi1 - phi2;
				}
				else
				{
					phi3 = -phi1 - phi2;
				}
			}

			//For first iteration, force to search forward
			if (IC == 0 && phi3 < 0.0)
			{
				phi3 += PI2;
			}

			dt = phi3 / (l_dot + g_dot);
			if (coast_auto(sv, dt, sv))
			{
				return 2;
			}
			IC++;
			if (IC > 15)
			{
				return 1;
			}
		} while (abs(dt) > 0.25);

		//Output
		sv1 = sv;

		return 0;
	}

	int OrbitalManeuverProcessor::FindOrbitalSunriseRelativeTime(OrbMech::SV sv0, bool sunrise, double dt1, OrbMech::SV& sv_out)
	{
		//sunrise: true = rise, false = set
		//dt1: Offset from time it finds

		OrbMech::SV sv1;

		Error = Sunrise(sv0, sunrise, false, sv1);
		if (Error) return Error;

		//Now to offset
		if (coast_auto(sv1, dt1, sv_out))
		{
			return 1;
		}
		return 0;
	}

	int OrbitalManeuverProcessor::FindOrbitalMidnightRelativeTime(OrbMech::SV sv0, bool midnight, double dt1, OrbMech::SV& sv_out)
	{
		//sunrise: true = rise, false = set
		//dt1: Offset from time it finds

		OrbMech::SV sv1;

		Error = Sunrise(sv0, midnight, true, sv1);
		if (Error) return Error;

		//Now to offset
		if (coast_auto(sv1, dt1, sv_out))
		{
			return 1;
		}
		return 0;
	}

	bool OrbitalManeuverProcessor::HeightManeuverAuto(OrbMech::SV sv_A, double r_D, bool horizontal, VECTOR3& DV, double dv_guess)
	{
		OrbMech::SV sv_A_apo, sv_D;
		OrbMech::OELEMENTS coe;
		VECTOR3 am, U_hor;
		double eps, p_H, c_I, u_b, u_d, DN, v_H, e_H, e_Ho, v_Ho;
		int s_F;

		sv_A_apo = sv_A;

		//Some helping vectors
		am = unit(crossp(sv_A.R, sv_A.V));
		U_hor = unit(crossp(am, unit(sv_A.R)));

		//Tolerance
		eps = 250.0 * 0.3048;
		//Set up iterator
		p_H = c_I = 0.0;
		s_F = 0;

		//Set up desired argument of latitude (only used with non-spherical gravity)
		coe = OrbMech::coe_from_sv(sv_A.R, sv_A.V, OrbMech::mu_Earth);
		u_b = fmod(coe.TA + coe.w, PI2);
		u_d = u_b + PI;
		if (u_d < PI2)
		{
			DN = 0.0;
		}
		else
		{
			u_d = u_d - PI2;
			DN = 1.0;
		}

		//Initial guess
		if (horizontal)
		{
			v_H = dv_guess;
		}
		else
		{
			v_H = sqrt(2.0 * OrbMech::mu_Earth / (length(sv_A.R) * (1.0 + length(sv_A.R) / r_D)));
		}

		do
		{
			if (horizontal)
			{
				sv_A_apo.V = sv_A.V + U_hor * v_H;
			}
			else
			{
				sv_A_apo.V = U_hor * v_H;
			}

			if (GeneralTrajectoryPropagation(sv_A_apo, 2, u_d, DN, sv_D))
			{
				return true;
			}

			e_H = length(sv_D.R) - r_D;
			if (abs(e_H) >= eps)
			{
				OrbMech::ITER(c_I, s_F, e_H, p_H, v_H, e_Ho, v_Ho);
				if (s_F == 1)
				{
					//Error
					return true;
				}
			}
		} while (abs(e_H) >= eps);

		DV = sv_A_apo.V - sv_A.V;
		return false;
	}

	int OrbitalManeuverProcessor::Lambert(OrbMech::SV sv_A1, VECTOR3 RP2_off, double dt, VECTOR3& V_A1_apo)
	{
		OrbMech::SV sv_A1_apo, sv_A2;
		VECTOR3 R3, DR;
		int IC;

		sv_A1_apo = sv_A1;
		R3 = RP2_off;
		IC = 0;

		do
		{
			//Calculate required velocity
			sv_A1_apo.V = OrbMech::elegant_lambert(sv_A1.R, sv_A1.V, R3, dt, 0, true, OrbMech::mu_Earth);
			//Error?
			if (length(sv_A1_apo.V) == 0.0)
			{
				return 1;
			}
			//Coast to T2
			if (coast_auto(sv_A1_apo, dt, sv_A2))
			{
				return 1;
			}
			DR = sv_A2.R - RP2_off;
			R3 = R3 - DR;
			IC++;
			if (IC > 15)
			{
				return 2;
			}
		} while (length(DR) > 100.0 * 0.3048);

		V_A1_apo = sv_A1_apo.V;

		return 0;
	}

	int OrbitalManeuverProcessor::SOIManeuver(OrbMech::SV sv_A1, OrbMech::SV sv_P, double dt, VECTOR3 off, VECTOR3& DV)
	{
		//sv_A is at T1
		OrbMech::SV sv_P2;
		VECTOR3 RP2_off, V_A1_apo;

		if (coast_auto(sv_P, sv_A1.GMT + dt - sv_P.GMT, sv_P2))
		{
			return 1;
		}

		//Get target position
		OrbMech::REL_COMP(sv_P2.R, sv_P2.V, RP2_off, off);

		if (Lambert(sv_A1, RP2_off, dt, V_A1_apo))
		{
			return 2;
		}

		DV = V_A1_apo - sv_A1.V;

		return 0;
	}

	int OrbitalManeuverProcessor::SORManeuver(OrbMech::SV sv_A1, OrbMech::SV sv_P, VECTOR3 off, VECTOR3& DV)
	{
		//sv_A is at T1
		OrbMech::SV sv_P1, sv_P2;
		VECTOR3 RP2_off, V_A1_apo;
		double dt;

		if (coast_auto(sv_P, sv_A1.GMT - sv_P.GMT, sv_P1))
		{
			return 1;
		}

		dt = OrbMech::time_theta(sv_P1.R, sv_P1.V, 270.0 * RAD, OrbMech::mu_Earth);

		if (coast_auto(sv_P1, dt, sv_P2))
		{
			return 1;
		}

		//Get target position
		OrbMech::REL_COMP(sv_P2.R, sv_P2.V, RP2_off, off);

		if (Lambert(sv_A1, RP2_off, dt, V_A1_apo))
		{
			return 2;
		}

		DV = V_A1_apo - sv_A1.V;

		return 0;
	}

	VECTOR3 OrbitalManeuverProcessor::NPCManeuver(OrbMech::SV sv_A, VECTOR3 H_P) const
	{
		VECTOR3 I_T, V2;

		I_T = unit(H_P);
		V2 = unit(sv_A.V - I_T * dotp(sv_A.V, I_T)) * length(sv_A.V);
		return V2 - sv_A.V;
	}

	double OrbitalManeuverProcessor::CalculateYDot(VECTOR3 V_A, VECTOR3 R_P, VECTOR3 V_P) const
	{
		VECTOR3 u2 = unit(crossp(V_P, R_P));
		double Y_A_dot = dotp(V_A, u2);
		return Y_A_dot;
	}

	VECTOR3 OrbitalManeuverProcessor::NSRManeuver(OrbMech::SV sv_A, OrbMech::SV sv_P) const
	{
		//Assume both SVs are at TIG
		VECTOR3 R_P2, V_P2, R_A2, V_A2, R_PC, V_PC, u, V_A2_apo, DV2, X, Y, Z;
		double DH_CDH, r_PC, r_A2, v_PV, a_A, a_P, v_A2, v_AV, v_AH;

		R_A2 = sv_A.R;
		r_A2 = length(R_A2);
		V_A2 = sv_A.V;
		v_A2 = length(V_A2);
		R_P2 = sv_P.R;
		V_P2 = sv_P.V;
		u = unit(crossp(R_P2, V_P2));
		R_A2 = unit(R_A2 - u * dotp(R_A2, u)) * r_A2;
		V_A2 = unit(V_A2 - u * dotp(V_A2, u)) * v_A2;

		OrbMech::RADUP(R_P2, V_P2, R_A2, OrbMech::mu_Earth, R_PC, V_PC);
		a_P = OrbMech::GetSemiMajorAxis(R_PC, V_PC, OrbMech::mu_Earth);

		r_PC = length(R_PC);
		DH_CDH = r_PC - r_A2;
		v_PV = dotp(V_PC, R_A2) / r_A2;
		a_A = a_P - DH_CDH;
		v_AV = v_PV * pow(a_P / a_A, 1.5);
		v_AH = sqrt(OrbMech::mu_Earth * (2.0 / r_A2 - 1.0 / a_A) - v_AV * v_AV);
		V_A2_apo = unit(crossp(u, R_A2)) * v_AH + unit(R_A2) * v_AV;
		DV2 = V_A2_apo - V_A2;
		Z = unit(-R_A2);
		Y = -u;
		X = unit(crossp(Y, Z));
		return tmul(_M(X.x, Y.x, Z.x, X.y, Y.y, Z.y, X.z, Y.z, Z.z), DV2);
	}

	VECTOR3 OrbitalManeuverProcessor::NodeShiftManeuver(OrbMech::SV sv0, double dh_D) const
	{
		OrbMech::CELEMENTS coe_b, coe_a;
		VECTOR3 Rtemp, Vtemp;
		double cos_u_a, sin_u_a, u_b, u_a, cos_dw, sin_dw, dw, f_a, f_b;

		coe_b = OrbMech::CartesianToKeplerian(sv0.R, sv0.V, OrbMech::mu_Earth);
		f_b = OrbMech::MeanToTrueAnomaly(coe_b.l, coe_b.e);
		u_b = fmod(f_b + coe_b.g, PI2);

		coe_a = coe_b;
		f_a = f_b;

		cos_u_a = cos(dh_D) * cos(u_b) + sin(dh_D) * sin(u_b) * cos(coe_b.i);
		sin_u_a = sqrt(1.0 - cos_u_a * cos_u_a);
		if (u_b > PI)
		{
			sin_u_a = -sin_u_a;
		}
		u_a = atan2(sin_u_a, cos_u_a);
		if (u_a < 0)
		{
			u_a = PI2;
		}
		cos_dw = (cos(dh_D) - cos(u_b) * cos(u_a)) / (sin(u_b) * sin(u_a));
		sin_dw = sin(dh_D) * sin(coe_b.i) / sin_u_a;
		dw = atan2(sin_dw, cos_dw);
		coe_a.i = acos(cos(coe_b.i) * cos_dw - sin(coe_b.i) * sin_dw * cos(u_b));
		coe_a.h = coe_b.h + dh_D;
		coe_a.h = fmod(coe_a.h, PI2);
		coe_a.g = u_a - f_a;
		if (coe_a.h < 0)
		{
			coe_a.h += PI2;
		}
		if (coe_a.g < 0)
		{
			coe_a.g += PI2;
		}
		OrbMech::KeplerianToCartesian(coe_a, OrbMech::mu_Earth, Rtemp, Vtemp);

		return Vtemp - sv0.V;
	}

	VECTOR3 OrbitalManeuverProcessor::PlaneChangeManeuver(OrbMech::SV sv0, double dw_D) const
	{
		OrbMech::CELEMENTS coe_b, coe_a;
		VECTOR3 Rtemp, Vtemp;
		double f_b, u_b, sin_dh, cos_dh, dh, sin_u_a, cos_u_a, u_a, f_a;

		coe_b = OrbMech::CartesianToKeplerian(sv0.R, sv0.V, OrbMech::mu_Earth);
		f_b = OrbMech::MeanToTrueAnomaly(coe_b.l, coe_b.e);
		u_b = fmod(f_b + coe_b.g, PI2);

		coe_a = coe_b;
		f_a = f_b;

		coe_a.i = acos(cos(coe_b.i) * cos(dw_D) - sin(coe_b.i) * sin(dw_D) * cos(u_b));
		sin_dh = sin(dw_D) * sin(u_b) / sin(coe_a.i);
		cos_dh = (cos(dw_D) - cos(coe_b.i) * cos(coe_a.i)) / (sin(coe_b.i) * sin(coe_a.i));
		dh = atan2(sin_dh, cos_dh);
		coe_a.h = coe_b.h + dh;
		if (coe_a.h < 0)
		{
			coe_a.h += PI2;
		}
		else if (coe_a.h >= PI2)
		{
			coe_a.h -= PI2;
		}
		sin_u_a = sin(u_b) * sin(coe_b.i) / sin(coe_a.i);
		cos_u_a = cos(u_b) * cos(dh) + sin(u_b) * sin(dh) * cos(coe_b.i);
		u_a = atan2(sin_u_a, cos_u_a);
		if (u_a < 0)
		{
			u_a += PI2;
		}
		coe_a.g = u_a - f_a;
		if (coe_a.g < 0)
		{
			coe_a.g += PI2;
		}

		OrbMech::KeplerianToCartesian(coe_a, OrbMech::mu_Earth, Rtemp, Vtemp);
		return Vtemp - sv0.V;
	}

	double OrbitalManeuverProcessor::GMTfromGET(double get) const
	{
		return get + sesconst.GMTLO;
	}

	double OrbitalManeuverProcessor::GETfromGMT(double gmt) const
	{
		return gmt - sesconst.GMTLO;
	}

	OrbMech::SV OrbitalManeuverProcessor::ApplyLVLHManeuver(OrbMech::SV sv0, VECTOR3 DV_LVLH, int ThrustProfile, bool forwards) const
	{
		OrbMech::SV sv1;
		MATRIX3 Rot;
		VECTOR3 DV;
		double isp, sgn, dv;

		//Rough estimate
		isp = ompvariables.ISP[ThrustProfile];

		Rot = OrbMech::LVLH_Matrix(sv0.R, sv0.V);
		DV = tmul(Rot, DV_LVLH);
		dv = length(DV);

		sv1 = sv0;

		if (forwards)
		{
			sv1.V += DV;
			sgn = -1.0;
		}
		else
		{
			sv1.V -= DV;
			sgn = 1.0;
		}

		//Estimate weight
		sv1.mass = sv0.mass * exp(sgn * dv / isp);

		return sv1;
	}

	std::string OrbitalManeuverProcessor::GetOPMManeuverType(OMPDefs::MANTYPE type)
	{
		if (type == OMPDefs::MANTYPE::HA) return "HA";
		else if (type == OMPDefs::MANTYPE::HASH) return "HASH";
		else if (type == OMPDefs::MANTYPE::NC) return "NC";
		else if (type == OMPDefs::MANTYPE::EXDV) return "EXDV";
		else if (type == OMPDefs::MANTYPE::NH) return "NH";
		else if (type == OMPDefs::MANTYPE::SOI) return "SOI";
		else if (type == OMPDefs::MANTYPE::SOR) return "SOR";
		else if (type == OMPDefs::MANTYPE::NPC) return "NPC";
		else if (type == OMPDefs::MANTYPE::NCC) return "NCC";
		else if (type == OMPDefs::MANTYPE::APSO) return "APSO";
		else if (type == OMPDefs::MANTYPE::CIRC) return "CIRC";
		else if (type == OMPDefs::MANTYPE::NHRD) return "NHRD";
		else if (type == OMPDefs::MANTYPE::NSR) return "NSR";
		else if (type == OMPDefs::MANTYPE::NOSH) return "NOSH";
		else if (type == OMPDefs::MANTYPE::PC) return "PC";
		else if (type == OMPDefs::MANTYPE::TPI) return "TPI";
		else if (type == OMPDefs::MANTYPE::TPF) return "TPF";

		return "N/A";
	}

	OMPDefs::MANTYPE OrbitalManeuverProcessor::GetOPMManeuverType(std::string buf)
	{
		if (buf == "HA") return OMPDefs::HA;
		else if (buf == "HASH") return OMPDefs::HASH;
		else if (buf == "NC") return OMPDefs::NC;
		else if (buf == "EXDV") return OMPDefs::EXDV;
		else if (buf == "NH") return OMPDefs::NH;
		else if (buf == "SOI") return OMPDefs::SOI;
		else if (buf == "SOR") return OMPDefs::SOR;
		else if (buf == "NPC") return OMPDefs::NPC;
		else if (buf == "NCC") return OMPDefs::NCC;
		else if (buf == "APSO") return OMPDefs::APSO;
		else if (buf == "CIRC") return OMPDefs::CIRC;
		else if (buf == "NHRD") return OMPDefs::NHRD;
		else if (buf == "NSR") return OMPDefs::NSR;
		else if (buf == "NOSH") return OMPDefs::NOSH;
		else if (buf == "PC") return OMPDefs::PC;
		else if (buf == "TPI") return OMPDefs::TPI;
		else if (buf == "TPF") return OMPDefs::TPF;

		return OMPDefs::NOMAN;
	}

	OMP::OMPDefs::THRESHOLD OrbitalManeuverProcessor::GetOPMThresholdType(std::string buf)
	{
		OMP::OMPDefs::THRESHOLD type = OMP::OMPDefs::THRESHOLD::NOTHR;

		if (buf == "APS")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_APS;
		}
		else if (buf == "CAN")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_CAN;
		}
		else if (buf == "DLT")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_DLT;
		}
		else if (buf == "DT")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_DT;
		}
		else if (buf == "DTL")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_DTL;
		}
		else if (buf == "M")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_M;
		}
		else if (buf == "REV")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_REV;
		}
		else if (buf == "T")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_T;
		}
		else if (buf == "N")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_N;
		}
		else if (buf == "WT")
		{
			type = OMP::OMPDefs::THRESHOLD::THRES_WT;
		}
		return type;
	}

	double DDDHHHMMSS2MET(int dd, int hh, int mm, double ss)
	{
		return ss + 60.0 * mm + 3600.0 * hh + 24.0 * 3600.0 * dd;
	}

	int OrbitalManeuverProcessor::GetOMPThresholdValue(std::string buf, OMP::OMPDefs::THRESHOLD type, double& val)
	{
		switch (type)
		{
		case OMP::OMPDefs::THRESHOLD::THRES_APS:
		case OMP::OMPDefs::THRESHOLD::THRES_CAN:
		case OMP::OMPDefs::THRESHOLD::THRES_M:
		case OMP::OMPDefs::THRESHOLD::THRES_N:
		case OMP::OMPDefs::THRESHOLD::THRES_REV:
		case OMP::OMPDefs::THRESHOLD::THRES_WT:
		{
			if (sscanf_s(buf.c_str(), "%lf", &val) != 1) return 1;

			if (type == OMP::OMPDefs::THRESHOLD::THRES_CAN || type == OMP::OMPDefs::THRESHOLD::THRES_WT)
			{
				val *= RAD;
			}
		}
		break;
		case OMP::OMPDefs::THRESHOLD::THRES_DLT:
		case OMP::OMPDefs::THRESHOLD::THRES_DT:
		case OMP::OMPDefs::THRESHOLD::THRES_DTL:
		case OMP::OMPDefs::THRESHOLD::THRES_T:
		{
			int dd, hh, mm;
			double ss;
			if (sscanf_s(buf.c_str(), "%d:%d:%d:%lf", &dd, &hh, &mm, &ss) != 4) return 1;
			val = DDDHHHMMSS2MET(dd, hh, mm, ss);
		}
		break;
		default:
			return 2;
		}

		return 0;
	}

	OMP::OMPDefs::SECONDARIES OrbitalManeuverProcessor::GetSecondaryType(std::string buf)
	{
		if (buf == "A") return OMP::OMPDefs::SECONDARIES::A;
		else if (buf == "ALT") return OMP::OMPDefs::SECONDARIES::ALT;
		else if (buf == "APO") return OMP::OMPDefs::SECONDARIES::APO;
		else if (buf == "APS") return OMP::OMPDefs::SECONDARIES::SEC_APS;
		else if (buf == "ARG") return OMP::OMPDefs::SECONDARIES::ARG;
		else if (buf == "ASC") return OMP::OMPDefs::SECONDARIES::ASC;
		else if (buf == "CN") return OMP::OMPDefs::SECONDARIES::CN;
		else if (buf == "DEC") return OMP::OMPDefs::SECONDARIES::DEC;
		else if (buf == "DSC") return OMP::OMPDefs::SECONDARIES::DSC;
		else if (buf == "EL") return OMP::OMPDefs::SECONDARIES::EL;
		else if (buf == "LAT") return OMP::OMPDefs::SECONDARIES::LAT;
		else if (buf == "LON") return OMP::OMPDefs::SECONDARIES::LON;
		else if (buf == "N") return OMP::OMPDefs::SECONDARIES::N;
		else if (buf == "NA") return OMP::OMPDefs::SECONDARIES::NA;
		else if (buf == "NP") return OMP::OMPDefs::SECONDARIES::NP;
		else if (buf == "OPT") return OMP::OMPDefs::SECONDARIES::OPT;
		else if (buf == "P") return OMP::OMPDefs::SECONDARIES::P;
		else if (buf == "PER") return OMP::OMPDefs::SECONDARIES::PER;
		else if (buf == "RAS") return OMP::OMPDefs::SECONDARIES::RAS;
		else if (buf == "TGTA") return OMP::OMPDefs::SECONDARIES::TGTA;
		else if (buf == "TGTP") return OMP::OMPDefs::SECONDARIES::TGTP;
		else if (buf == "U") return OMP::OMPDefs::SECONDARIES::U;
		else if (buf == "HD") return OMP::OMPDefs::SECONDARIES::HD;
		else if (buf == "DV") return OMP::OMPDefs::SECONDARIES::DV;
		else if (buf == "DVLS") return OMP::OMPDefs::SECONDARIES::DVLS;
		else if (buf == "DVLV") return OMP::OMPDefs::SECONDARIES::DVLV;
		else if (buf == "LITI") return OMP::OMPDefs::SECONDARIES::LITI;
		else if (buf == "LITM") return OMP::OMPDefs::SECONDARIES::LITM;
		else if (buf == "LITO") return OMP::OMPDefs::SECONDARIES::LITO;
		else if (buf == "NITI") return OMP::OMPDefs::SECONDARIES::NITI;
		else if (buf == "NITM") return OMP::OMPDefs::SECONDARIES::NITM;
		else if (buf == "NITO") return OMP::OMPDefs::SECONDARIES::NITO;
		else if (buf == "CXYZ") return OMP::OMPDefs::SECONDARIES::CXYZ;
		else if (buf == "DH") return OMP::OMPDefs::SECONDARIES::DH;
		else if (buf == "DNOD") return OMP::OMPDefs::SECONDARIES::DNOD;
		else if (buf == "DPC") return OMP::OMPDefs::SECONDARIES::DPC;
		else if (buf == "DR") return OMP::OMPDefs::SECONDARIES::DR;
		else if (buf == "ITSR") return OMP::OMPDefs::SECONDARIES::ITSR;
		else if (buf == "MREV") return OMP::OMPDefs::SECONDARIES::MREV;
		else if (buf == "NULL") return OMP::OMPDefs::SECONDARIES::SEC_NULL;
		else if (buf == "PIT") return OMP::OMPDefs::SECONDARIES::PIT;
		else if (buf == "VFIL") return OMP::OMPDefs::SECONDARIES::VFIL;
		else if (buf == "YAW") return OMP::OMPDefs::SECONDARIES::YAW;
		else if (buf == "PHA") return OMP::OMPDefs::SECONDARIES::PHA;
		else if (buf == "WEDG") return OMP::OMPDefs::SECONDARIES::WEDG;
		else return OMP::OMPDefs::SECONDARIES::NOSEC;
	}

	std::string OrbitalManeuverProcessor::GetSecondaryName(OMP::OMPDefs::SECONDARIES sec)
	{
		switch (sec)
		{
		case OMP::OMPDefs::SECONDARIES::A: return "A";
		case OMP::OMPDefs::SECONDARIES::ALT: return "ALT";
		case OMP::OMPDefs::SECONDARIES::APO: return "APO";
		case OMP::OMPDefs::SECONDARIES::SEC_APS: return "APS";
		case OMP::OMPDefs::SECONDARIES::ARG: return "ARG";
		case OMP::OMPDefs::SECONDARIES::ASC: return "ASC";
		case OMP::OMPDefs::SECONDARIES::CN: return "CN";
		case OMP::OMPDefs::SECONDARIES::DEC: return "DEC";
		case OMP::OMPDefs::SECONDARIES::DSC: return "DSC";
		case OMP::OMPDefs::SECONDARIES::EL: return "EL";
		case OMP::OMPDefs::SECONDARIES::LAT: return "LAT";
		case OMP::OMPDefs::SECONDARIES::LON: return "LON";
		case OMP::OMPDefs::SECONDARIES::N: return "N";
		case OMP::OMPDefs::SECONDARIES::NA: return "NA";
		case OMP::OMPDefs::SECONDARIES::NP: return "NP";
		case OMP::OMPDefs::SECONDARIES::OPT: return "OPT";
		case OMP::OMPDefs::SECONDARIES::P: return "P";
		case OMP::OMPDefs::SECONDARIES::PER: return "PER";
		case OMP::OMPDefs::SECONDARIES::RAS: return "RAS";
		case OMP::OMPDefs::SECONDARIES::TGTA: return "TGTA";
		case OMP::OMPDefs::SECONDARIES::TGTP: return "TGTP";
		case OMP::OMPDefs::SECONDARIES::U: return "U";
		case OMP::OMPDefs::SECONDARIES::HD: return "HD";
		case OMP::OMPDefs::SECONDARIES::DV: return "DV";
		case OMP::OMPDefs::SECONDARIES::DVLS: return "DVLS";
		case OMP::OMPDefs::SECONDARIES::DVLV: return "DVLV";
		case OMP::OMPDefs::SECONDARIES::LITI: return "LITI";
		case OMP::OMPDefs::SECONDARIES::LITM: return "LITM";
		case OMP::OMPDefs::SECONDARIES::LITO: return "LITO";
		case OMP::OMPDefs::SECONDARIES::NITI: return "NITI";
		case OMP::OMPDefs::SECONDARIES::NITM: return "NITM";
		case OMP::OMPDefs::SECONDARIES::NITO: return "NITO";
		case OMP::OMPDefs::SECONDARIES::CXYZ: return "CXYZ";
		case OMP::OMPDefs::SECONDARIES::DH: return "DH";
		case OMP::OMPDefs::SECONDARIES::DNOD: return "DNOD";
		case OMP::OMPDefs::SECONDARIES::DPC: return "DPC";
		case OMP::OMPDefs::SECONDARIES::DR: return "DR";
		case OMP::OMPDefs::SECONDARIES::ITSR: return "ITSR";
		case OMP::OMPDefs::SECONDARIES::MREV: return "MREV";
		case OMP::OMPDefs::SECONDARIES::SEC_NULL: return "NULL";
		case OMP::OMPDefs::SECONDARIES::PIT: return "PIT";
		case OMP::OMPDefs::SECONDARIES::VFIL: return "VFIL";
		case OMP::OMPDefs::SECONDARIES::YAW: return "YAW";
		case OMP::OMPDefs::SECONDARIES::PHA: return "PHA";
		case OMP::OMPDefs::SECONDARIES::WEDG: return "WEDG";
		default: return "NOSEC";
		}
		return "NOSEC";
	}

	void OrbitalManeuverProcessor::PrintManeuverEvaluationTable()
	{
		OutputPrint.clear();

		std::vector<std::string> outarray;

		char Buffer[512], Buffer2[128], Buffer3[128], Buffer4[128];
		double hh, mm, ss;

		outarray.push_back("                MANEUVER EVALUATION TABLE");
		outarray.push_back("");
		outarray.push_back("                MCT: " + OMPMCTFile);
		outarray.push_back("");
		sprintf_s(Buffer, "                GMTR : %04d:%03d:%02d:%02d:%06.3f", sesconst.Year, sesconst.DayOfYear, sesconst.Hours, sesconst.Minutes, sesconst.launchdateSec);
		outarray.push_back(Buffer);
		outarray.push_back("");
		outarray.push_back("            TIME            COMMENT");
		OrbMech::MET2String(Buffer2, ManeuverEvaluationTable.GMT_C - sesconst.GMTLO);
		sprintf_s(Buffer, "CHASER   %s   %s", Buffer2, OMPChaserFile.c_str());
		outarray.push_back(Buffer);
		OrbMech::MET2String(Buffer2, ManeuverEvaluationTable.GMT_T - sesconst.GMTLO);
		sprintf_s(Buffer, "TARGET   %s   %s", Buffer2, OMPTargetFile.c_str());
		outarray.push_back(Buffer);
		outarray.push_back("");
		sprintf_s(Buffer, "  CHASER  DVtot = %7.2lf  DVx = %7.2lf  DVy = %7.2lf  DVz = %7.2lf", length(ManeuverEvaluationTable.dv_C),
			ManeuverEvaluationTable.dv_C.x, ManeuverEvaluationTable.dv_C.y, ManeuverEvaluationTable.dv_C.z);
		outarray.push_back(Buffer);

		sprintf_s(Buffer, "  TARGET  DVtot = %7.2lf  DVx = %7.2lf  DVy = %7.2lf  DVz = %7.2lf", length(ManeuverEvaluationTable.dv_T),
			ManeuverEvaluationTable.dv_T.x, ManeuverEvaluationTable.dv_T.y, ManeuverEvaluationTable.dv_T.z);
		outarray.push_back(Buffer);

		outarray.push_back(" ------------------------------------------------------------------------------ ");
		outarray.push_back("| MNVR  NAME |      GMTIG  IMP  |   DVX   |    HA   |    RANGE   |       Y     |");
		outarray.push_back("|  COMMENT   |      METIG       |   DVY   |    HP   |    PHASE   |      YDOT   |");
		outarray.push_back("|  DVMAG     |       DT         |   DVZ   |    DH   | Noon/Mid - |    SR/SS -  |");

		for (unsigned i = 0; i < ManeuverEvaluationTable.Maneuvers.size(); i++)
		{
			outarray.push_back("|------------+------------------+---------+---------+------------+-------------|");

			OrbMech::GMT2String(Buffer2, ManeuverEvaluationTable.Maneuvers[i].GMTIG, sesconst.DayOfYear);
			sprintf_s(Buffer, 512, "| %2d %-8.8s| %s | %7.2lf | %7.2lf | %10.4lf | %11.1lf |", i + 1, ManeuverEvaluationTable.Maneuvers[i].type.c_str(), Buffer2, ManeuverEvaluationTable.Maneuvers[i].DV.x,
				ManeuverEvaluationTable.Maneuvers[i].HA, ManeuverEvaluationTable.Maneuvers[i].RANGE, ManeuverEvaluationTable.Maneuvers[i].Y);
			outarray.push_back(Buffer);

			OrbMech::MET2String(Buffer2, ManeuverEvaluationTable.Maneuvers[i].METIG);
			sprintf_s(Buffer, 512, "| %-10.10s | %s | %7.2lf | %7.2lf | %10.4lf | %11.1lf |", ManeuverEvaluationTable.Maneuvers[i].name.c_str(), Buffer2, ManeuverEvaluationTable.Maneuvers[i].DV.y,
				ManeuverEvaluationTable.Maneuvers[i].HP, ManeuverEvaluationTable.Maneuvers[i].PHASE, ManeuverEvaluationTable.Maneuvers[i].Ydot);
			outarray.push_back(Buffer);

			OrbMech::MET2String(Buffer2, ManeuverEvaluationTable.Maneuvers[i].DT);

			OrbMech::SS2HHMMSS(ManeuverEvaluationTable.Maneuvers[i].TTN, hh, mm, ss);
			if (ManeuverEvaluationTable.Maneuvers[i].noon)
			{
				sprintf_s(Buffer3, "N-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer3, "M-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			OrbMech::SS2HHMMSS(ManeuverEvaluationTable.Maneuvers[i].TTS, hh, mm, ss);
			if (ManeuverEvaluationTable.Maneuvers[i].sunrise)
			{
				sprintf_s(Buffer4, "SR-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer4, "SS-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}

			sprintf_s(Buffer, 512, "| %6.1lf     | %s | %7.2lf | %7.2lf | %s | %s |", ManeuverEvaluationTable.Maneuvers[i].DVMag, Buffer2, ManeuverEvaluationTable.Maneuvers[i].DV.z,
				ManeuverEvaluationTable.Maneuvers[i].DH, Buffer3, Buffer4);

			outarray.push_back(Buffer);
		}

		outarray.push_back(" ------------------------------------------------------------------------------ ");

		OutputPrint.push_back(outarray);
	}
}
