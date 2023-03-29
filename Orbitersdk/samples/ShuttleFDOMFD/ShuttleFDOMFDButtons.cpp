/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Spaceflight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD Buttons

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

#include "MFDButtonPage.hpp"
#include "ShuttleFDOMFDButtons.h"
#include "ShuttleFDOMFD.h"

ShuttleFDOMFDButtons::ShuttleFDOMFDButtons()
{
	static const MFDBUTTONMENU mnu0[] =
	{
		{ "Config Menu", 0, 'S' },
		{ "Launch Window Processor", 0, 'L' },
		{ "Constraints Page", 0, 'C' },
		{ "Evaluation Page", 0, 'E' },
		{ "Transfer Page", 0, 'M' },
		{ "Detailed Maneuver Table" , 0, 'D' },

		{ "Deorb Opport", 0, 'B' },
		{ "Deorb Plan", 0, 'T' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
	};

	RegisterPage(mnu0, sizeof(mnu0) / sizeof(MFDBUTTONMENU));

	RegisterFunction("CFG", OAPI_KEY_S, &ShuttleFDOMFD::menuSetConfigurationMenu);
	RegisterFunction("LWP", OAPI_KEY_A, &ShuttleFDOMFD::menuSetLWPPage);
	RegisterFunction("MCT", OAPI_KEY_C, &ShuttleFDOMFD::menuSetMCTPage);
	RegisterFunction("MET", OAPI_KEY_E, &ShuttleFDOMFD::menuSetMETPage);
	RegisterFunction("MTT", OAPI_KEY_M, &ShuttleFDOMFD::menuSetMTTPage);
	RegisterFunction("DMT", OAPI_KEY_D, &ShuttleFDOMFD::menuSetDMTPage);

	RegisterFunction("DOP", OAPI_KEY_B, &ShuttleFDOMFD::menuSetDOPSPage);
	RegisterFunction("DMP", OAPI_KEY_T, &ShuttleFDOMFD::menuSetDMPPage);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_I, &ShuttleFDOMFD::menuVoid);


	static const MFDBUTTONMENU mnu1[] =
	{
		{ "Add Maneuver", 0, 'A' },
		{ "Add Threshold", 0, 'T' },
		{ "Add Secondary", 0, 'S' },
		{ "Delete maneuver", 0, 'D' },
		{ "Modify maneuver", 0, 'M' },
		{ "Delete secondary", 0, 'E' },

		{ "Calculate plan", 0, 'C' },
		{ "Insert maneuver", 0, 'I' },
		{ "Modify secondary", 0, 'F' },
		{ "Scroll up", 0, 'G' },
		{ "Scroll down", 0, 'H' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu1, sizeof(mnu1) / sizeof(MFDBUTTONMENU));

	RegisterFunction("ADD", OAPI_KEY_A, &ShuttleFDOMFD::menuAddOMPManeuver);
	RegisterFunction("THR", OAPI_KEY_T, &ShuttleFDOMFD::menuAddOMPThreshold);
	RegisterFunction("SEC", OAPI_KEY_S, &ShuttleFDOMFD::menuAddOMPSecondary);
	RegisterFunction("DEL", OAPI_KEY_D, &ShuttleFDOMFD::menuDeleteOMPManeuver);
	RegisterFunction("MOD", OAPI_KEY_M, &ShuttleFDOMFD::menuModifyOMPManeuver);
	RegisterFunction("DES", OAPI_KEY_E, &ShuttleFDOMFD::menuDeleteOMPSecondary);

	RegisterFunction("CLC", OAPI_KEY_C, &ShuttleFDOMFD::menuCalculateOMPPlan);
	RegisterFunction("INS", OAPI_KEY_I, &ShuttleFDOMFD::menuInsertOMPManeuver);
	RegisterFunction("MOS", OAPI_KEY_F, &ShuttleFDOMFD::menuModifySecondary);
	RegisterFunction("UP", OAPI_KEY_G, &ShuttleFDOMFD::menuScrollMCTUp);
	RegisterFunction("DN", OAPI_KEY_H, &ShuttleFDOMFD::menuScrollMCTDown);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu2[] =
	{
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },

		{ "Transfer to MTT", 0, 'T' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Scroll up", 0, 'G' },
		{ "Scroll down", 0, 'H' },
		{ "Back to menu", 0, ' ' },
	};

	RegisterPage(mnu2, sizeof(mnu2) / sizeof(MFDBUTTONMENU));

	RegisterFunction("", OAPI_KEY_C, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_E, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_B, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("TRA", OAPI_KEY_T, &ShuttleFDOMFD::menuTransferToMTT);
	RegisterFunction("", OAPI_KEY_D, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("UP", OAPI_KEY_G, &ShuttleFDOMFD::menuScrollMETUp);
	RegisterFunction("DN", OAPI_KEY_H, &ShuttleFDOMFD::menuScrollMETDown);
	RegisterFunction("BCK", OAPI_KEY_I, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu3[] =
	{
		{ "Set target", 0, 'T' },
		{ "Set launch site", 0, 'L' },
		{ "Set LS coordinates", 0, 'A' },
		{ "Launch direction", 0, 'D' },
		{ "Max yaw steering", 0, 'Y' },
		{ "Back to menu", 0, 'B' },

		{ "Set powered arc", 0, 'P' },
		{ "Set powered time", 0, 'F' },
		{ "Set insertion rad", 0, 'R' },
		{ "Set insertion vel", 0, 'V' },
		{ "Set insertion FPA", 0, 'H' },
		{ "Next page", 0, 'N' },
	};

	RegisterPage(mnu3, sizeof(mnu3) / sizeof(MFDBUTTONMENU));

	RegisterFunction("TGT", OAPI_KEY_T, &ShuttleFDOMFD::set_target);
	RegisterFunction("LS", OAPI_KEY_L, &ShuttleFDOMFD::menuLWPSetLS);
	RegisterFunction("LAT", OAPI_KEY_A, &ShuttleFDOMFD::menuLWPSetLSLatLng);
	RegisterFunction("AZI", OAPI_KEY_D, &ShuttleFDOMFD::menuLWPLaunchAzimuthDirectionFlag);
	RegisterFunction("YS", OAPI_KEY_Y, &ShuttleFDOMFD::menuLWPSetYS);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);

	RegisterFunction("PFA", OAPI_KEY_P, &ShuttleFDOMFD::menuLWPSetPFA);
	RegisterFunction("PFT", OAPI_KEY_F, &ShuttleFDOMFD::menuLWPSetPFT);
	RegisterFunction("RAD", OAPI_KEY_R, &ShuttleFDOMFD::menuLWPSetRAD);
	RegisterFunction("VEL", OAPI_KEY_V, &ShuttleFDOMFD::menuLWPSetVEL);
	RegisterFunction("FPA", OAPI_KEY_H, &ShuttleFDOMFD::menuLWPSetFPA);
	RegisterFunction("NXT", OAPI_KEY_N, &ShuttleFDOMFD::menuSetLWPPage2);


	static const MFDBUTTONMENU mnu4[] =
	{
		{ "Choose slot", 0, 'C' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },

		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, ' ' },
	};

	RegisterPage(mnu4, sizeof(mnu4) / sizeof(MFDBUTTONMENU));

	RegisterFunction("SLO", OAPI_KEY_C, &ShuttleFDOMFD::menuMTTChangeSlot);
	RegisterFunction("", OAPI_KEY_E, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_B, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("EXE", OAPI_KEY_D, &ShuttleFDOMFD::menuExecuteMTT);
	RegisterFunction("", OAPI_KEY_T, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_I, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu5[] =
	{
		{ "Choose maneuver", 0, 'M' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },

		{ "Calculate DMT", 0, 'C' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu5, sizeof(mnu5) / sizeof(MFDBUTTONMENU));

	RegisterFunction("MNV", OAPI_KEY_M, &ShuttleFDOMFD::menuDMTChooseManeuver);
	RegisterFunction("", OAPI_KEY_E, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_D, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_I, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("CLC", OAPI_KEY_C, &ShuttleFDOMFD::menuCalcDMT);
	RegisterFunction("", OAPI_KEY_T, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu6[] =
	{
		{ "Set chaser", 0, 'M' },
		{ "Set target", 0, 'T' },
		{ "Set liftoff day", 0, 'S' },
		{ "Gravity option", 0, 'G' },
		{ "Save to file", 0, 'A' },
		{ "Load from file", 0, 'L' },

		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Set launch time", 0, 'F' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu6, sizeof(mnu6) / sizeof(MFDBUTTONMENU));

	RegisterFunction("CHA", OAPI_KEY_M, &ShuttleFDOMFD::set_shuttle);
	RegisterFunction("TGT", OAPI_KEY_T, &ShuttleFDOMFD::set_target);
	RegisterFunction("DLO", OAPI_KEY_S, &ShuttleFDOMFD::menuSetLaunchDay);
	RegisterFunction("GRA", OAPI_KEY_G, &ShuttleFDOMFD::menuCycleGravityOption);
	RegisterFunction("SAV", OAPI_KEY_A, &ShuttleFDOMFD::menuSaveState);
	RegisterFunction("LOA", OAPI_KEY_L, &ShuttleFDOMFD::menuLoadState);

	RegisterFunction("", OAPI_KEY_I, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_C, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("TLO", OAPI_KEY_F, &ShuttleFDOMFD::menuSetLaunchTime);
	RegisterFunction("", OAPI_KEY_D, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu7[] =
	{
		{ "DT ET SEP", 0, 'D' },
		{ "DV ET SEP", 0, 'V' },
		{ "DT MPS Dump", 0, 'T' },
		{ "DV MPS Dump", 0, 'M' },
		{ "", 0, ' ' },
		{ "Pre OMS-2 weight", 0, 'W' },

		{ "Set DTOPT", 0, 'O' },
		{ "Set window opening", 0, 'U' },
		{ "Set window closing", 0, 'C' },
		{ "Set phase flag", 0, 'P' },
		{ "Set wrap counter", 0, 'R' },
		{ "Next page", 0, 'N' },
	};

	RegisterPage(mnu7, sizeof(mnu7) / sizeof(MFDBUTTONMENU));

	RegisterFunction("DTE", OAPI_KEY_D, &ShuttleFDOMFD::menuLWPSetDTETSEP);
	RegisterFunction("DVE", OAPI_KEY_V, &ShuttleFDOMFD::menuLWPSetDVETSEP);
	RegisterFunction("DTM", OAPI_KEY_T, &ShuttleFDOMFD::menuLWPSetDTMPS);
	RegisterFunction("DVM", OAPI_KEY_M, &ShuttleFDOMFD::menuLWPSetDVMPS);
	RegisterFunction("", OAPI_KEY_L, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("WT", OAPI_KEY_W, &ShuttleFDOMFD::menuLWPSetWT);

	RegisterFunction("OPT", OAPI_KEY_O, &ShuttleFDOMFD::menuLWPSetDTOPT);
	RegisterFunction("DTO", OAPI_KEY_U, &ShuttleFDOMFD::menuLWPSetDTO);
	RegisterFunction("DTC", OAPI_KEY_C, &ShuttleFDOMFD::menuLWPSetDTC);
	RegisterFunction("PHA", OAPI_KEY_P, &ShuttleFDOMFD::menuLWPSetPHASEFLAG);
	RegisterFunction("WRA", OAPI_KEY_R, &ShuttleFDOMFD::menuLWPSetWRAPFLAG);
	RegisterFunction("NXT", OAPI_KEY_N, &ShuttleFDOMFD::menuSetLWPPage3);


	static const MFDBUTTONMENU mnu8[] =
	{
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },

		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu8, sizeof(mnu8) / sizeof(MFDBUTTONMENU));

	RegisterFunction("", OAPI_KEY_M, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_E, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_D, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("CLC", OAPI_KEY_C, &ShuttleFDOMFD::menuCalcLaunchTime);
	RegisterFunction("", OAPI_KEY_T, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("LTP", OAPI_KEY_H, &ShuttleFDOMFD::menuSetLTPPage);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu9[] =
	{
		{ "Initial rev counter", 0, 'I' },
		{ "MET for start of search", 0, 'S' },
		{ "MET for end of search", 0, 'E' },
		{ "Maximum crossrange", 0, 'X' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },

		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu9, sizeof(mnu9) / sizeof(MFDBUTTONMENU));

	RegisterFunction("REV", OAPI_KEY_I, &ShuttleFDOMFD::menuDOPSSetRev);
	RegisterFunction("STA", OAPI_KEY_S, &ShuttleFDOMFD::menuDOPSSetGETS);
	RegisterFunction("END", OAPI_KEY_E, &ShuttleFDOMFD::menuDOPSSetGETF);
	RegisterFunction("XR", OAPI_KEY_X, &ShuttleFDOMFD::menuDOPSSetMaxXRNG);
	RegisterFunction("", OAPI_KEY_I, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("CLC", OAPI_KEY_C, &ShuttleFDOMFD::menuCalcDeorbitOpportunities);
	RegisterFunction("", OAPI_KEY_T, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu10[] =
	{
		{ "Fixed or free TIG", 0, 'F' },
		{ "Input TIG or threshold", 0, 'T' },
		{ "Primary thruster", 0, 'P' },
		{ "Backup thruster", 0, 'B' },
		{ "Landing site", 0, 'L' },
		{ "", 0, ' ' },

		{ "Calculate", 0, 'C' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu10, sizeof(mnu10) / sizeof(MFDBUTTONMENU));

	RegisterFunction("ITI", OAPI_KEY_F, &ShuttleFDOMFD::menuDMPCycleTIGOption);
	RegisterFunction("TIG", OAPI_KEY_S, &ShuttleFDOMFD::menuDMPInputTIG);
	RegisterFunction("TPR", OAPI_KEY_P, &ShuttleFDOMFD::menuDMPCyclePrimaryThruster);
	RegisterFunction("TBU", OAPI_KEY_T, &ShuttleFDOMFD::menuDMPCycleBackupThruster);
	RegisterFunction("SIT", OAPI_KEY_L, &ShuttleFDOMFD::menuDMPLandingSite);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("CLC", OAPI_KEY_C, &ShuttleFDOMFD::menuCalcDMP);
	RegisterFunction("", OAPI_KEY_T, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu11[] =
	{
		{ "Input launch time", 0, 'M' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },

		{ "Calculate LTP", 0, 'C' },
		{ "Export solution", 0, 'T' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu11, sizeof(mnu11) / sizeof(MFDBUTTONMENU));

	RegisterFunction("TLO", OAPI_KEY_M, &ShuttleFDOMFD::menuSetLTPLaunchTime);
	RegisterFunction("", OAPI_KEY_E, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_D, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("CLC", OAPI_KEY_C, &ShuttleFDOMFD::menuCalcLTP);
	RegisterFunction("EXP", OAPI_KEY_T, &ShuttleFDOMFD::menuExportLTP);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);
}

bool ShuttleFDOMFDButtons::SearchForKeysInOtherPages() const
{
	return false;
}