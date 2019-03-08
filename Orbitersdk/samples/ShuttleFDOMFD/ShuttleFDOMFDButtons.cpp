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
		{ "Executive Menu", 0, 'S' },
		{ "Constraints Page", 0, 'C' },
		{ "Evaluation Page", 0, 'E' },
		{ "Transfer Page", 0, 'M' },
		{ "Detailed Maneuver Table" , 0, 'D' },
		{ "", 0, ' ' },

		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
	};

	RegisterPage(mnu0, sizeof(mnu0) / sizeof(MFDBUTTONMENU));

	RegisterFunction("EXE", OAPI_KEY_S, &ShuttleFDOMFD::menuSetOMPExeMenu);
	RegisterFunction("MCT", OAPI_KEY_C, &ShuttleFDOMFD::menuSetMCTPage);
	RegisterFunction("MET", OAPI_KEY_E, &ShuttleFDOMFD::menuSetMETPage);
	RegisterFunction("MTT", OAPI_KEY_M, &ShuttleFDOMFD::menuSetMTTPage);
	RegisterFunction("DMT", OAPI_KEY_D, &ShuttleFDOMFD::menuSetDMTPage);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("", OAPI_KEY_B, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_T, &ShuttleFDOMFD::menuVoid);
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
		{ "Calculate Launch Time", 0, 'C' },
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

	RegisterPage(mnu3, sizeof(mnu3) / sizeof(MFDBUTTONMENU));

	RegisterFunction("LAU", OAPI_KEY_C, &ShuttleFDOMFD::menuCalcLaunchTime);
	RegisterFunction("", OAPI_KEY_E, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_B, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &ShuttleFDOMFD::menuVoid);

	RegisterFunction("", OAPI_KEY_D, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_T, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_I, &ShuttleFDOMFD::menuSetMainMenu);


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
		{ "Set liftoff time", 0, 'S' },
		{ "Gravity option", 0, 'G' },
		{ "Save to file", 0, 'A' },
		{ "Load from file", 0, 'L' },

		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu6, sizeof(mnu6) / sizeof(MFDBUTTONMENU));

	RegisterFunction("CHA", OAPI_KEY_M, &ShuttleFDOMFD::set_shuttle);
	RegisterFunction("TGT", OAPI_KEY_T, &ShuttleFDOMFD::set_target);
	RegisterFunction("TLO", OAPI_KEY_S, &ShuttleFDOMFD::menuSetLiftoffTime);
	RegisterFunction("GRA", OAPI_KEY_G, &ShuttleFDOMFD::menuCycleGravityOption);
	RegisterFunction("SAV", OAPI_KEY_A, &ShuttleFDOMFD::menuSaveState);
	RegisterFunction("LOA", OAPI_KEY_L, &ShuttleFDOMFD::menuLoadState);

	RegisterFunction("", OAPI_KEY_I, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_C, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_D, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &ShuttleFDOMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &ShuttleFDOMFD::menuSetMainMenu);
}

bool ShuttleFDOMFDButtons::SearchForKeysInOtherPages() const
{
	return false;
}