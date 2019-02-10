/****************************************************************************
  This file is part of OMP MFD for Orbiter Spaceflight Simulator
  Copyright (C) 2019 Niklas Beug

  OMP MFD Buttons

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
#include "OMPMFDButtons.h"
#include "OMPMFD.h"

OMPMFDButtons::OMPMFDButtons()
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

	RegisterFunction("EXE", OAPI_KEY_S, &OMPMFD::menuSetOMPExeMenu);
	RegisterFunction("MCT", OAPI_KEY_C, &OMPMFD::menuSetMCTPage);
	RegisterFunction("MET", OAPI_KEY_E, &OMPMFD::menuSetMETPage);
	RegisterFunction("MTT", OAPI_KEY_M, &OMPMFD::menuSetMTTPage);
	RegisterFunction("DMT", OAPI_KEY_D, &OMPMFD::menuSetDMTPage);
	RegisterFunction("", OAPI_KEY_A, &OMPMFD::menuVoid);

	RegisterFunction("", OAPI_KEY_B, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_T, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_I, &OMPMFD::menuVoid);


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
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, 'B' },
	};

	RegisterPage(mnu1, sizeof(mnu1) / sizeof(MFDBUTTONMENU));

	RegisterFunction("ADD", OAPI_KEY_A, &OMPMFD::menuAddOMPManeuver);
	RegisterFunction("THR", OAPI_KEY_T, &OMPMFD::menuAddOMPThreshold);
	RegisterFunction("SEC", OAPI_KEY_S, &OMPMFD::menuAddOMPSecondary);
	RegisterFunction("DEL", OAPI_KEY_D, &OMPMFD::menuDeleteOMPManeuver);
	RegisterFunction("MOD", OAPI_KEY_M, &OMPMFD::menuModifyOMPManeuver);
	RegisterFunction("DES", OAPI_KEY_E, &OMPMFD::menuDeleteOMPSecondary);

	RegisterFunction("CLC", OAPI_KEY_C, &OMPMFD::menuCalculateOMPPlan);
	RegisterFunction("INS", OAPI_KEY_I, &OMPMFD::menuInsertOMPManeuver);
	RegisterFunction("", OAPI_KEY_F, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &OMPMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &OMPMFD::menuSetMainMenu);


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
		{ "", 0, ' ' },
		{ "", 0, ' ' },
		{ "Back to menu", 0, ' ' },
	};

	RegisterPage(mnu2, sizeof(mnu2) / sizeof(MFDBUTTONMENU));

	RegisterFunction("", OAPI_KEY_C, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_E, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_B, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &OMPMFD::menuVoid);

	RegisterFunction("TRA", OAPI_KEY_T, &OMPMFD::menuTransferToMTT);
	RegisterFunction("", OAPI_KEY_D, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &OMPMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_I, &OMPMFD::menuSetMainMenu);


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

	RegisterFunction("LAU", OAPI_KEY_C, &OMPMFD::menuCalcLaunchTime);
	RegisterFunction("", OAPI_KEY_E, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_B, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &OMPMFD::menuVoid);

	RegisterFunction("", OAPI_KEY_D, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_T, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &OMPMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_I, &OMPMFD::menuSetMainMenu);


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

	RegisterFunction("SLO", OAPI_KEY_C, &OMPMFD::menuMTTChangeSlot);
	RegisterFunction("", OAPI_KEY_E, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_M, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_B, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &OMPMFD::menuVoid);

	RegisterFunction("EXE", OAPI_KEY_D, &OMPMFD::menuExecuteMTT);
	RegisterFunction("", OAPI_KEY_T, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &OMPMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_I, &OMPMFD::menuSetMainMenu);


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

	RegisterFunction("MNV", OAPI_KEY_M, &OMPMFD::menuDMTChooseManeuver);
	RegisterFunction("", OAPI_KEY_E, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_S, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_D, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_I, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &OMPMFD::menuVoid);

	RegisterFunction("CLC", OAPI_KEY_C, &OMPMFD::menuCalcDMT);
	RegisterFunction("", OAPI_KEY_T, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_G, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &OMPMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &OMPMFD::menuSetMainMenu);


	static const MFDBUTTONMENU mnu6[] =
	{
		{ "", 0, ' ' },
		{ "Set target", 0, 'T' },
		{ "Set liftoff time", 0, 'S' },
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

	RegisterPage(mnu6, sizeof(mnu6) / sizeof(MFDBUTTONMENU));

	RegisterFunction("", OAPI_KEY_M, &OMPMFD::menuVoid);
	RegisterFunction("TGT", OAPI_KEY_T, &OMPMFD::set_target);
	RegisterFunction("TLO", OAPI_KEY_S, &OMPMFD::menuSetLiftoffTime);
	RegisterFunction("GRA", OAPI_KEY_G, &OMPMFD::menuCycleGravityOption);
	RegisterFunction("", OAPI_KEY_I, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_A, &OMPMFD::menuVoid);

	RegisterFunction("", OAPI_KEY_C, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_E, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_F, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_D, &OMPMFD::menuVoid);
	RegisterFunction("", OAPI_KEY_H, &OMPMFD::menuVoid);
	RegisterFunction("BCK", OAPI_KEY_B, &OMPMFD::menuSetMainMenu);
}

bool OMPMFDButtons::SearchForKeysInOtherPages() const
{
	return false;
}