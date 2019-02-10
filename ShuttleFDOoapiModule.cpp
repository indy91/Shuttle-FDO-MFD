/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO Module

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

#define STRICT
#define ORBITER_MODULE

#include "orbitersdk.h"
#include "ShuttleFDOMFD.h"
#include "ShuttleFDOoapiModule.h"

//
// ==============================================================
// Pointers to Global variables
// ==============================================================
//

extern ShuttleFDOoapiModule *g_coreMod;
extern ShuttleFDOCore *GCoreData[32];
extern OBJHANDLE GCoreVessel[32];
extern int nGutsUsed;
extern int g_MFDmode;

//
// ==============================================================
// Orbiter DLL API interface
// ==============================================================
//

DLLCLBK void InitModule(HINSTANCE hDLL) {          // Called by Orbiter when module selected in the Launchpad
	static char *name = "Shuttle FDO MFD";   // MFD mode name
	MFDMODESPECEX spec;
	spec.name = name;
	spec.key = OAPI_KEY_T;                // MFD mode selection key
	spec.context = NULL;
	spec.msgproc = ShuttleFDOoapiModule::MsgProc;  // MFD mode callback function

	// Register the new MFD mode with Orbiter
	g_MFDmode = oapiRegisterMFDMode(spec);
	oapiRegisterModule(g_coreMod);                    // Register this whole module with Orbiter
	nGutsUsed = 0;
}

DLLCLBK void ExitModule(HINSTANCE hDLL) {          // Called by Orbiter when module deselected in the Launchpad
	oapiUnregisterMFDMode(g_MFDmode);                // Unregister use as an MFD. Note - don't kill the g_coreMod module (Orbiter does this)
	g_coreMod = NULL;
	nGutsUsed = 0;
}

int ShuttleFDOoapiModule::MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam) {  // Message parser, handling MFD open requests
	switch (msg) {
	case OAPI_MSG_MFD_OPENED:
		return (int)(new ShuttleFDOMFD(LOWORD(wparam), HIWORD(wparam), (VESSEL*)lparam, mfd));    // Open an ephemeral instance each time we make a new OMP MFD, plus F8, etc/ 
	}
	return 0;
}

ShuttleFDOoapiModule::ShuttleFDOoapiModule(HINSTANCE hDLL) : oapi::Module(hDLL) {}
ShuttleFDOoapiModule::~ShuttleFDOoapiModule() {}

void ShuttleFDOoapiModule::clbkSimulationStart(RenderMode mode) {}

void ShuttleFDOoapiModule::clbkSimulationEnd() {                                      // When we exit sim back to Launchpad, make sure we tidy things up properly
	for (int i = 0;i < nGutsUsed;i++) {
		delete GCoreData[i];
		GCoreVessel[i] = NULL;
	}
	nGutsUsed = 0;
	return;
}
void ShuttleFDOoapiModule::clbkPreStep(double simt, double simdt, double mjd) {      // Called on each iteration of the calc engine (more often than the MFD Update
	for (int i = 0;i < nGutsUsed;i++) {
		//if (GCoreData[i]->isInit)GCoreData[i]->MinorCycle(simt, simdt);
	}
	return;
}

void ShuttleFDOoapiModule::clbkPostStep(double simt, double simdt, double mjd) {}

void ShuttleFDOoapiModule::clbkDeleteVessel(OBJHANDLE hVessel) {                     // Tidy up when a vessel is deleted (stops clbkPreStep calling a dead vessel)
	for (int i = 0;i < nGutsUsed;i++) {
		if (GCoreVessel[i] == hVessel) {
			delete GCoreData[i];
			for (int j = i; j < nGutsUsed - 1; j++) {
				GCoreVessel[j] = GCoreVessel[j + 1];
				GCoreData[j] = GCoreData[j + 1];
			}
			nGutsUsed--;
			break;
		}
	}
}