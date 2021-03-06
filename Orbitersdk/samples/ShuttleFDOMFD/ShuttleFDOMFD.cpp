/****************************************************************************
  This file is part of the Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD

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

#include "windows.h"
#include <iostream>
#include <fstream>
#include <string>
#include "Orbitersdk.h"
#include "papi.h"
#include "OrbMech.h"
#include "ShuttleFDOCore.h"
#include "ShuttleFDOMFD.h"
#include "ShuttleFDOoapiModule.h"

// ==============================================================
// Global variables

ShuttleFDOoapiModule *g_coreMod;
int g_MFDmode; // identifier for new MFD mode
ShuttleFDOCore *GCoreData[32];
OBJHANDLE GCoreVessel[32];
int nGutsUsed;

// ==============================================================
// MFD class implementation

// Constructor
ShuttleFDOMFD::ShuttleFDOMFD(DWORD w, DWORD h, VESSEL *v, UINT im)
: MFD2 (w, h, v)
{
	font = oapiCreateFont(w / 20, true, "Courier", FONT_NORMAL, 0);
	font2 = oapiCreateFont(w / 30, true, "Courier", FONT_NORMAL, 0);
	// Add MFD initialisation here
	screen = 0;
	MTTFlag = false;
	MCTScroll = 0;
	METScroll = 0;
	bool found = false;
	for (int i = 0; i < nGutsUsed; i++) {
		if (i == 32) {
			i = 0;
			GCoreVessel[i] = v;
		}
		if (GCoreVessel[i] == v)
		{
			found = true;
			G = GCoreData[i];
		}
	}
	if (!found)
	{
		GCoreData[nGutsUsed] = new ShuttleFDOCore(v);
		G = GCoreData[nGutsUsed];
		GCoreVessel[nGutsUsed] = v;
		nGutsUsed++;
	}
}

// Destructor
ShuttleFDOMFD::~ShuttleFDOMFD()
{
	oapiReleaseFont(font);
	oapiReleaseFont(font2);
	// Add MFD cleanup code here
}

// Return button labels
char *ShuttleFDOMFD::ButtonLabel (int bt)
{
	// The labels for the two buttons used by our MFD mode
	return coreButtons.ButtonLabel(bt);
}

// Return button menus
int ShuttleFDOMFD::ButtonMenu (const MFDBUTTONMENU **menu) const
{
	// The menu descriptions for the two buttons
	return coreButtons.ButtonMenu(menu);
}

bool ShuttleFDOMFD::ConsumeButton(int bt, int event)
{
	return coreButtons.ConsumeButton(this, bt, event);
}

bool ShuttleFDOMFD::ConsumeKeyBuffered(DWORD key)
{
	return coreButtons.ConsumeKeyBuffered(this, key);
}

/*void ShuttleFDOMFD::WriteStatus(FILEHANDLE scn) const
{

}

void ShuttleFDOMFD::ReadStatus(FILEHANDLE scn)
{

}*/

// Repaint the MFD
bool ShuttleFDOMFD::Update(oapi::Sketchpad *skp)
{
	Title(skp, "Shuttle FDO MFD");
	// Draws the MFD title

	skp->SetFont(font);
	//skp->SetTextAlign (oapi::Sketchpad::CENTER, oapi::Sketchpad::BASELINE);
	//skp->SetTextColor (0x00FFFF);

	// Add MFD display routines here.
	// Use the device context (hDC) for Windows GDI paint functions.

	if (screen == 0)
	{
		skp->Text(1 * W / 8, 2 * H / 14, "Executive Menu", 14);
		skp->Text(1 * W / 8, 4 * H / 14, "Launch Window Processor", 23);
		skp->Text(1 * W / 8, 6 * H / 14, "Maneuver Constraints Table", 27);
		skp->Text(1 * W / 8, 8 * H / 14, "Maneuver Evaluation Table", 25);
		skp->Text(1 * W / 8, 10 * H / 14, "Maneuver Transfer Table", 23);
		skp->Text(1 * W / 8, 12 * H / 14, "Detailed Maneuver Table", 23);
	}
	else if (screen == 1)
	{
		unsigned ii;

		skp->SetFont(font2);

		sprintf_s(Buffer, "MANEUVER");
		skp->Text(1 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "THRESHOLD");
		skp->Text(8 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "SECONDARIES");
		skp->Text(17 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));

		if (G->subThreadStatus)
		{
			sprintf_s(Buffer, "Iterating...");
			skp->Text(2 * W / 32, 31 * H / 32, Buffer, strlen(Buffer));
		}
		else if (G->OMPErrorCode)
		{
			GetOMPError(Buffer, G->OMPErrorCode);
			skp->Text(2 * W / 32, 31 * H / 32, Buffer, strlen(Buffer));
		}

		for (unsigned i = MCTScroll;i < G->ManeuverConstraintsTable.size();i++)
		{
			ii = i - MCTScroll;
			//MANEUVER
			sprintf_s(Buffer, "%d", i + 1);
			skp->Text(1 * W / 32, (ii * 3 + 4) * H / 32, Buffer, strlen(Buffer));

			GetOPMManeuverType(Buffer, G->ManeuverConstraintsTable[i].type);
			skp->Text(2 * W / 32, (ii * 3 + 4) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, 100, G->ManeuverConstraintsTable[i].name);
			skp->Text(1 * W / 32, (ii * 3 + 5) * H / 32, Buffer, strlen(Buffer));

			//THRESHOLD
			GetOPMManeuverThreshold(Buffer, G->ManeuverConstraintsTable[i].threshold);
			skp->Text(9 * W / 32, (ii * 3 + 4) * H / 32, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::CENTER);

			GetOPMManeuverThresholdTime(Buffer, G->ManeuverConstraintsTable[i].threshold, G->ManeuverConstraintsTable[i].thresh_num);
			skp->Text(9 * W / 32, (ii * 3 + 5) * H / 32, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			//SECONDARIES

			int k, l;

			for (unsigned j = 0;j < G->ManeuverConstraintsTable[i].secondaries.size();j++)
			{
				if (j >= 6)
				{
					k = j - 6;
					l = 1;
				}
				else if (j >= 3)
				{
					k = j - 3;
					l = 1;
				}
				else
				{
					k = j;
					l = 0;
				}
				GetOPMManeuverSecondary(Buffer, G->ManeuverConstraintsTable[i].secondaries[j].type, G->ManeuverConstraintsTable[i].secondaries[j].value);
				skp->Text((26 + k * 13) * W / 64, (ii * 3 + 5 + l) * H / 32, Buffer, strlen(Buffer));
			}

			//Only display 9 maneuvers at once
			if (i >= 8 + MCTScroll) break;
		}
	}
	else if (screen == 2)
	{
		unsigned ii;

		skp->SetFont(font2);

		double hh, mm, ss;

		sprintf_s(Buffer, "MNVR  NAME");
		skp->Text(1 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "COMMENT");
		skp->Text(2 * W / 32, 3 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DVMAG");
		skp->Text(2 * W / 32, 4 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "GMTIG  IMP");
		skp->Text(8 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "METIG");
		skp->Text(8 * W / 32, 3 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DT");
		skp->Text(9 * W / 32, 4 * H / 32, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::RIGHT);

		sprintf_s(Buffer, "DVX");
		skp->Text(16 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DVY");
		skp->Text(16 * W / 32, 3 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DVZ");
		skp->Text(16 * W / 32, 4 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "HA");
		skp->Text(18 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "HP");
		skp->Text(18 * W / 32, 3 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DH");
		skp->Text(18 * W / 32, 4 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "RANGE");
		skp->Text(24 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "PHASE");
		skp->Text(24 * W / 32, 3 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "Noon/Mid");
		skp->Text(24 * W / 32, 4 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "Y");
		skp->Text(29 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "YDOT");
		skp->Text(30 * W / 32, 3 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "SR/SS");
		skp->Text(29 * W / 32, 4 * H / 32, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::LEFT);

		if (G->subThreadStatus != 0) return true;

		for (unsigned i = METScroll;i < G->ManeuverEvaluationTable.size();i++)
		{
			ii = i - METScroll;

			sprintf_s(Buffer, "%d", i + 1);
			skp->Text(1 * W / 32, (ii * 5) * H / 48 + 5 * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, G->ManeuverEvaluationTable[i].type);
			skp->Text(3 * W / 32, (ii * 5) * H / 48 + 5 * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, G->ManeuverEvaluationTable[i].name);
			skp->Text(1 * W / 32, (ii * 5) * H / 48 + 6 * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable[i].DVMag);
			skp->Text(1 * W / 32, (ii * 5) * H / 48 + 7 * H / 32, Buffer, strlen(Buffer));

			GMT2String(Buffer, G->ManeuverEvaluationTable[i].GMTIG);
			skp->Text(6 * W / 32, (ii * 5) * H / 48 + 5 * H / 32, Buffer, strlen(Buffer));
			MET2String(Buffer, G->ManeuverEvaluationTable[i].METIG);
			skp->Text(6 * W / 32, (ii * 5) * H / 48 + 6 * H / 32, Buffer, strlen(Buffer));
			MET2String(Buffer, G->ManeuverEvaluationTable[i].DT);
			skp->Text(6 * W / 32, (ii * 5) * H / 48 + 7 * H / 32, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::RIGHT);

			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DV.x);
			skp->Text(16 * W / 32, (ii * 5) * H / 48 + 5 * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DV.y);
			skp->Text(16 * W / 32, (ii * 5) * H / 48 + 6 * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DV.z);
			skp->Text(16 * W / 32, (ii * 5) * H / 48 + 7 * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].HA);
			skp->Text(19 * W / 32, (ii * 5) * H / 48 + 5 * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].HP);
			skp->Text(19 * W / 32, (ii * 5) * H / 48 + 6 * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DH);
			skp->Text(19 * W / 32, (ii * 5) * H / 48 + 7 * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.4f", G->ManeuverEvaluationTable[i].RANGE);
			skp->Text(24 * W / 32, (ii * 5) * H / 48 + 5 * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.4f", G->ManeuverEvaluationTable[i].PHASE);
			skp->Text(24 * W / 32, (ii * 5) * H / 48 + 6 * H / 32, Buffer, strlen(Buffer));
			SS2HHMMSS(G->ManeuverEvaluationTable[i].TTN, hh, mm, ss);
			if (G->ManeuverEvaluationTable[i].noon)
			{
				sprintf_s(Buffer, "N-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer, "M-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			skp->Text(24 * W / 32, (ii * 5) * H / 48 + 7 * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable[i].Y);
			skp->Text(30 * W / 32, (ii * 5) * H / 48 + 5 * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable[i].Ydot);
			skp->Text(30 * W / 32, (ii * 5) * H / 48 + 6 * H / 32, Buffer, strlen(Buffer));
			SS2HHMMSS(G->ManeuverEvaluationTable[i].TTS, hh, mm, ss);
			if (G->ManeuverEvaluationTable[i].sunrise)
			{
				sprintf_s(Buffer, "SR-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer, "SS-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			skp->Text(30 * W / 32, (ii * 5) * H / 48 + 7 * H / 32, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			//Only display 8 maneuvers at once
			if (i >= 7 + METScroll) break;
		}
	}
	else if (screen == 3)
	{
		double mm, ss;

		if (G->target)
		{
			sprintf(Buffer, G->target->GetName());
			skp->Text(1 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));
		}

		if (G->LWP_LaunchSite == 1)
		{
			sprintf_s(Buffer, "39A");
			skp->Text(1 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));
		}
		else if (G->LWP_LaunchSite == 2)
		{
			sprintf_s(Buffer, "39B");
			skp->Text(1 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));
		}

		sprintf_s(Buffer, "%.3f�", G->LWP_Settings.LATLS*DEG);
		skp->Text(1 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f�", G->LWP_Settings.LONGLS*DEG);
		skp->Text(1 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f�", G->LWP_Settings.YSMAX*DEG);
		skp->Text(1 * W / 8, 10 * H / 14, Buffer, strlen(Buffer));


		sprintf_s(Buffer, "%.3f�", G->LWP_Settings.PFA*DEG);
		skp->Text(5 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));

		SS2MMSS(G->LWP_Settings.PFT, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(5 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f ft", G->LWP_Settings.RINS*MPS2FPS);
		skp->Text(5 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f fps", G->LWP_Settings.VINS*MPS2FPS);
		skp->Text(5 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f�", G->LWP_Settings.GAMINS*DEG);
		skp->Text(5 * W / 8, 10 * H / 14, Buffer, strlen(Buffer));
	}
	else if (screen == 4)
	{
		skp->SetFont(font2);

		if (MTTFlag)
		{
			sprintf(Buffer, "Transfer successful!");
			skp->Text(10 * W / 32, 23 * H / 32, Buffer, strlen(Buffer));
		}

		sprintf_s(Buffer, "MNVR");
		skp->Text(1 * W / 64, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "NAME");
		skp->Text(5 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "COMMENT");
		skp->Text(6 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "SLOT");
		skp->Text(11 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "THR");
		skp->Text(14 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "GUID");
		skp->Text(17 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "ITER");
		skp->Text(20 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "IMP");
		skp->Text(23 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "RREF");
		skp->Text(26 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "ROLL");
		skp->Text(29 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));

		for (unsigned i = 0;i < G->ManeuverTransferTable.size();i++)
		{
			sprintf_s(Buffer, "%d", i + 1);
			skp->Text(1 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, G->ManeuverTransferTable[i].NAME);
			skp->Text(3 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, G->ManeuverTransferTable[i].COMMENT);
			skp->Text(7 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%d", G->ManeuverTransferTable[i].SLOT);
			skp->Text(12 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			GetMTTThrusterType(Buffer, G->ManeuverTransferTable[i].thrusters);
			skp->Text(14 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			GetMTTGuidanceType(Buffer, G->ManeuverTransferTable[i].guid);
			skp->Text(17 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			if (G->ManeuverTransferTable[i].ITER)
			{
				sprintf_s(Buffer, "YES");
			}
			else
			{
				sprintf_s(Buffer, "NO");
			}
			skp->Text(20 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			if (G->ManeuverTransferTable[i].IMP)
			{
				sprintf_s(Buffer, "OPT");
			}
			else
			{
				sprintf_s(Buffer, "IMP");
			}
			skp->Text(23 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			if (G->ManeuverTransferTable[i].RREF)
			{
				sprintf_s(Buffer, "TVR");
			}
			else
			{
				sprintf_s(Buffer, "ADI");
			}
			skp->Text(26 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.0f", G->ManeuverTransferTable[i].ROLL*DEG);
			skp->Text(29 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
		}

		skp->SetTextAlign(oapi::Sketchpad::RIGHT);

		sprintf_s(Buffer, "SLOT");
		skp->Text(7 * W / 32, 25 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "THRUSTER");
		skp->Text(7 * W / 32, 26 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "GUIDANCE MODE");
		skp->Text(7 * W / 32, 27 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "ITERATE FLAG");
		skp->Text(7 * W / 32, 28 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "IMPULSIVE FLAG");
		skp->Text(7 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "LOCAL ROLL REF");
		skp->Text(7 * W / 32, 30 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "LOCAL ROLL");
		skp->Text(7 * W / 32, 31 * H / 32, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::LEFT);

		for (int i = 0;i < 10;i++)
		{
			sprintf_s(Buffer, "%d", G->MTTSlotData[i].SLOT);
			skp->Text((2 * i + 8) * W / 32, 25 * H / 32, Buffer, strlen(Buffer));
			GetMTTThrusterType(Buffer, G->MTTSlotData[i].thrusters);
			skp->Text((2 * i + 8) * W / 32, 26 * H / 32, Buffer, strlen(Buffer));
			GetMTTGuidanceType(Buffer, G->MTTSlotData[i].guid);
			skp->Text((2 * i + 8) * W / 32, 27 * H / 32, Buffer, strlen(Buffer));
			if (G->MTTSlotData[i].ITER)
			{
				sprintf_s(Buffer, "YES");
			}
			else
			{
				sprintf_s(Buffer, "NO");
			}
			skp->Text((2 * i + 8) * W / 32, 28 * H / 32, Buffer, strlen(Buffer));
			if (G->MTTSlotData[i].IMP)
			{
				sprintf_s(Buffer, "OPT");
			}
			else
			{
				sprintf_s(Buffer, "IMP");
			}
			skp->Text((2 * i + 8) * W / 32, 29 * H / 32, Buffer, strlen(Buffer));
			if (G->MTTSlotData[i].RREF)
			{
				sprintf_s(Buffer, "TVR");
			}
			else
			{
				sprintf_s(Buffer, "ADI");
			}
			skp->Text((2 * i + 8) * W / 32, 30 * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.0f", G->MTTSlotData[i].ROLL*DEG);
			skp->Text((2 * i + 8) * W / 32, 31 * H / 32, Buffer, strlen(Buffer));
		}
	}
	else if (screen == 5)
	{
		double hh, mm, ss;

		sprintf_s(Buffer, "DMT");
		skp->Text(15 * W / 32, 1 * H / 32, Buffer, strlen(Buffer));

		skp->SetFont(font2);

		sprintf_s(Buffer, "MNVR %d", G->DMT_MNVR);
		skp->Text(15 * W / 32, 3 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "PAD");
		skp->Text(16 * W / 32, 5 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "CODE");
		skp->Text(13 * W / 32, 6 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "TV ROLL");
		skp->Text(13 * W / 32, 7 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "TRIMS P");
		skp->Text(15 * W / 32, 8 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "LY");
		skp->Text(17 * W / 32, 9 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "RY");
		skp->Text(17 * W / 32, 10 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "WEIGHT");
		skp->Text(13 * W / 32, 11 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "TIG");
		skp->Text(13 * W / 32, 12 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "PEG 4 C1");
		skp->Text(13 * W / 32, 13 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "C2");
		skp->Text(15 * W / 32, 14 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "HT");
		skp->Text(15 * W / 32, 15 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "TT");
		skp->Text(15 * W / 32, 16 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "PRPLT");
		skp->Text(15 * W / 32, 17 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "PEG 7 DVX");
		skp->Text(13 * W / 32, 18 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DVY");
		skp->Text(16 * W / 32, 19 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DVZ");
		skp->Text(16 * W / 32, 20 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "BURN ATT R");
		skp->Text(13 * W / 32, 21 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "P");
		skp->Text(16 * W / 32, 22 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "Y");
		skp->Text(16 * W / 32, 23 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "DVTOT");
		skp->Text(13 * W / 32, 24 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "TGO");
		skp->Text(13 * W / 32, 25 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "VGO");
		skp->Text(13 * W / 32, 26 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "X");
		skp->Text(15 * W / 32, 26 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "Y");
		skp->Text(15 * W / 32, 27 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "Z");
		skp->Text(15 * W / 32, 28 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "TGT");
		skp->Text(13 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "HA");
		skp->Text(15 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "HP");
		skp->Text(37 * W / 64, 29 * H / 32, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::RIGHT);

		sprintf_s(Buffer, G->DMT.CODE);
		skp->Text(21 * W / 32, 6 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.TV_ROLL);
		skp->Text(21 * W / 32, 7 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f", G->DMT.TRIMS_P);
		skp->Text(21 * W / 32, 8 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f", G->DMT.TRIMS_LY);
		skp->Text(21 * W / 32, 9 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f", G->DMT.TRIMS_RY);
		skp->Text(21 * W / 32, 10 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.WEIGHT);
		skp->Text(21 * W / 32, 11 * H / 32, Buffer, strlen(Buffer));
		DMTMET2String(Buffer, G->DMT.TIG);
		skp->Text(21 * W / 32, 12 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.PEG4_C1);
		skp->Text(21 * W / 32, 13 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.4f", G->DMT.PEG4_C2);
		skp->Text(21 * W / 32, 14 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.3f", G->DMT.PEG4_HT);
		skp->Text(21 * W / 32, 15 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.3f", G->DMT.PEG4_THETAT);
		skp->Text(21 * W / 32, 16 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.PEG4_PRPLT);
		skp->Text(21 * W / 32, 17 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f", G->DMT.PEG7_DV.x);
		skp->Text(21 * W / 32, 18 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f", G->DMT.PEG7_DV.y);
		skp->Text(21 * W / 32, 19 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f", G->DMT.PEG7_DV.z);
		skp->Text(21 * W / 32, 20 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%03.0f", G->DMT.BURN_ATT.x);
		skp->Text(21 * W / 32, 21 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%03.0f", G->DMT.BURN_ATT.y);
		skp->Text(21 * W / 32, 22 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%03.0f", G->DMT.BURN_ATT.z);
		skp->Text(21 * W / 32, 23 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f", G->DMT.DVTOT);
		skp->Text(21 * W / 32, 24 * H / 32, Buffer, strlen(Buffer));

		SS2HHMMSS(G->DMT.TGO, hh, mm, ss);
		sprintf_s(Buffer, "%02.0f:%02.0f", mm, ss);
		skp->Text(21 * W / 32, 25 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.2f", G->DMT.VGO.x);
		skp->Text(21 * W / 32, 26 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.2f", G->DMT.VGO.y);
		skp->Text(21 * W / 32, 27 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.2f", G->DMT.VGO.z);
		skp->Text(21 * W / 32, 28 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.TGT_HA);
		skp->Text(18 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.TGT_HP);
		skp->Text(21 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::LEFT);
	}
	else if (screen == 6)
	{
		skp->Text(14 * W / 32, 1 * H / 32, "OMP Executive Menu", 18);

		skp->Text(1 * W / 8, 2 * H / 14, "Chaser:", 7);
		skp->Text(1 * W / 8, 4 * H / 14, "Target:", 7);
		skp->Text(1 * W / 8, 6 * H / 14, "Liftoff Time:", 13);
		skp->Text(1 * W / 8, 8 * H / 14, "Propagation:", 13);
		skp->Text(1 * W / 8, 10 * H / 14, "Save to file", 12);
		skp->Text(1 * W / 8, 12 * H / 14, "Load from file", 14);

		if (G->chaserSVOption)
		{
			skp->Text(4 * W / 8, 2 * H / 14, "LWP Output", 10);
		}
		else
		{
			if (G->shuttle)
			{
				sprintf(Buffer, G->shuttle->GetName());
				skp->Text(4 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));
			}
		}
		if (G->target)
		{
			sprintf(Buffer, G->target->GetName());
			skp->Text(4 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));
		}

		sprintf(Buffer, "%04d:%03d:%02d:%02d:%06.3f", G->launchdate[0], G->launchdate[1], G->launchdate[2], G->launchdate[3], G->launchdateSec);
		skp->Text(4 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		if (G->useNonSphericalGravity)
		{
			skp->Text(4 * W / 8, 8 * H / 14, "Non-spherical Gravity", 21);
		}
		else
		{
			skp->Text(4 * W / 8, 8 * H / 14, "Spherical Gravity", 17);
		}
	}
	else if (screen == 7)
	{
		double mm, ss;

		SS2MMSS(G->DTIG_ET_SEP, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(1 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f %.1f %.1f", G->DV_ET_SEP.x*MPS2FPS, G->DV_ET_SEP.y*MPS2FPS, G->DV_ET_SEP.z*MPS2FPS);
		skp->Text(1 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));

		SS2MMSS(G->DTIG_MPS, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(1 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f %.1f %.1f", G->DV_MPS.x*MPS2FPS, G->DV_MPS.y*MPS2FPS, G->DV_MPS.z*MPS2FPS);
		skp->Text(1 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f�", G->LWP_Settings.DELNO*DEG);
		skp->Text(1 * W / 8, 10 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.0f lbm", G->LWP_Settings.CWHT/LBM2KG);
		skp->Text(1 * W / 8, 12 * H / 14, Buffer, strlen(Buffer));

		SS2MMSS(G->LWP_Settings.DTOPT, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(5 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));

		SS2MMSS(G->LWP_Settings.TSTART, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(5 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));

		SS2MMSS(G->LWP_Settings.TEND, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(5 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%d", G->LWP_Settings.NEGTIV);
		skp->Text(5 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%d", G->LWP_Settings.WRAP);
		skp->Text(5 * W / 8, 10 * H / 14, Buffer, strlen(Buffer));

	}
	else if (screen == 8)
	{
		skp->Text(1 * W / 16, 2 * H / 14, "OPTIMUM L/O", 11);
		LWPGMT2String(Buffer, G->InPlaneGMT);
		skp->Text(4 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));
		skp->Text(2 * W / 8, 3 * H / 14, "PHASE", 5);
		sprintf_s(Buffer, "%.1f�", G->LPW_Summary.PA*DEG);
		skp->Text(4 * W / 8, 3 * H / 14, Buffer, strlen(Buffer));	

		skp->Text(1 * W / 16, 5 * H / 14, "PLANAR OPEN", 11);
		skp->Text(1 * W / 16, 6 * H / 14, "L/O", 3);
		skp->Text(1 * W / 16, 7 * H / 14, "PHASE", 5);
		LWPGMT2String(Buffer, G->LWP_PlanarOpenGMT);
		skp->Text(3 * W / 16, 6 * H / 14, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f�", G->LWP_Parameters.PHASE[0]*DEG);
		skp->Text(5 * W / 16, 7 * H / 14, Buffer, strlen(Buffer));

		skp->Text(4 * W / 8, 5 * H / 14, "PLANAR CLOSE", 12);
		skp->Text(4 * W / 8, 6 * H / 14, "L/O", 3);
		skp->Text(4 * W / 8, 7 * H / 14, "PHASE", 5);
		LWPGMT2String(Buffer, G->LWP_PlanarCloseGMT);
		skp->Text(5 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f�", G->LWP_Parameters.PHASE[1] * DEG);
		skp->Text(11 * W / 16, 7 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "IMECO:");
		skp->Text(1 * W / 16, 9 * H / 14, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.3f�", G->LPW_Summary.IIGM*DEG);
		skp->Text(1 * W / 16, 10 * H / 14, Buffer, strlen(Buffer));
	}
	return true;
}

void ShuttleFDOMFD::menuSetMainMenu()
{
	screen = 0;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuSetMCTPage()
{
	screen = 1;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuSetMETPage()
{
	METScroll = 0;
	screen = 2;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuSetLWPPage()
{
	screen = 3;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuSetMTTPage()
{
	screen = 4;
	coreButtons.SelectPage(this, screen);

	MTTFlag = false;
}

void ShuttleFDOMFD::menuSetDMTPage()
{
	screen = 5;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuSetOMPExeMenu()
{
	screen = 6;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuSetLWPPage2()
{
	screen = 7;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuSetLWPPage3()
{
	screen = 8;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::MET2String(char *buf, double MET)
{
	MET = round(MET*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%06.3f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void ShuttleFDOMFD::DMTMET2String(char *buf, double MET)
{
	MET = round(MET*10.0) / 10.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%04.1f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void ShuttleFDOMFD::GMT2String(char *buf, double GMT)
{
	GMT = round(GMT*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%06.3f", floor(GMT / 86400.0) + 1.0, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
}

void ShuttleFDOMFD::LWPGMT2String(char *buf, double GMT)
{
	GMT = round(GMT*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%04.1f", floor(GMT / 86400.0) + 1.0, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
}

double ShuttleFDOMFD::DDDHHHMMSS2MET(int dd, int hh, int mm, double ss)
{
	return ss + 60.0*mm + 3600.0*hh + 24.0*3600.0*dd;
}

void ShuttleFDOMFD::SS2HHMMSS(double val, double &hh, double &mm, double &ss)
{
	val = round(val);
	hh = floor(val / 3600.0);
	mm = floor(fmod(val, 3600.0) / 60.0);
	ss = fmod(val, 60.0);
}

void ShuttleFDOMFD::SS2MMSS(double val, double &mm, double &ss)
{
	mm = floor(abs(val) / 60.0);
	if (val < 0) mm = -mm;
	ss = fmod(abs(val), 60.0);
}

void ShuttleFDOMFD::menuAddOMPManeuver()
{
	bool AddOMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Add Maneuver (format: type name)", AddOMPManeuverInput, 0, 20, (void*)this);
}

bool AddOMPManeuverInput(void *id, char *str, void *data)
{
	char type[32], name[32];

	if (sscanf_s(str, "%s %s", type, 32, name, 32) == 2)
	{
		return ((ShuttleFDOMFD*)data)->add_OMPManeuver(type, name, 0);
	}
	return false;
}

bool ShuttleFDOMFD::add_OMPManeuver(char *type, char *name, unsigned ins)
{
	if (strcmp(type, "HA") == 0)
	{
		G->AddManeuver(OMPDefs::HA, name, ins);
		return true;
	}
	else if (strcmp(type, "HASH") == 0)
	{
		G->AddManeuver(OMPDefs::HASH, name, ins);
		return true;
	}
	else if (strcmp(type, "NC") == 0)
	{
		G->AddManeuver(OMPDefs::NC, name, ins);
		return true;
	}
	else if (strcmp(type, "EXDV") == 0)
	{
		G->AddManeuver(OMPDefs::EXDV, name, ins);
		return true;
	}
	else if (strcmp(type, "NH") == 0)
	{
		G->AddManeuver(OMPDefs::NH, name, ins);
		return true;
	}
	else if (strcmp(type, "SOI") == 0)
	{
		G->AddManeuver(OMPDefs::SOI, name, ins);
		return true;
	}
	else if (strcmp(type, "SOR") == 0)
	{
		G->AddManeuver(OMPDefs::SOR, name, ins);
		return true;
	}
	else if (strcmp(type, "NPC") == 0)
	{
		G->AddManeuver(OMPDefs::NPC, name, ins);
		return true;
	}
	else if (strcmp(type, "NCC") == 0)
	{
		G->AddManeuver(OMPDefs::NCC, name, ins);
		return true;
	}
	else if (strcmp(type, "APSO") == 0)
	{
		G->AddManeuver(OMPDefs::APSO, name, ins);
		return true;
	}
	else if (strcmp(type, "CIRC") == 0)
	{
		G->AddManeuver(OMPDefs::CIRC, name, ins);
		return true;
	}
	else if (strcmp(type, "NHRD") == 0)
	{
		G->AddManeuver(OMPDefs::NHRD, name, ins);
		return true;
	}
	else if (strcmp(type, "NSR") == 0)
	{
		G->AddManeuver(OMPDefs::NSR, name, ins);
		return true;
	}

	return false;
}

void ShuttleFDOMFD::menuModifySecondary()
{
	bool ModifyOMPSecondaryInput(void *id, char *str, void *data);
	oapiOpenInputBox("Modify Secondary (format: Man Sec Type Value)", ModifyOMPSecondaryInput, 0, 20, (void*)this);
}

bool ModifyOMPSecondaryInput(void *id, char *str, void *data)
{
	unsigned man, sec;
	char type[32];
	double val;

	if (sscanf_s(str, "%d %d %s %lf", &man, &sec, type, 32, &val) == 4)
	{
		return ((ShuttleFDOMFD*)data)->modify_OMPManeuverSecondary(man, sec, type, val);
	}
	return false;
}

bool ShuttleFDOMFD::modify_OMPManeuverSecondary(unsigned man, unsigned sec, char * str, double val)
{
	if (man <= G->ManeuverConstraintsTable.size() && man >= 1)
	{
		if (sec <= G->ManeuverConstraintsTable[man - 1].secondaries.size() && sec >= 1)
		{
			sprintf_s(G->ManeuverConstraintsTable[man - 1].secondaries[sec - 1].type, 5, str);
			G->ManeuverConstraintsTable[man - 1].secondaries[sec - 1].value = val;
			return true;
		}
	}
	return false;
}


void ShuttleFDOMFD::menuModifyOMPManeuver()
{
	bool ModifyOMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Modify Maneuver (format: ID type name)", ModifyOMPManeuverInput, 0, 20, (void*)this);
}

bool ModifyOMPManeuverInput(void *id, char *str, void *data)
{
	unsigned num;
	char type[32], name[32];

	if (sscanf_s(str, "%d %s %s", &num, type, 32, name, 32) == 3)
	{
		return ((ShuttleFDOMFD*)data)->modify_OMPManeuver(num, type, name);
	}
	return false;
}

bool ShuttleFDOMFD::modify_OMPManeuver(unsigned num, char *type, char *name)
{
	if (strcmp(type, "HA") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::HA, name);
		return true;
	}
	else if (strcmp(type, "HASH") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::HASH, name);
		return true;
	}
	else if (strcmp(type, "NC") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::NC, name);
		return true;
	}
	else if (strcmp(type, "EXDV") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::EXDV, name);
		return true;
	}
	else if (strcmp(type, "NH") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::NH, name);
		return true;
	}
	else if (strcmp(type, "SOI") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::SOI, name);
		return true;
	}
	else if (strcmp(type, "SOR") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::SOR, name);
		return true;
	}
	else if (strcmp(type, "NPC") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::NPC, name);
		return true;
	}
	else if (strcmp(type, "NCC") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::NCC, name);
		return true;
	}
	else if (strcmp(type, "APSO") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::APSO, name);
		return true;
	}
	else if (strcmp(type, "CIRC") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::CIRC, name);
		return true;
	}
	else if (strcmp(type, "NHRD") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::NHRD, name);
		return true;
	}
	else if (strcmp(type, "NSR") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::NSR, name);
		return true;
	}

	return false;
}

void ShuttleFDOMFD::menuAddOMPThreshold()
{
	bool AddOMPThresholdInput(void *id, char *str, void *data);
	oapiOpenInputBox("Add Maneuver Threshold (format: Man Type Value)", AddOMPThresholdInput, 0, 20, (void*)this);
}

bool AddOMPThresholdInput(void *id, char *str, void *data)
{
	unsigned num;
	char type[32], time[32];

	if (sscanf_s(str, "%d %s %s", &num, type, 32, time, 32) == 3)
	{
		return ((ShuttleFDOMFD*)data)->add_OMPManeuverThreshold(num, type, time);
	}
	return false;
}

bool ShuttleFDOMFD::add_OMPManeuverThreshold(unsigned num, char *type, char * str)
{
	if (num <= G->ManeuverConstraintsTable.size() && num >= 1)
	{
		if (strcmp(type, "T") == 0)
		{
			int dd, hh, mm;
			double ss;
			if (sscanf_s(str, "%d:%d:%d:%lf", &dd, &hh, &mm, &ss) == 4)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_T, DDDHHHMMSS2MET(dd, hh, mm, ss));
				return true;
			}
		}
		else if (strcmp(type, "M") == 0)
		{
			double m;
			if (sscanf_s(str, "%lf", &m) == 1)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_M, m);
				return true;
			}
		}
		else if (strcmp(type, "DT") == 0)
		{
			int dd, hh, mm;
			double ss;
			if (sscanf_s(str, "%d:%d:%d:%lf", &dd, &hh, &mm, &ss) == 4)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_DT, DDDHHHMMSS2MET(dd, hh, mm, ss));
				return true;
			}
		}
		else if (strcmp(type, "APS") == 0)
		{
			double aps;
			if (sscanf_s(str, "%lf", &aps) == 1)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_APS, aps);
				return true;
			}
		}
		else if (strcmp(type, "CAN") == 0)
		{
			double ang;
			if (sscanf_s(str, "%lf", &ang) == 1)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_CAN, ang*RAD);
				return true;
			}
		}
		else if (strcmp(type, "N") == 0)
		{
			double aps;
			if (sscanf_s(str, "%lf", &aps) == 1)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_N, aps);
				return true;
			}
		}
		else if (strcmp(type, "REV") == 0)
		{
			double aps;
			if (sscanf_s(str, "%lf", &aps) == 1)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_REV, aps);
				return true;
			}
		}
		else if (strcmp(type, "WT") == 0)
		{
			double ang;
			if (sscanf_s(str, "%lf", &ang) == 1)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::THRES_WT, ang*RAD);
				return true;
			}
		}

		return false;
	}

	return false;
}

void ShuttleFDOMFD::menuAddOMPSecondary()
{
	bool AddOMPSecondaryInput(void *id, char *str, void *data);
	oapiOpenInputBox("Add Secondary Constraint (format: Man Type Value)", AddOMPSecondaryInput, 0, 20, (void*)this);
}

bool AddOMPSecondaryInput(void *id, char *str, void *data)
{
	unsigned num;
	char type[32];
	double val;

	if (sscanf_s(str, "%d %s %lf", &num, type, 32, &val) == 3)
	{
		return ((ShuttleFDOMFD*)data)->add_OMPManeuverSecondary(num, type, val);
	}
	return false;
}

bool ShuttleFDOMFD::add_OMPManeuverSecondary(unsigned num, char * str, double val)
{
	if (num <= G->ManeuverConstraintsTable.size() && num >= 1)
	{
		if (G->ManeuverConstraintsTable[num - 1].secondaries.size() < 4)
		{
			G->AddManeuverSecondary(num - 1, str, val);
			return true;
		}
	}
	return false;
}

void ShuttleFDOMFD::GetOPMManeuverType(char *buf, OMPDefs::MANTYPE type)
{
	G->GetOPMManeuverType(buf, type);
}

void ShuttleFDOMFD::GetOPMManeuverThreshold(char *buf, OMPDefs::THRESHOLD type)
{
	if (type == OMPDefs::THRESHOLD::THRES_T)
	{
		sprintf_s(buf, 100, "T");
	}
	else if (type == OMPDefs::THRESHOLD::THRES_DT)
	{
		sprintf_s(buf, 100, "DT");
	}
	else if (type == OMPDefs::THRESHOLD::THRES_M)
	{
		sprintf_s(buf, 100, "M");
	}
	else if (type == OMPDefs::THRESHOLD::THRES_APS)
	{
		sprintf_s(buf, 100, "APS");
	}
	else if (type == OMPDefs::THRESHOLD::THRES_CAN)
	{
		sprintf_s(buf, 100, "CAN");
	}
	else if (type == OMPDefs::THRESHOLD::THRES_N)
	{
		sprintf_s(buf, 100, "N");
	}
	else if (type == OMPDefs::THRESHOLD::THRES_REV)
	{
		sprintf_s(buf, 100, "REV");
	}
	else if (type == OMPDefs::THRESHOLD::THRES_WT)
	{
		sprintf_s(buf, 100, "WT");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}

}

void ShuttleFDOMFD::GetOPMManeuverThresholdTime(char *buf, OMPDefs::THRESHOLD type, double num)
{
	if (type == OMPDefs::THRESHOLD::THRES_T)
	{
		MET2String(buf, num);
	}
	else if (type == OMPDefs::THRESHOLD::THRES_DT)
	{
		MET2String(buf, num);
	}
	else if (type == OMPDefs::THRESHOLD::THRES_M)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMPDefs::THRESHOLD::THRES_APS)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMPDefs::THRESHOLD::THRES_CAN)
	{
		sprintf_s(buf, 100, "%.1f�", num*DEG);
	}
	else if (type == OMPDefs::THRESHOLD::THRES_N)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMPDefs::THRESHOLD::THRES_REV)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMPDefs::THRESHOLD::THRES_WT)
	{
		sprintf_s(buf, 100, "%.1f�", num*DEG);
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOMFD::GetOPMManeuverSecondary(char *buf, char *type, double num)
{
	if (strlen(type) > 0)
	{
		if (strcmp(type, "CXYZ") == 0)
		{
			sprintf_s(buf, 100, "%s =%.4f", type, num);
		}
		else
		{
			sprintf_s(buf, 100, "%s =%.1f", type, num);
		}
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOMFD::menuDeleteOMPManeuver()
{
	bool DeleteOMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Delete specified maneuver: ", DeleteOMPManeuverInput, 0, 20, (void*)this);
}

bool DeleteOMPManeuverInput(void *id, char *str, void *data)
{
	unsigned num;

	if (sscanf_s(str, "%d", &num) == 1)
	{
		return ((ShuttleFDOMFD*)data)->delete_OMPManeuver(num);
	}
	return false;
}

bool ShuttleFDOMFD::delete_OMPManeuver(unsigned num)
{
	if (num >= 1 && num <= G->ManeuverConstraintsTable.size())
	{
		MCTScroll = 0;
		METScroll = 0;
		G->ManeuverConstraintsTable.erase(G->ManeuverConstraintsTable.begin() + num - 1);
		return true;
	}

	return false;
}

void ShuttleFDOMFD::menuCalculateOMPPlan()
{
	G->CalcMCT();
}

void ShuttleFDOMFD::menuCalcLaunchTime()
{
	G->CalcLaunchTime();
}

void ShuttleFDOMFD::menuTransferToMTT()
{
	G->MET2MTT();
}

void ShuttleFDOMFD::GetMTTThrusterType(char *buf, OMPDefs::THRUSTERS type)
{
	G->GetMTTThrusterType(buf, type);
}

void ShuttleFDOMFD::GetMTTGuidanceType(char *buf, OMPDefs::GUID type)
{
	if (type == OMPDefs::GUID::M50)
	{
		sprintf_s(buf, 100, "M50");
	}
	else if (type == OMPDefs::GUID::P7)
	{
		sprintf_s(buf, 100, "P7");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOMFD::menuMTTChangeSlot()
{
	bool MTTChangeSlotInput(void *id, char *str, void *data);
	oapiOpenInputBox("Change maneuver data (format: MNVR SLOT)", MTTChangeSlotInput, 0, 20, (void*)this);
}

bool MTTChangeSlotInput(void *id, char *str, void *data)
{
	unsigned mnvr;
	int slot;

	if (sscanf_s(str, "%d %d", &mnvr, &slot) == 2)
	{
		return ((ShuttleFDOMFD*)data)->set_MTTManeuverSlot(mnvr, slot);
	}
	return false;
}

bool ShuttleFDOMFD::set_MTTManeuverSlot(unsigned mnvr, int slot)
{
	if (mnvr >= 1 && mnvr <= G->ManeuverTransferTable.size())
	{
		G->ChangeMTTManeuverSlot(mnvr - 1, slot);
		return true;
	}

	return false;
}

void ShuttleFDOMFD::menuExecuteMTT()
{
	G->ExecuteMTT();
	MTTFlag = true;
}

void ShuttleFDOMFD::menuDMTChooseManeuver()
{
	bool DMTChooseManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Choose maneuver from table:", DMTChooseManeuverInput, 0, 20, (void*)this);
}

bool DMTChooseManeuverInput(void *id, char *str, void *data)
{
	unsigned mnvr;

	if (sscanf_s(str, "%d", &mnvr) == 1)
	{
		((ShuttleFDOMFD*)data)->set_DMTManeuver(mnvr);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_DMTManeuver(unsigned mnvr)
{
	G->DMT_MNVR = mnvr;
}

void ShuttleFDOMFD::menuCalcDMT()
{
	G->CalcDMT();
}

void ShuttleFDOMFD::menuDeleteOMPSecondary()
{
	bool DeleteOMPSecondaryInput(void *id, char *str, void *data);
	oapiOpenInputBox("Secondary to delete (format: MAN SEC)", DeleteOMPSecondaryInput, 0, 20, (void*)this);
}

bool DeleteOMPSecondaryInput(void *id, char *str, void *data)
{
	unsigned num, sec;

	if (sscanf_s(str, "%d %d", &num, &sec) == 2)
	{
		return ((ShuttleFDOMFD*)data)->delete_OMPSecondary(num, sec);
	}
	return false;
}

bool ShuttleFDOMFD::delete_OMPSecondary(unsigned num, unsigned sec)
{
	if (num >= 1 && num <= G->ManeuverConstraintsTable.size())
	{
		if (sec >= 1 && sec <= G->ManeuverConstraintsTable[num - 1].secondaries.size())
		{
			G->ManeuverConstraintsTable[num - 1].secondaries.erase(G->ManeuverConstraintsTable[num - 1].secondaries.begin() + sec - 1);
			return true;
		}
	}
	return false;
}

void ShuttleFDOMFD::menuInsertOMPManeuver()
{
	bool InsertMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Insert maneuver at specified ID: ", InsertMPManeuverInput, 0, 20, (void*)this);
}

bool InsertMPManeuverInput(void *id, char *str, void *data)
{
	unsigned ins;
	char type[32], name[32];

	if (sscanf_s(str, "%d %s %s", &ins, type, 32, name, 32) == 3)
	{
		return ((ShuttleFDOMFD*)data)->insert_OMPManeuver(ins, type, name);
	}
	return false;
}

bool ShuttleFDOMFD::insert_OMPManeuver(unsigned ins, char *type, char *name)
{
	if (ins >= 1 && ins <= G->ManeuverConstraintsTable.size() + 1)
	{
		return add_OMPManeuver(type, name, ins);
	}
	return false;
}

void ShuttleFDOMFD::menuSetLiftoffTime()
{
	bool LiftoffTimeInput(void *id, char *str, void *data);
	oapiOpenInputBox("Set liftoff time (YYYY:DD:HH:MM:SS)", LiftoffTimeInput, 0, 20, (void*)this);
}

bool LiftoffTimeInput(void *id, char *str, void *data)
{
	int yy, dd, hh, mm;
	double ss;

	if (sscanf_s(str, "%d:%d:%d:%d:%lf", &yy, &dd, &hh, &mm, &ss) == 5)
	{
		((ShuttleFDOMFD*)data)->set_LiftoffTime(yy, dd, hh, mm, ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LiftoffTime(int YY, int DD, int HH, int MM, double SS)
{
	G->SetLaunchMJD(YY, DD, HH, MM, SS);
}

void ShuttleFDOMFD::set_target()
{
	int vesselcount;

	vesselcount = oapiGetVesselCount();

	if (G->targetnumber < vesselcount - 1)
	{
		G->targetnumber++;
	}
	else
	{
		G->targetnumber = 0;
	}

	G->target = oapiGetVesselInterface(oapiGetVesselByIndex(G->targetnumber));
}

void ShuttleFDOMFD::set_shuttle()
{
	int vesselcount;

	vesselcount = oapiGetVesselCount();

	if (G->shuttlenumber < vesselcount - 1)
	{
		G->shuttlenumber++;
	}
	else
	{
		G->shuttlenumber = 0;
	}

	G->shuttle = oapiGetVesselInterface(oapiGetVesselByIndex(G->shuttlenumber));
	G->chaserSVOption = false;
}

void ShuttleFDOMFD::menuCycleGravityOption()
{
	G->useNonSphericalGravity = !G->useNonSphericalGravity;
}

void ShuttleFDOMFD::GetOMPError(char *buf, int err)
{
	if (err == 1)
	{
		sprintf_s(buf, 100, "Error: no maneuvers in constraint table");
	}
	else if (err == 2)
	{
		sprintf_s(buf, 100, "Error: first maneuver needs a T as threshold");
	}
	else if (err == 3)
	{
		sprintf_s(buf, 100, "Error: not enough DV components specified");
	}
	else if (err == 4)
	{
		sprintf_s(buf, 100, "Error: too many DV components specified");
	}
	else if (err == 5)
	{
		sprintf_s(buf, 100, "Error: a maneuver doesn't have a threshold");
	}
	else if (err == 6)
	{
		sprintf_s(buf, 100, "Error: could not find HD constraints for HA maneuver");
	}
	else if (err == 7)
	{
		sprintf_s(buf, 100, "Error: NC maneuver has no DR constraint specified");
	}
	else if (err == 8)
	{
		sprintf_s(buf, 100, "Error: NH maneuver has no DH constraint specified");
	}
	else if (err == 9)
	{
		sprintf_s(buf, 100, "Error: too many CXYZ components specified");
	}
	else if (err == 10)
	{
		sprintf_s(buf, 100, "Error: not enough CXYZ components specified");
	}
	else if (err == 11)
	{
		sprintf_s(buf, 100, "Error: no maneuver after SOI/NCC");
	}
	else if (err == 12)
	{
		sprintf_s(buf, 100, "Error: wrong maneuver after SOI");
	}
	else if (err == 14)
	{
		sprintf_s(buf, 100, "Error: CN secondary only applies to NPC");
	}
	else if (err == 20)
	{
		sprintf_s(buf, 100, "Error: Too many iterations");
	}
	else if (err == 22)
	{
		sprintf_s(buf, 100, "Error: More than one NPC maneuver specified");
	}
	else if (err == 23)
	{
		sprintf_s(buf, 100, "Error: No valid threshold for SOI/NCC");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOMFD::menuSaveState()
{
	bool SaveStateInput(void *id, char *str, void *data);
	oapiOpenInputBox("Choose name of file to save:", SaveStateInput, 0, 20, (void*)this);
}

bool SaveStateInput(void *id, char *str, void *data)
{
	return ((ShuttleFDOMFD*)data)->SaveState(str);
}

bool ShuttleFDOMFD::SaveState(char *filename)
{
	char Buffer[128];
	sprintf_s(Buffer, ".\\Config\\MFD\\ShuttleFDOMFD\\%s.txt", filename);
	std::ofstream myfile;
	myfile.open(Buffer);
	if (myfile.is_open())
	{
		papiWriteLine_int(myfile, "LAUNCHDATE0", G->launchdate[0]);
		papiWriteLine_int(myfile, "LAUNCHDATE1", G->launchdate[1]);
		papiWriteLine_int(myfile, "LAUNCHDATE2", G->launchdate[2]);
		papiWriteLine_int(myfile, "LAUNCHDATE3", G->launchdate[3]);
		papiWriteLine_double(myfile, "LAUNCHDATE4", G->launchdateSec);
		if (G->shuttle)
			papiWriteLine_string(myfile, "SHUTTLE", G->shuttle->GetName());
		if (G->target)
			papiWriteLine_string(myfile, "TARGET", G->target->GetName());
		papiWriteLine_bool(myfile, "NONSPHERICAL", G->useNonSphericalGravity);
		myfile << "START_MCT" << std::endl;
		for (unsigned i = 0;i < G->ManeuverConstraintsTable.size();i++)
		{
			WriteMCTLine(myfile, G->ManeuverConstraintsTable[i]);
		}
		myfile << "END_MCT" << std::endl;

		myfile.close();
	}

return true;
}

void ShuttleFDOMFD::menuLoadState()
{
	bool LoadStateInput(void *id, char *str, void *data);
	oapiOpenInputBox("Choose name of file to load:", LoadStateInput, 0, 20, (void*)this);
}

bool LoadStateInput(void *id, char *str, void *data)
{
	return ((ShuttleFDOMFD*)data)->LoadState(str);
}

bool ShuttleFDOMFD::LoadState(char *filename)
{
	bool isMCT = false;
	char Buffer[128];
	char shuttlebuff[100] = "";
	char targetbuff[100] = "";
	sprintf_s(Buffer, ".\\Config\\MFD\\ShuttleFDOMFD\\%s.txt", filename);
	std::ifstream myfile;
	myfile.open(Buffer);
	if (myfile.is_open())
	{
		G->ManeuverConstraintsTable.clear();
		G->chaserSVOption = false;

		std::string line;
		while (std::getline(myfile, line))
		{
			papiReadScenario_int(line.c_str(), "LAUNCHDATE0", G->launchdate[0]);
			papiReadScenario_int(line.c_str(), "LAUNCHDATE1", G->launchdate[1]);
			papiReadScenario_int(line.c_str(), "LAUNCHDATE2", G->launchdate[2]);
			papiReadScenario_int(line.c_str(), "LAUNCHDATE3", G->launchdate[3]);
			papiReadScenario_double(line.c_str(), "LAUNCHDATE4", G->launchdateSec);
			papiReadScenario_string(line.c_str(), "SHUTTLE", shuttlebuff);
			papiReadScenario_string(line.c_str(), "TARGET", targetbuff);
			papiReadScenario_bool(line.c_str(), "NONSPHERICAL", G->useNonSphericalGravity);
			if (strcmp(line.c_str(), "END_MCT") == 0) isMCT = false;
			if (isMCT) ReadMCTLine(line.c_str());
			if (strcmp(line.c_str(), "START_MCT") == 0) isMCT = true;
		}

		myfile.close();

		//Processing
		G->SetLaunchMJD(G->launchdate[0], G->launchdate[1], G->launchdate[2], G->launchdate[3], G->launchdateSec);

		OBJHANDLE hShuttle = oapiGetVesselByName(shuttlebuff);
		if (hShuttle)
		{
			G->shuttle = oapiGetVesselInterface(hShuttle);
			for (unsigned i = 0;i < oapiGetVesselCount();i++)
			{
				if (hShuttle == oapiGetVesselByIndex(i))
				{
					G->shuttlenumber = i;
				}
			}
		}

		OBJHANDLE hTarget = oapiGetVesselByName(targetbuff);
		if (hTarget)
		{
			G->target = oapiGetVesselInterface(hTarget);
			for (unsigned i = 0;i < oapiGetVesselCount();i++)
			{
				if (hTarget == oapiGetVesselByIndex(i))
				{
					G->targetnumber = i;
				}
			}
		}

		return true;
	}

	return false;
}

void ShuttleFDOMFD::WriteMCTLine(std::ofstream &file, ManeuverConstraints &constr)
{
	char sectype[MAXSECONDARIES][5];
	double secnum[MAXSECONDARIES];
	for (unsigned i = 0;i < MAXSECONDARIES;i++)
	{
		sprintf_s(sectype[i], 5, "NSEC");
		secnum[i] = 0.0;
	}
	for (unsigned i = 0;i < constr.secondaries.size();i++)
	{
		sprintf_s(sectype[i], 5, constr.secondaries[i].type);
		secnum[i] = constr.secondaries[i].value;
	}

	sprintf_s(Buffer, 100, "%s %d %d %lf %s %lf %s %lf %s %lf %s %lf", constr.name, constr.type, constr.threshold, constr.thresh_num,
		sectype[0], secnum[0], sectype[1], secnum[1], sectype[2], secnum[2], sectype[3], secnum[3]);
	file << Buffer << std::endl;
}

void ShuttleFDOMFD::ReadMCTLine(const char *line)
{
	SecData sec;
	unsigned i = 0;
	char sectype[MAXSECONDARIES][5];
	double secnum[MAXSECONDARIES];
	for (i = 0;i < MAXSECONDARIES;i++)
	{
		sprintf_s(sectype[i], 5, "NSEC");
		secnum[i] = 0.0;
	}
	ManeuverConstraints temp;
	if (sscanf_s(line, "%s %d %d %lf %s %lf %s %lf %s %lf %s %lf", temp.name, 64, &temp.type, &temp.threshold, &temp.thresh_num,
		sectype[0], 5, &secnum[0], sectype[1], 5, &secnum[1], sectype[2], 5, &secnum[2], sectype[3], 5, &secnum[3]) == 12)
	{
		i = 0;
		G->ManeuverConstraintsTable.push_back(temp);
		while (strcmp(sectype[i], "NSEC") && i < MAXSECONDARIES)
		{
			sprintf_s(sec.type, sectype[i]);
			sec.value = secnum[i];
			G->ManeuverConstraintsTable.back().secondaries.push_back(sec);
			i++;
		}
	}
}

void ShuttleFDOMFD::menuScrollMETUp()
{
	if (METScroll > 0) METScroll--;
}

void ShuttleFDOMFD::menuScrollMETDown()
{
	if (METScroll + 8 < G->ManeuverEvaluationTable.size()) METScroll++;
}

void ShuttleFDOMFD::menuScrollMCTUp()
{
	if (MCTScroll > 0) MCTScroll--;
}

void ShuttleFDOMFD::menuScrollMCTDown()
{
	if (MCTScroll + 9 < G->ManeuverConstraintsTable.size()) MCTScroll++;
}

void ShuttleFDOMFD::menuLWPSetDELNO()
{
	bool LWPDELNOInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DELNO in degrees:", LWPDELNOInput, 0, 20, (void*)this);
}

bool LWPDELNOInput(void *id, char *str, void *data)
{
	double delno;

	if (sscanf_s(str, "%lf", &delno) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DELNO(delno);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DELNO(double delno)
{
	G->LWP_Settings.DELNO = delno * RAD;
}

void ShuttleFDOMFD::menuLWPSetDTOPT()
{
	bool LWPDTOPTInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DTOPT in MM:SS", LWPDTOPTInput, 0, 20, (void*)this);
}

bool LWPDTOPTInput(void *id, char *str, void *data)
{
	double mm, ss;

	if (sscanf_s(str, "%lf:%lf", &mm, &ss) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DTOPT(mm*60.0 + ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DTOPT(double dtopt)
{
	G->LWP_Settings.DTOPT = dtopt;
}

void ShuttleFDOMFD::menuLWPSetLS()
{
	bool LWPLSInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input launch site:", LWPLSInput, 0, 20, (void*)this);
}

bool LWPLSInput(void *id, char *str, void *data)
{
	if (strcmp(str, "39A") == 0)
	{
		((ShuttleFDOMFD*)data)->set_LWP_LS(1);
		return true;
	}
	else if (strcmp(str, "39B") == 0)
	{
		((ShuttleFDOMFD*)data)->set_LWP_LS(2);
		return true;
	}

	return false;
}

void ShuttleFDOMFD::set_LWP_LS(int ls)
{
	G->LWP_LaunchSite = ls;
	G->LWP_Settings.LATLS = LAUNCHSITE_LATITUDE[ls - 1]*RAD;
	G->LWP_Settings.LONGLS = LAUNCHSITE_LONGITUDE[ls - 1]*RAD;
}

void ShuttleFDOMFD::menuLWPSetLSLat()
{
	bool LWPLSLatInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input launch site latitude in degrees:", LWPLSLatInput, 0, 20, (void*)this);
}

bool LWPLSLatInput(void *id, char *str, void *data)
{
	double lat;

	if (sscanf_s(str, "%lf", &lat) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_LSLat(lat);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_LSLat(double lat)
{
	G->LWP_LaunchSite = 0;
	G->LWP_Settings.LATLS = lat * RAD;
}

void ShuttleFDOMFD::menuLWPSetLSLng()
{
	bool LWPLSLngInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input launch site longitude in degrees:", LWPLSLatInput, 0, 20, (void*)this);
}

bool LWPLSLngInput(void *id, char *str, void *data)
{
	double lng;

	if (sscanf_s(str, "%lf", &lng) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_LSLng(lng);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_LSLng(double lng)
{
	G->LWP_LaunchSite = 0;
	G->LWP_Settings.LONGLS = lng * RAD;
}

void ShuttleFDOMFD::menuLWPSetYS()
{
	bool LWPYSInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input max yaw steering in degrees:", LWPYSInput, 0, 20, (void*)this);
}

bool LWPYSInput(void *id, char *str, void *data)
{
	double ys;

	if (sscanf_s(str, "%lf", &ys) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_YS(ys);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_YS(double ys)
{
	G->LWP_Settings.YSMAX = ys * RAD;
}

void ShuttleFDOMFD::menuLWPSetPFA()
{
	bool LWPPFAInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input powered flight arc in degrees:", LWPPFAInput, 0, 20, (void*)this);
}

bool LWPPFAInput(void *id, char *str, void *data)
{
	double pfa;

	if (sscanf_s(str, "%lf", &pfa) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_PFA(pfa);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_PFA(double pfa)
{
	G->LWP_Settings.PFA = pfa * RAD;
}

void ShuttleFDOMFD::menuLWPSetPFT()
{
	bool LWPPFTInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input powered flight time in MM:SS", LWPPFTInput, 0, 20, (void*)this);
}

bool LWPPFTInput(void *id, char *str, void *data)
{
	double mm, ss;

	if (sscanf_s(str, "%lf:%lf", &mm, &ss) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_PFT(mm*60.0 + ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_PFT(double pft)
{
	G->LWP_Settings.PFT = pft;
}

void ShuttleFDOMFD::menuLWPSetFPA()
{
	bool LWPFPAInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input powered flight arc in degrees:", LWPFPAInput, 0, 20, (void*)this);
}

bool LWPFPAInput(void *id, char *str, void *data)
{
	double fpa;

	if (sscanf_s(str, "%lf", &fpa) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_FPA(fpa);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_FPA(double fpa)
{
	G->LWP_Settings.GAMINS = fpa * RAD;
}

void ShuttleFDOMFD::menuLWPSetRAD()
{
	bool LWPRADInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input insertion radius in feet:", LWPRADInput, 0, 20, (void*)this);
}

bool LWPRADInput(void *id, char *str, void *data)
{
	double rad;

	if (sscanf_s(str, "%lf", &rad) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_RAD(rad);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_RAD(double rad)
{
	G->LWP_Settings.RINS = rad / MPS2FPS;
}

void ShuttleFDOMFD::menuLWPSetVEL()
{
	bool LWPVELInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input insertion velocity in feet per second:", LWPVELInput, 0, 20, (void*)this);
}

bool LWPVELInput(void *id, char *str, void *data)
{
	double vel;

	if (sscanf_s(str, "%lf", &vel) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_VEL(vel);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_VEL(double vel)
{
	G->LWP_Settings.VINS = vel / MPS2FPS;
}

void ShuttleFDOMFD::menuLWPSetDTETSEP()
{
	bool LWPDTETSEPInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DTIG of ET SEP from MECO in MM:SS", LWPDTETSEPInput, 0, 20, (void*)this);
}

bool LWPDTETSEPInput(void *id, char *str, void *data)
{
	double mm, ss;

	if (sscanf_s(str, "%lf:%lf", &mm, &ss) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DTETSEP(mm*60.0 + ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DTETSEP(double dt)
{
	G->DTIG_ET_SEP = dt;
}

void ShuttleFDOMFD::menuLWPSetDTMPS()
{
	bool LWPDTMPSInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DTIG of MPS DUMP from MECO in MM:SS", LWPDTMPSInput, 0, 20, (void*)this);
}

bool LWPDTMPSInput(void *id, char *str, void *data)
{
	double mm, ss;

	if (sscanf_s(str, "%lf:%lf", &mm, &ss) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DTMPS(mm*60.0 + ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DTMPS(double dt)
{
	G->DTIG_MPS = dt;
}

void ShuttleFDOMFD::menuLWPSetDVETSEP()
{
	bool LWPDVETSEPInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DV of ET SEP:", LWPDVETSEPInput, 0, 20, (void*)this);
}

bool LWPDVETSEPInput(void *id, char *str, void *data)
{
	double x, y, z;

	if (sscanf_s(str, "%lf %lf %lf", &x, &y, &z) == 3)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DVETSEP(_V(x, y, z));
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DVETSEP(VECTOR3 DV)
{
	G->DV_ET_SEP = DV / MPS2FPS;
}

void ShuttleFDOMFD::menuLWPSetDVMPS()
{
	bool LWPDVMPSInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DV of MPS Dump:", LWPDVMPSInput, 0, 20, (void*)this);
}

bool LWPDVMPSInput(void *id, char *str, void *data)
{
	double x, y, z;

	if (sscanf_s(str, "%lf %lf %lf", &x, &y, &z) == 3)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DVMPS(_V(x, y, z));
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DVMPS(VECTOR3 DV)
{
	G->DV_MPS = DV / MPS2FPS;
}

void ShuttleFDOMFD::menuLWPSetWT()
{
	bool LWPWTInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input chaser weight before OMS-2:", LWPWTInput, 0, 20, (void*)this);
}

bool LWPWTInput(void *id, char *str, void *data)
{
	double wt;

	if (sscanf_s(str, "%lf", &wt) == 1)
	{
		((ShuttleFDOMFD*)data)->set_LWP_WT(wt);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_WT(double wt)
{
	G->LWP_Settings.CWHT = wt*LBM2KG;
}

void ShuttleFDOMFD::menuLWPSetDTO()
{
	bool LWPDTOInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DT of launch window opening in MM:SS", LWPDTOInput, 0, 20, (void*)this);
}

bool LWPDTOInput(void *id, char *str, void *data)
{
	double mm, ss;

	if (sscanf_s(str, "%lf:%lf", &mm, &ss) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DTO(mm*60.0 + ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DTO(double dt)
{
	G->LWP_Settings.TSTART = dt;
}

void ShuttleFDOMFD::menuLWPSetDTC()
{
	bool LWPDTCInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input DT of launch window closing in MM:SS", LWPDTCInput, 0, 20, (void*)this);
}

bool LWPDTCInput(void *id, char *str, void *data)
{
	double mm, ss;

	if (sscanf_s(str, "%lf:%lf", &mm, &ss) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_DTC(mm*60.0 + ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_DTC(double dt)
{
	G->LWP_Settings.TEND = dt;
}

void ShuttleFDOMFD::menuLWPSetPHASEFLAG()
{
	bool LWPPHASEFLAGInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input phase flag (0 = 0� to 360�, 1 = -360� to 0�, 2 = -180� to 180�):", LWPPHASEFLAGInput, 0, 20, (void*)this);
}

bool LWPPHASEFLAGInput(void *id, char *str, void *data)
{
	int flag;

	if (sscanf_s(str, "%d", &flag) == 1)
	{
		if (flag >= 0 && flag <= 2)
		{
			((ShuttleFDOMFD*)data)->set_LWP_PHASEFLAG(flag);
			return true;
		}
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_PHASEFLAG(int flag)
{
	G->LWP_Settings.NEGTIV = flag;
}

void ShuttleFDOMFD::menuLWPSetWRAPFLAG()
{
	bool LWPWRAPFLAGInput(void *id, char *str, void *data);
	oapiOpenInputBox("Input wrap flag (adds N*360� to phase angle):", LWPWRAPFLAGInput, 0, 20, (void*)this);
}

bool LWPWRAPFLAGInput(void *id, char *str, void *data)
{
	int flag;

	if (sscanf_s(str, "%d", &flag) == 1)
	{
		if (flag >= 0 && flag <= 2)
		{
			((ShuttleFDOMFD*)data)->set_LWP_WRAPFLAG(flag);
			return true;
		}
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_WRAPFLAG(int flag)
{
	G->LWP_Settings.WRAP = flag;
}