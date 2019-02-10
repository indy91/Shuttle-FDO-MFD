/****************************************************************************
  This file is part of OMP MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  OMP MFD

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
#include "Orbitersdk.h"
#include "papi.h"
#include "OrbMech.h"
#include "OMPCore.h"
#include "OMPMFD.h"
#include "OMPoapiModule.h"

// ==============================================================
// Global variables

OMPoapiModule *g_coreMod;
int g_MFDmode; // identifier for new MFD mode
OMPCore *GCoreData[32];
OBJHANDLE GCoreVessel[32];
int nGutsUsed;

// ==============================================================
// MFD class implementation

// Constructor
OMPMFD::OMPMFD(DWORD w, DWORD h, VESSEL *v, UINT im)
: MFD2 (w, h, v)
{
	font = oapiCreateFont(w / 20, true, "Courier", FONT_NORMAL, 0);
	font2 = oapiCreateFont(w / 30, true, "Courier", FONT_NORMAL, 0);
	// Add MFD initialisation here
	screen = 0;
	MTTFlag = false;
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
		GCoreData[nGutsUsed] = new OMPCore(v);
		G = GCoreData[nGutsUsed];
		GCoreVessel[nGutsUsed] = v;
		nGutsUsed++;
	}
}

// Destructor
OMPMFD::~OMPMFD()
{
	oapiReleaseFont(font);
	oapiReleaseFont(font2);
	// Add MFD cleanup code here
}

// Return button labels
char *OMPMFD::ButtonLabel (int bt)
{
	// The labels for the two buttons used by our MFD mode
	return coreButtons.ButtonLabel(bt);
}

// Return button menus
int OMPMFD::ButtonMenu (const MFDBUTTONMENU **menu) const
{
	// The menu descriptions for the two buttons
	return coreButtons.ButtonMenu(menu);
}

bool OMPMFD::ConsumeButton(int bt, int event)
{
	return coreButtons.ConsumeButton(this, bt, event);
}

bool OMPMFD::ConsumeKeyBuffered(DWORD key)
{
	return coreButtons.ConsumeKeyBuffered(this, key);
}

void OMPMFD::WriteStatus(FILEHANDLE scn) const
{
	oapiWriteScenario_int(scn, "LAUNCHDATE0", G->launchdate[0]);
	oapiWriteScenario_int(scn, "LAUNCHDATE1", G->launchdate[1]);
	oapiWriteScenario_int(scn, "LAUNCHDATE2", G->launchdate[2]);
	oapiWriteScenario_int(scn, "LAUNCHDATE3", G->launchdate[3]);
	papiWriteScenario_double(scn, "LAUNCHDATE4", G->launchdateSec);
	if (G->target)
		oapiWriteScenario_string(scn, "TARGET", G->target->GetName());
}
void OMPMFD::ReadStatus(FILEHANDLE scn)
{
	char *line;
	char targetbuff[100] = "";

	while (oapiReadScenario_nextline(scn, line)) {
		if (!_strnicmp(line, "END_MFD", 7))
			return;

		papiReadScenario_int(line, "LAUNCHDATE0", G->launchdate[0]);
		papiReadScenario_int(line, "LAUNCHDATE1", G->launchdate[1]);
		papiReadScenario_int(line, "LAUNCHDATE2", G->launchdate[2]);
		papiReadScenario_int(line, "LAUNCHDATE3", G->launchdate[3]);
		papiReadScenario_double(line, "LAUNCHDATE4", G->launchdateSec);
		papiReadScenario_string(line, "TARGET", targetbuff);
	}

	G->SetLaunchMJD(G->launchdate[0], G->launchdate[1], G->launchdate[2], G->launchdate[3], G->launchdateSec);
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
}

// Repaint the MFD
bool OMPMFD::Update(oapi::Sketchpad *skp)
{
	Title(skp, "OMP MFD");
	// Draws the MFD title

	skp->SetFont(font);
	//skp->SetTextAlign (oapi::Sketchpad::CENTER, oapi::Sketchpad::BASELINE);
	//skp->SetTextColor (0x00FFFF);

	// Add MFD display routines here.
	// Use the device context (hDC) for Windows GDI paint functions.

	if (screen == 0)
	{
		skp->Text(1 * W / 8, 2 * H / 14, "Executive Menu", 14);
		skp->Text(1 * W / 8, 4 * H / 14, "Maneuver Constraints Table", 27);
		skp->Text(1 * W / 8, 6 * H / 14, "Maneuver Evaluation Table", 25);
		skp->Text(1 * W / 8, 8 * H / 14, "Maneuver Transfer Table", 23);
		skp->Text(1 * W / 8, 10 * H / 14, "Detailed Maneuver Table", 23);
	}
	else if (screen == 1)
	{
		skp->SetFont(font2);

		sprintf_s(Buffer, "MANEUVER");
		skp->Text(1 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "THRESHOLD");
		skp->Text(8 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "SECONDARIES");
		skp->Text(17 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));

		if (G->OMPErrorCode)
		{
			GetOMPError(Buffer, G->OMPErrorCode);
			skp->Text(2 * W / 32, 31 * H / 32, Buffer, strlen(Buffer));
		}

		for (unsigned i = 0;i < G->ManeuverConstraintsTable.size();i++)
		{
			//MANEUVER
			sprintf_s(Buffer, "%d", i + 1);
			skp->Text(1 * W / 32, (i * 3 + 4) * H / 32, Buffer, strlen(Buffer));

			GetOPMManeuverType(Buffer, G->ManeuverConstraintsTable[i].type);
			skp->Text(2 * W / 32, (i * 3 + 4) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, 100, G->ManeuverConstraintsTable[i].name);
			skp->Text(1 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));

			//THRESHOLD
			GetOPMManeuverThreshold(Buffer, G->ManeuverConstraintsTable[i].threshold);
			skp->Text(9 * W / 32, (i * 3 + 4) * H / 32, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::CENTER);

			GetOPMManeuverThresholdTime(Buffer, G->ManeuverConstraintsTable[i].threshold, G->ManeuverConstraintsTable[i].thresh_num);
			skp->Text(9 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));

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
				skp->Text((26 + k * 13) * W / 64, (i * 3 + 5 + l) * H / 32, Buffer, strlen(Buffer));
			}
		}
	}
	else if (screen == 2)
	{
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

		for (unsigned i = 0;i < G->ManeuverEvaluationTable.size();i++)
		{
			sprintf_s(Buffer, "%d", i + 1);
			skp->Text(1 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, G->ManeuverEvaluationTable[i].type);
			skp->Text(3 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, G->ManeuverEvaluationTable[i].name);
			skp->Text(1 * W / 32, (i * 3 + 6) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable[i].DVMag);
			skp->Text(1 * W / 32, (i * 3 + 7) * H / 32, Buffer, strlen(Buffer));


			GMT2String(Buffer, G->ManeuverEvaluationTable[i].GMTIG);
			skp->Text(6 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));
			MET2String(Buffer, G->ManeuverEvaluationTable[i].METIG);
			skp->Text(6 * W / 32, (i * 3 + 6) * H / 32, Buffer, strlen(Buffer));
			MET2String(Buffer, G->ManeuverEvaluationTable[i].DT);
			skp->Text(6 * W / 32, (i * 3 + 7) * H / 32, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::RIGHT);

			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DV.x);
			skp->Text(16 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DV.y);
			skp->Text(16 * W / 32, (i * 3 + 6) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DV.z);
			skp->Text(16 * W / 32, (i * 3 + 7) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].HA);
			skp->Text(19 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].HP);
			skp->Text(19 * W / 32, (i * 3 + 6) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable[i].DH);
			skp->Text(19 * W / 32, (i * 3 + 7) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.4f", G->ManeuverEvaluationTable[i].RANGE);
			skp->Text(24 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.4f", G->ManeuverEvaluationTable[i].PHASE);
			skp->Text(24 * W / 32, (i * 3 + 6) * H / 32, Buffer, strlen(Buffer));
			SS2HHMMSS(G->ManeuverEvaluationTable[i].TTN, hh, mm, ss);
			if (G->ManeuverEvaluationTable[i].noon)
			{
				sprintf_s(Buffer, "N-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer, "M-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			skp->Text(24 * W / 32, (i * 3 + 7) * H / 32, Buffer, strlen(Buffer));

			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable[i].Y);
			skp->Text(30 * W / 32, (i * 3 + 5) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable[i].Ydot);
			skp->Text(30 * W / 32, (i * 3 + 6) * H / 32, Buffer, strlen(Buffer));
			SS2HHMMSS(G->ManeuverEvaluationTable[i].TTS, hh, mm, ss);
			if (G->ManeuverEvaluationTable[i].sunrise)
			{
				sprintf_s(Buffer, "SR-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer, "SS-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			skp->Text(30 * W / 32, (i * 3 + 7) * H / 32, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::LEFT);
		}
	}
	else if (screen == 3)
	{
		GMT2String(Buffer, G->InPlaneGMT);
		skp->Text(5 * W / 32, 5 * H / 28, Buffer, strlen(Buffer));
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
		skp->Text(1 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "NAME");
		skp->Text(4 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "COMMENT");
		skp->Text(7 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "SLOT");
		skp->Text(12 * W / 32, 2 * H / 32, Buffer, strlen(Buffer));
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
		skp->Text(18 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));

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
		sprintf_s(Buffer, "%.0f", G->DMT.BURN_ATT.x);
		skp->Text(21 * W / 32, 21 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.BURN_ATT.y);
		skp->Text(21 * W / 32, 22 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.BURN_ATT.z);
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
		skp->Text(17 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.0f", G->DMT.TGT_HP);
		skp->Text(21 * W / 32, 29 * H / 32, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::LEFT);
	}
	else if (screen == 6)
	{
		skp->Text(10 * W / 32, 1 * H / 32, "OMP Executive Menu", 18);

		skp->Text(1 * W / 8, 2 * H / 14, "Chaser:", 7);
		skp->Text(1 * W / 8, 4 * H / 14, "Target:", 7);
		skp->Text(1 * W / 8, 6 * H / 14, "Liftoff Time:", 13);
		skp->Text(1 * W / 8, 8 * H / 14, "Propagation:", 13);

		if (G->vessel)
		{
			sprintf(Buffer, G->vessel->GetName());
			skp->Text(4 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));
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

	return true;
}

void OMPMFD::menuSetMainMenu()
{
	screen = 0;
	coreButtons.SelectPage(this, screen);
}

void OMPMFD::menuSetMCTPage()
{
	screen = 1;
	coreButtons.SelectPage(this, screen);
}

void OMPMFD::menuSetMETPage()
{
	screen = 2;
	coreButtons.SelectPage(this, screen);
}

void OMPMFD::menuSetLaunchWindowPage()
{
	screen = 3;
	coreButtons.SelectPage(this, screen);
}

void OMPMFD::menuSetMTTPage()
{
	screen = 4;
	coreButtons.SelectPage(this, screen);

	MTTFlag = false;
}

void OMPMFD::menuSetDMTPage()
{
	screen = 5;
	coreButtons.SelectPage(this, screen);
}

void OMPMFD::menuSetOMPExeMenu()
{
	screen = 6;
	coreButtons.SelectPage(this, screen);
}

void OMPMFD::MET2String(char *buf, double MET)
{
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%06.3f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void OMPMFD::DMTMET2String(char *buf, double MET)
{
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%04.1f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void OMPMFD::GMT2String(char *buf, double GMT)
{
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%06.3f", floor(GMT / 86400.0) + 1.0, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
}

double OMPMFD::DDDHHHMMSS2MET(int dd, int hh, int mm, double ss)
{
	return ss + 60.0*mm + 3600.0*hh + 24.0*3600.0*dd;
}

void OMPMFD::SS2HHMMSS(double val, double &hh, double &mm, double &ss)
{
	val = round(val);
	hh = floor(val / 3600.0);
	mm = floor(fmod(val, 3600.0) / 60.0);
	ss = fmod(val, 60.0);
}

void OMPMFD::menuAddOMPManeuver()
{
	bool AddOMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Add Maneuver (format: type name)", AddOMPManeuverInput, 0, 20, (void*)this);
}

bool AddOMPManeuverInput(void *id, char *str, void *data)
{
	char type[32], name[32];

	if (sscanf_s(str, "%s %s", type, 32, name, 32) == 2)
	{
		return ((OMPMFD*)data)->add_OMPManeuver(type, name, 0);
	}
	return false;
}

bool OMPMFD::add_OMPManeuver(char *type, char *name, unsigned ins)
{
	if (strcmp(type, "HA") == 0)
	{
		G->AddManeuver(OMPDefs::HA, name, ins);
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

	return false;
}

void OMPMFD::menuModifyOMPManeuver()
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
		return ((OMPMFD*)data)->modify_OMPManeuver(num, type, name);
	}
	return false;
}

bool OMPMFD::modify_OMPManeuver(unsigned num, char *type, char *name)
{
	if (strcmp(type, "HA") == 0)
	{
		G->ModifyManeuver(num - 1, OMPDefs::HA, name);
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

	return false;
}

void OMPMFD::menuAddOMPThreshold()
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
		return ((OMPMFD*)data)->add_OMPManeuverThreshold(num, type, time);
	}
	return false;
}

bool OMPMFD::add_OMPManeuverThreshold(unsigned num, char *type, char * str)
{
	if (num <= G->ManeuverConstraintsTable.size() && num >= 1)
	{
		if (strcmp(type, "T") == 0)
		{
			int dd, hh, mm;
			double ss;
			if (sscanf_s(str, "%d:%d:%d:%lf", &dd, &hh, &mm, &ss) == 4)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::T, DDDHHHMMSS2MET(dd, hh, mm, ss));
				return true;
			}
		}
		else if (strcmp(type, "M") == 0)
		{
			double m;
			if (sscanf_s(str, "%lf", &m) == 1)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::M, m);
				return true;
			}
		}
		else if (strcmp(type, "DT") == 0)
		{
			int dd, hh, mm;
			double ss;
			if (sscanf_s(str, "%d:%d:%d:%lf", &dd, &hh, &mm, &ss) == 4)
			{
				G->AddManeuverThreshold(num - 1, OMPDefs::THRESHOLD::DT, DDDHHHMMSS2MET(dd, hh, mm, ss));
				return true;
			}
		}

		return false;
	}

	return false;
}

void OMPMFD::menuAddOMPSecondary()
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
		return ((OMPMFD*)data)->add_OMPManeuverSecondary(num, type, val);
	}
	return false;
}

bool OMPMFD::add_OMPManeuverSecondary(unsigned num, char * str, double val)
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

void OMPMFD::GetOPMManeuverType(char *buf, OMPDefs::MANTYPE type)
{
	G->GetOPMManeuverType(buf, type);
}

void OMPMFD::GetOPMManeuverThreshold(char *buf, OMPDefs::THRESHOLD type)
{
	if (type == OMPDefs::THRESHOLD::T)
	{
		sprintf_s(buf, 100, "T");
	}
	else if (type == OMPDefs::THRESHOLD::DT)
	{
		sprintf_s(buf, 100, "DT");
	}
	else if (type == OMPDefs::THRESHOLD::M)
	{
		sprintf_s(buf, 100, "M");
	}
	else
	{
		sprintf_s(buf, 100, "");
	}

}

void OMPMFD::GetOPMManeuverThresholdTime(char *buf, OMPDefs::THRESHOLD type, double num)
{
	if (type == OMPDefs::THRESHOLD::T)
	{
		MET2String(buf, num);
	}
	else if (type == OMPDefs::THRESHOLD::DT)
	{
		MET2String(buf, num);
	}
	else if (type == OMPDefs::THRESHOLD::M)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void OMPMFD::GetOPMManeuverSecondary(char *buf, char *type, double num)
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

void OMPMFD::menuDeleteOMPManeuver()
{
	bool DeleteOMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Delete specified maneuver: ", DeleteOMPManeuverInput, 0, 20, (void*)this);
}

bool DeleteOMPManeuverInput(void *id, char *str, void *data)
{
	unsigned num;

	if (sscanf_s(str, "%d", &num) == 1)
	{
		return ((OMPMFD*)data)->delete_OMPManeuver(num);
	}
	return false;
}

bool OMPMFD::delete_OMPManeuver(unsigned num)
{
	if (num >= 1 && num <= G->ManeuverConstraintsTable.size())
	{
		G->ManeuverConstraintsTable.erase(G->ManeuverConstraintsTable.begin() + num - 1);
		return true;
	}

	return false;
}

void OMPMFD::menuCalculateOMPPlan()
{
	G->CalcMCT();
}

void OMPMFD::menuCalcLaunchTime()
{
	G->CalcLaunchTime();
}

void OMPMFD::menuTransferToMTT()
{
	G->MET2MTT();
}

void OMPMFD::GetMTTThrusterType(char *buf, OMPDefs::THRUSTERS type)
{
	G->GetMTTThrusterType(buf, type);
}

void OMPMFD::GetMTTGuidanceType(char *buf, OMPDefs::GUID type)
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

void OMPMFD::menuMTTChangeSlot()
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
		return ((OMPMFD*)data)->set_MTTManeuverSlot(mnvr, slot);
	}
	return false;
}

bool OMPMFD::set_MTTManeuverSlot(unsigned mnvr, int slot)
{
	if (mnvr >= 1 && mnvr <= G->ManeuverTransferTable.size())
	{
		G->ChangeMTTManeuverSlot(mnvr - 1, slot);
		return true;
	}

	return false;
}

void OMPMFD::menuExecuteMTT()
{
	G->ExecuteMTT();
	MTTFlag = true;
}

void OMPMFD::menuDMTChooseManeuver()
{
	bool DMTChooseManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Choose maneuver from table:", DMTChooseManeuverInput, 0, 20, (void*)this);
}

bool DMTChooseManeuverInput(void *id, char *str, void *data)
{
	unsigned mnvr;

	if (sscanf_s(str, "%d", &mnvr) == 1)
	{
		((OMPMFD*)data)->set_DMTManeuver(mnvr);
		return true;
	}
	return false;
}

void OMPMFD::set_DMTManeuver(unsigned mnvr)
{
	G->DMT_MNVR = mnvr;
}

void OMPMFD::menuCalcDMT()
{
	G->CalcDMT();
}

void OMPMFD::menuDeleteOMPSecondary()
{
	bool DeleteOMPSecondaryInput(void *id, char *str, void *data);
	oapiOpenInputBox("Choose secondary to delete:", DeleteOMPSecondaryInput, 0, 20, (void*)this);
}

bool DeleteOMPSecondaryInput(void *id, char *str, void *data)
{
	unsigned num, sec;

	if (sscanf_s(str, "%d %d", &num, &sec) == 2)
	{
		return ((OMPMFD*)data)->delete_OMPSecondary(num, sec);
	}
	return false;
}

bool OMPMFD::delete_OMPSecondary(unsigned num, unsigned sec)
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

void OMPMFD::menuInsertOMPManeuver()
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
		return ((OMPMFD*)data)->insert_OMPManeuver(ins, type, name);
	}
	return false;
}

bool OMPMFD::insert_OMPManeuver(unsigned ins, char *type, char *name)
{
	if (ins >= 1 && ins <= G->ManeuverConstraintsTable.size() + 1)
	{
		return add_OMPManeuver(type, name, ins);
	}
	return false;
}

void OMPMFD::menuSetLiftoffTime()
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
		((OMPMFD*)data)->set_LiftoffTime(yy, dd, hh, mm, ss);
		return true;
	}
	return false;
}

void OMPMFD::set_LiftoffTime(int YY, int DD, int HH, int MM, double SS)
{
	G->SetLaunchMJD(YY, DD, HH, MM, SS);
}

void OMPMFD::set_target()
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

void OMPMFD::menuCycleGravityOption()
{
	G->useNonSphericalGravity = !G->useNonSphericalGravity;
}

void OMPMFD::GetOMPError(char *buf, int err)
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
		sprintf_s(buf, 100, "Error: could not find DR constraint for NC maneuver");
	}
	else if (err == 8)
	{
		sprintf_s(buf, 100, "Error: could not find DH constraint for NH maneuver");
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