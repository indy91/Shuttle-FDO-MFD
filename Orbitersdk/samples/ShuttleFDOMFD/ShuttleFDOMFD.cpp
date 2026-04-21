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

// DEFINITIONS

// Maximum number of maneuvers shown on the Maneuver Constraints Table display
#define FDOMFD_MCT_MAX_MANEUVERS 7
// Maximum number of maneuvers shown on the Maneuver Evaluation Table display
#define FDOMFD_MET_MAX_MANEUVERS 6
// Cyan
#define COLOR_CYAN 0xffff00

// ==============================================================
// Global variables

ShuttleFDOoapiModule *g_coreMod;
int g_MFDmode; // identifier for new MFD mode
ShuttleFDOCore *GCoreData[32];
OBJHANDLE GCoreVessel[32];
int nGutsUsed;
std::vector<RTCCMFDData> g_MFDData;

// ==============================================================
// MFD class implementation

// Constructor
ShuttleFDOMFD::ShuttleFDOMFD(DWORD w, DWORD h, VESSEL *v, UINT im)
: MFD2 (w, h, v)
{
	ID = im;

	int hh = h;

	font = oapiCreateFont(w / 20, true, "Courier", FONT_NORMAL, 0);
	font2 = oapiCreateFont(w / 30, true, "Courier", FONT_NORMAL, 0);
	font3 = oapiCreateFont(w / 36, false, "fixed", FONT_NORMAL);
	font4 = oapiCreateFont(-(hh / 36), false, "fixed", FONT_NORMAL, 0);
	pen1 = oapiCreatePen(1, 1, 0x00FFFFFF);

	// Add MFD initialisation here
	G = NULL;
	screen = 0;
	subscreen = 0;
	marker = 0;
	markermax = 0;
	MTTFlag = false;
	MCTSelectedManeuver = 0;
	MCTScroll = 0;
	METScroll = 0;
	x = 0;
	dx = 0;
	y = 0;
	xmax = 0;
	ymax = 0;
	sprintf(Buffer, "");
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

	LoadState();
}

// Destructor
ShuttleFDOMFD::~ShuttleFDOMFD()
{
	oapiReleaseFont(font);
	oapiReleaseFont(font2);
	oapiReleaseFont(font3);
	oapiReleaseFont(font4);
	oapiReleasePen(pen1);

	SaveState();
}

void ShuttleFDOMFD::SaveState()
{
	RTCCMFDData temp;

	temp.ID = ID;
	temp.screen = screen;
	temp.subscreen = subscreen;
	temp.subscreenmax = subscreenmax;
	temp.marker = marker;
	temp.markermax = markermax;
	temp.MCTSelectedManeuver = MCTSelectedManeuver;
	temp.MCTScroll = MCTScroll;
	temp.METScroll = METScroll;

	bool found = false;

	//Search for existing MFD data
	for (unsigned i = 0; i < g_MFDData.size(); i++)
	{
		if (g_MFDData[i].ID == ID)
		{
			//Found it, save in that place
			g_MFDData[i] = temp;
			found = true;
			break;
		}
	}

	if (!found)
	{
		//Found in array yet, add it
		g_MFDData.push_back(temp);
	}
}

void ShuttleFDOMFD::LoadState()
{
	//Load MFD data

	for (unsigned i = 0; i < g_MFDData.size(); i++)
	{
		if (g_MFDData[i].ID == ID)
		{
			screen = g_MFDData[i].screen;
			subscreen = g_MFDData[i].subscreen;
			subscreenmax = g_MFDData[i].subscreenmax;
			marker = g_MFDData[i].marker;
			markermax = g_MFDData[i].markermax;
			MCTSelectedManeuver = g_MFDData[i].MCTSelectedManeuver;
			MCTScroll = g_MFDData[i].MCTScroll;
			METScroll = g_MFDData[i].METScroll;
		}
	}
}

void ShuttleFDOMFD::RecallStatus(void)
{
	//MFD data got reloaded in LoadState from the constructor, but resetting the MFD buttons crashes there. Do it here instead
	SetScreen(screen);
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
	// Draws the MFD title
	if (screen == 0)
	{
		Title(skp, "Shuttle FDO MFD");
	}
	skp->SetTextColor(GetDefaultColour(2)); //White
	skp->SetFont(font);
	GetCharSize(skp, CW, CH);
	//skp->SetTextAlign (oapi::Sketchpad::CENTER, oapi::Sketchpad::BASELINE);
	//skp->SetTextColor (0x00FFFF);

	// Add MFD display routines here.
	// Use the device context (hDC) for Windows GDI paint functions.

	switch (G->ErrorCode)
	{
	case 1:
		skp->Text(3 * W / 16, 13 * H / 14, "Launch day not initialized!", 27);
		break;
	}

	if (screen == 0)
	{
		skp->Text(1 * W / 16, 2 * H / 14, "Config", 6);
		skp->Text(1 * W / 16, 4 * H / 14, "Launch Window Processor", 23);
		skp->Text(1 * W / 16, 6 * H / 14, "Orbital Maneuver Processor", 26);
		skp->Text(1 * W / 16, 8 * H / 14, "Deorbit Opportunities", 21);
		skp->Text(1 * W / 16, 10 * H / 14, "Deorbit Planning", 16);
		skp->Text(1 * W / 16, 12 * H / 14, "MCC Displays", 12);
	}
	else if (screen == 1)
	{
		unsigned ii;

		skp->SetFont(font2);
		skp->SetTextAlign(oapi::Sketchpad::LEFT);
		skp->SetPen(pen1);

		xmax = 80;
		y = 3;
		ymax = 38;

		sprintf_s(Buffer, "MANEUVER CONSTRAINTS TABLE %s", G->MCT.Header.Name.c_str());
		Text(skp, 25 * W / xmax, y * H / ymax, Buffer);
		y += 2;
		sprintf_s(Buffer, "MNVRS: %2d / 40", G->MCT.Table.size());
		skp->Text(63 * W / xmax, y * H / ymax, Buffer, strlen(Buffer));
		y++;
		skp->Line(1 * W / 160, ((2 * y + 1) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 1) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 5) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 5) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 13) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 13) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 21) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 21) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 29) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 29) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 37) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 37) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 45) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 45) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 53) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 53) * H) / (ymax * 2));
		skp->Line(1 * W / 160, ((2 * y + 61) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 61) * H) / (ymax * 2));

		skp->Line(1 * W / 160, ((2 * y + 1) * H) / (ymax * 2), 1 * W / 160, ((2 * y + 61) * H) / (ymax * 2));
		skp->Line(29 * W / 160, ((2 * y + 1) * H) / (ymax * 2), 29 * W / 160, ((2 * y + 61) * H) / (ymax * 2));
		skp->Line(67 * W / 160, ((2 * y + 1) * H) / (ymax * 2), 67 * W / 160, ((2 * y + 61) * H) / (ymax * 2));
		skp->Line(159 * W / 160, ((2 * y + 1) * H) / (ymax * 2), 159 * W / 160, ((2 * y + 61) * H) / (ymax * 2));
		y++;
		Text(skp, 3 * W / xmax, y * H / ymax, "MANEUVER");
		Text(skp, 19 * W / xmax, y * H / ymax, "THRESHOLD");
		Text(skp, 50 * W / xmax, y * H / ymax, "SECONDARIES");
		y += 2;
		for (unsigned i = MCTScroll;i < G->MCT.Table.size();i++)
		{
			ii = i - MCTScroll;
			//MANEUVER
			if (i == MCTSelectedManeuver) skp->SetTextColor(GetDefaultColour(1));
			else skp->SetTextColor(GetDefaultColour(2));

			sprintf_s(Buffer, "%2d", i + 1);
			skp->Text(2 * W / xmax, (ii * 4 + y) * H / ymax, Buffer, strlen(Buffer));

			Text(skp, 5 * W / xmax, (ii * 4 + y) * H / ymax, OMP::GetOPMManeuverType(G->MCT.Table[i].type));

			sprintf_s(Buffer, 100, G->MCT.Table[i].name.c_str());
			skp->Text(2 * W / xmax, (ii * 4 + y + 1) * H / ymax, Buffer, strlen(Buffer));
			skp->SetTextColor(GetDefaultColour(2));
			//THRESHOLD
			Text(skp, 22 * W / xmax, (ii * 4 + y) * H / ymax, OMP::GetOPMManeuverThreshold(G->MCT.Table[i].threshold));

			skp->SetTextAlign(oapi::Sketchpad::CENTER);

			GetOPMManeuverThresholdTime(Buffer, G->MCT.Table[i].threshold, G->MCT.Table[i].thresh_num);
			skp->Text(23 * W / xmax, (ii * 4 + y + 1) * H / ymax, Buffer, strlen(Buffer));

			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			//SECONDARIES

			int k, l;

			for (unsigned j = 0;j < G->MCT.Table[i].secondaries.size();j++)
			{
				k = (j % 3);
				l = j / 3;

				GetOPMManeuverSecondary(Buffer, G->MCT.Table[i].secondaries[j].type, G->MCT.Table[i].secondaries[j].value);
				skp->Text((35 + k * 15) * W / xmax, (ii * 4 + y + l) * H / ymax, Buffer, strlen(Buffer));
			}

			//Limit number of maneuvers shown
			if (i >= (FDOMFD_MCT_MAX_MANEUVERS - 1) + MCTScroll) break;
		}

		if (G->subThreadStatus)
		{
			sprintf_s(Buffer, "Iterating...");
		}
		else
		{
			sprintf_s(Buffer, G->OMPErrorMessage.c_str());
		}
		skp->Text(2 * W / 32, 31 * H / 32, Buffer, strlen(Buffer));
	}
	else if (screen == 2)
	{
		skp->SetTextAlign(oapi::Sketchpad::LEFT);

		unsigned ii;

		skp->SetFont(font2);
		skp->SetPen(pen1);

		double hh, mm, ss;

		xmax = 80;
		y = 2;
		ymax = 36;

		sprintf_s(Buffer, "MANEUVER EVALUATION TABLE %s", G->ManeuverEvaluationTable.Name.c_str());
		Text(skp, 21 * W / xmax, y* H / ymax, Buffer);
		y += 2;
		sprintf_s(Buffer, "GMTR :");
		Text(skp, 21 * W / xmax, y* H / ymax, Buffer);
		sprintf_s(Buffer, "%04d:%03d:%02d:%02d:%06.3lf", G->sescnst.Year, G->sescnst.DayOfYear, G->sescnst.Hours, G->sescnst.Minutes, G->sescnst.launchdateSec);
		Text(skp, 28 * W / xmax, y* H / ymax, Buffer);
		y++;
		Text(skp, 2 * W / xmax, y* H / ymax, "Chaser");
		Text(skp, 10 * W / xmax, y* H / ymax, "DVtot =");
		Text(skp, 27 * W / xmax, y* H / ymax, "DVx =");
		Text(skp, 42 * W / xmax, y* H / ymax, "DVy =");
		Text(skp, 57 * W / xmax, y* H / ymax, "DVz =");
		skp->SetTextAlign(oapi::Sketchpad::RIGHT);
		sprintf_s(Buffer, "%.2lf", length(G->ManeuverEvaluationTable.dv_C));
		Text(skp, 26 * W / xmax, y* H / ymax, Buffer);
		sprintf_s(Buffer, "%.2lf", G->ManeuverEvaluationTable.dv_C.x);
		Text(skp, 41 * W / xmax, y* H / ymax, Buffer);
		sprintf_s(Buffer, "%.2lf", G->ManeuverEvaluationTable.dv_C.y);
		Text(skp, 56 * W / xmax, y* H / ymax, Buffer);
		sprintf_s(Buffer, "%.2lf", G->ManeuverEvaluationTable.dv_C.z);
		Text(skp, 71 * W / xmax, y* H / ymax, Buffer);
		skp->SetTextAlign(oapi::Sketchpad::LEFT);
		y++;
		Text(skp, 2 * W / xmax, y* H / ymax, "Target");
		Text(skp, 10 * W / xmax, y* H / ymax, "DVtot =");
		Text(skp, 27 * W / xmax, y* H / ymax, "DVx =");
		Text(skp, 42 * W / xmax, y* H / ymax, "DVy =");
		Text(skp, 57 * W / xmax, y* H / ymax, "DVz =");
		Text(skp, 75 * W / xmax, y* H / ymax, "Mvrs");
		sprintf_s(Buffer, "%d", G->ManeuverEvaluationTable.Maneuvers.size());
		Text(skp, 72 * W / xmax, y* H / ymax, Buffer);
		skp->SetTextAlign(oapi::Sketchpad::RIGHT);
		sprintf_s(Buffer, "%.2lf", length(G->ManeuverEvaluationTable.dv_T));
		Text(skp, 26 * W / xmax, y* H / ymax, Buffer);
		sprintf_s(Buffer, "%.2lf", G->ManeuverEvaluationTable.dv_T.x);
		Text(skp, 41 * W / xmax, y* H / ymax, Buffer);
		sprintf_s(Buffer, "%.2lf", G->ManeuverEvaluationTable.dv_T.y);
		Text(skp, 56 * W / xmax, y* H / ymax, Buffer);
		sprintf_s(Buffer, "%.2lf", G->ManeuverEvaluationTable.dv_T.z);
		Text(skp, 71 * W / xmax, y* H / ymax, Buffer);
		skp->SetTextAlign(oapi::Sketchpad::LEFT);
		y++;
		for (ii = 0; ii < 8; ii++)
		{
			skp->Line(1 * W / (xmax * 2), (y * 2 + ii * 8 + 1)* H / (ymax * 2), 159 * W / (xmax * 2), (y * 2 + ii * 8 + 1)* H / (ymax * 2));
		}
		skp->Line(1 * W / (xmax * 2), (y * 2 + 1)* H / (ymax * 2), 1 * W / (xmax * 2), (y * 2 + 1 + 8 * 7)* H / (ymax * 2));
		skp->Line(27 * W / (xmax * 2), (y * 2 + 1)* H / (ymax * 2), 27 * W / (xmax * 2), (y * 2 + 1 + 8 * 7)* H / (ymax * 2));
		skp->Line(65 * W / (xmax * 2), (y * 2 + 1)* H / (ymax * 2), 65 * W / (xmax * 2), (y * 2 + 1 + 8 * 7)* H / (ymax * 2));
		skp->Line(85 * W / (xmax * 2), (y * 2 + 1)* H / (ymax * 2), 85 * W / (xmax * 2), (y * 2 + 1 + 8 * 7)* H / (ymax * 2));
		skp->Line(105 * W / (xmax * 2), (y * 2 + 1)* H / (ymax * 2), 105 * W / (xmax * 2), (y * 2 + 1 + 8 * 7)* H / (ymax * 2));
		skp->Line(131 * W / (xmax * 2), (y * 2 + 1)* H / (ymax * 2), 131 * W / (xmax * 2), (y * 2 + 1 + 8 * 7)* H / (ymax * 2));
		skp->Line(159 * W / (xmax * 2), (y * 2 + 1)* H / (ymax * 2), 159 * W / (xmax * 2), (y * 2 + 1 + 8 * 7)* H / (ymax * 2));
		y++;
		Text(skp, 2 * W / xmax, y* H / ymax, "Mnvr Name");
		Text(skp, 20 * W / xmax, y* H / ymax, "GMTIG  IMP");
		Text(skp, 36 * W / xmax, y* H / ymax, "DVX");
		Text(skp, 47 * W / xmax, y* H / ymax, "HA");
		Text(skp, 57 * W / xmax, y* H / ymax, "RANGE");
		Text(skp, 73 * W / xmax, y* H / ymax, "Y");
		y++;
		Text(skp, 3 * W / xmax, y* H / ymax, "Comment");
		Text(skp, 20 * W / xmax, y* H / ymax, "METIG");
		Text(skp, 36 * W / xmax, y* H / ymax, "DVY");
		Text(skp, 47 * W / xmax, y* H / ymax, "HP");
		Text(skp, 57 * W / xmax, y* H / ymax, "PHASE");
		Text(skp, 72 * W / xmax, y* H / ymax, "Ydot");
		y++;
		Text(skp, 3 * W / xmax, y* H / ymax, "DVMag");
		Text(skp, 21 * W / xmax, y* H / ymax, "DT");
		Text(skp, 36 * W / xmax, y* H / ymax, "DVZ");
		Text(skp, 47 * W / xmax, y* H / ymax, "DH");
		Text(skp, 54 * W / xmax, y* H / ymax, "Noon/Mid -");
		Text(skp, 70 * W / xmax, y* H / ymax, "SR/SS -");
		y += 2;
		skp->SetTextAlign(oapi::Sketchpad::RIGHT);
		for (unsigned i = METScroll; i < G->ManeuverEvaluationTable.Maneuvers.size(); i++)
		{
			ii = i - METScroll;

			sprintf_s(Buffer, "%d", i + 1);
			Text(skp, 4 * W / xmax, (y + ii * 4)* H / ymax, Buffer);

			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			sprintf_s(Buffer, G->ManeuverEvaluationTable.Maneuvers[i].type.c_str());
			Text(skp, 5 * W / xmax, (y + ii * 4)* H / ymax, Buffer);

			sprintf_s(Buffer, G->ManeuverEvaluationTable.Maneuvers[i].name.c_str());
			Text(skp, 2 * W / xmax, (y + ii * 4 + 1)* H / ymax, Buffer);
			skp->SetTextAlign(oapi::Sketchpad::RIGHT);

			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable.Maneuvers[i].DVMag);
			Text(skp, 9 * W / xmax, (y + ii * 4 + 2)* H / ymax, Buffer);

			GMT2String(Buffer, G->ManeuverEvaluationTable.Maneuvers[i].GMTIG);
			Text(skp, 31 * W / xmax, (y + ii * 4)* H / ymax, Buffer);
			MET2String(Buffer, G->ManeuverEvaluationTable.Maneuvers[i].METIG);
			Text(skp, 31 * W / xmax, (y + ii * 4 + 1)* H / ymax, Buffer);
			MET2String(Buffer, G->ManeuverEvaluationTable.Maneuvers[i].DT);
			Text(skp, 31 * W / xmax, (y + ii * 4 + 2)* H / ymax, Buffer);

			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable.Maneuvers[i].DV.x);
			Text(skp, 41 * W / xmax, (y + ii * 4 + 0)* H / ymax, Buffer);
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable.Maneuvers[i].DV.y);
			Text(skp, 41 * W / xmax, (y + ii * 4 + 1)* H / ymax, Buffer);
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable.Maneuvers[i].DV.z);
			Text(skp, 41 * W / xmax, (y + ii * 4 + 2)* H / ymax, Buffer);

			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable.Maneuvers[i].HA);
			Text(skp, 51 * W / xmax, (y + ii * 4)* H / ymax, Buffer);
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable.Maneuvers[i].HP);
			Text(skp, 51 * W / xmax, (y + ii * 4 + 1)* H / ymax, Buffer);
			sprintf_s(Buffer, "%.2f", G->ManeuverEvaluationTable.Maneuvers[i].DH);
			Text(skp, 51 * W / xmax, (y + ii * 4 + 2)* H / ymax, Buffer);

			sprintf_s(Buffer, "%.4f", G->ManeuverEvaluationTable.Maneuvers[i].RANGE);
			Text(skp, 64 * W / xmax, (y + ii * 4)* H / ymax, Buffer);
			sprintf_s(Buffer, "%.4f", G->ManeuverEvaluationTable.Maneuvers[i].PHASE);
			Text(skp, 64 * W / xmax, (y + ii * 4 + 1) * H / ymax, Buffer);
			SS2HHMMSS(G->ManeuverEvaluationTable.Maneuvers[i].TTN, hh, mm, ss);
			if (G->ManeuverEvaluationTable.Maneuvers[i].noon)
			{
				sprintf_s(Buffer, "N-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer, "M-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			Text(skp, 64 * W / xmax, (y + ii * 4 + 2) * H / ymax, Buffer);

			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable.Maneuvers[i].Y);
			Text(skp, 78 * W / xmax, (y + ii * 4) * H / ymax, Buffer);
			sprintf_s(Buffer, "%.1f", G->ManeuverEvaluationTable.Maneuvers[i].Ydot);
			Text(skp, 78 * W / xmax, (y + ii * 4 + 1) * H / ymax, Buffer);
			SS2HHMMSS(G->ManeuverEvaluationTable.Maneuvers[i].TTS, hh, mm, ss);
			if (G->ManeuverEvaluationTable.Maneuvers[i].sunrise)
			{
				sprintf_s(Buffer, "SR-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			else
			{
				sprintf_s(Buffer, "SS-%02.0f:%02.0f:%02.0f", hh, mm, ss);
			}
			Text(skp, 78 * W / xmax, (y + ii * 4 + 2) * H / ymax, Buffer);

			//Only display 6 maneuvers at once
			if (i >= (FDOMFD_MET_MAX_MANEUVERS - 1) + METScroll) break;
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
		}
		else if (G->LWP_LaunchSite == 2)
		{
			sprintf_s(Buffer, "39B");
		}
		else if (G->LWP_LaunchSite == 3)
		{
			sprintf_s(Buffer, "SLC-6");
		}
		else
		{
			sprintf_s(Buffer, "Manual");
		}
		skp->Text(1 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.4f° %.4f°", G->LWP_Settings.LATLS*DEG, G->LWP_Settings.LONGLS*DEG);
		skp->Text(1 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		if (G->LWP_Settings.NS == 0)
		{
			sprintf_s(Buffer, "North");
		}
		else
		{
			sprintf_s(Buffer, "South");
		}
		skp->Text(1 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f°", G->LWP_Settings.YSMAX*DEG);
		skp->Text(1 * W / 8, 10 * H / 14, Buffer, strlen(Buffer));


		sprintf_s(Buffer, "%.3f°", G->LWP_Settings.PFA*DEG);
		skp->Text(5 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));

		SS2MMSS(G->LWP_Settings.PFT, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(5 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f NM", (G->LWP_Settings.RINS - OrbMech::EARTH_RADIUS_EQUATOR) / 1852.0);
		skp->Text(5 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f fps", G->LWP_Settings.VINS / FPS2MPS);
		skp->Text(5 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f°", G->LWP_Settings.GAMINS*DEG);
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
		skp->Text(7 * W / 64, 2 * H / 32, Buffer, strlen(Buffer));
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

		for (unsigned i = 0; i < 21; i++)
		{
			sprintf_s(Buffer, "%d", i + 1);
			skp->Text(1 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
		}

		for (unsigned i = 0;i < G->ManeuverTransferTable.size();i++)
		{
			sprintf_s(Buffer, G->ManeuverTransferTable[i].NAME.c_str());
			skp->Text(3 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, G->ManeuverTransferTable[i].COMMENT.c_str());
			skp->Text(7 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			sprintf_s(Buffer, "%d", G->ManeuverTransferTable[i].SLOT);
			skp->Text(12 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			GetMTTThrusterType(Buffer, G->ManeuverTransferTable[i].thrusters);
			skp->Text(14 * W / 32, (i + 4) * H / 32, Buffer, strlen(Buffer));
			G->GetMTTGuidanceType(Buffer, G->ManeuverTransferTable[i].guid);
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
			G->GetMTTGuidanceType(Buffer, G->MTTSlotData[i].guid);
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
		skp->Text(14 * W / 32, 1 * H / 32, "Config", 6);

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

		sprintf(Buffer, "%04d:%03d:%02d:%02d:%06.3f", G->sescnst.Year, G->sescnst.DayOfYear, G->sescnst.Hours, G->sescnst.Minutes, G->sescnst.launchdateSec);
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

		SS2MMSS(G->LWP_Settings.DTIG_ET_SEP, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(1 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f %.1f %.1f", G->LWP_Settings.DV_ET_SEP.x / FPS2MPS, G->LWP_Settings.DV_ET_SEP.y / FPS2MPS, G->LWP_Settings.DV_ET_SEP.z / FPS2MPS);
		skp->Text(1 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));

		SS2MMSS(G->LWP_Settings.DTIG_MPS, mm, ss);
		sprintf_s(Buffer, "% 03.0f:%04.1f", mm, ss);
		skp->Text(1 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f %.1f %.1f", G->LWP_Settings.DV_MPS.x / FPS2MPS, G->LWP_Settings.DV_MPS.y / FPS2MPS, G->LWP_Settings.DV_MPS.z / FPS2MPS);
		skp->Text(1 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		if (G->LWP_Settings.DirectInsertion)
		{
			skp->Text(1 * W / 8, 10 * H / 14, "Direct Insertion", 16);
		}
		else
		{
			skp->Text(1 * W / 8, 10 * H / 14, "Standard Insertion", 18);
		}

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
		LWPGMT2String(Buffer, G->LWP_Output.GMTOPT);
		skp->Text(4 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));
		skp->Text(2 * W / 8, 3 * H / 14, "PHASE", 5);
		sprintf_s(Buffer, "%.1f°", G->LWP_Output.PA_GMTOPT);
		skp->Text(4 * W / 8, 3 * H / 14, Buffer, strlen(Buffer));	

		skp->Text(1 * W / 16, 5 * H / 14, "PLANAR OPEN", 11);
		skp->Text(1 * W / 16, 6 * H / 14, "L/O", 3);
		skp->Text(1 * W / 16, 7 * H / 14, "PHASE", 5);
		LWPGMT2String(Buffer, G->LWP_Output.GMTPO);
		skp->Text(3 * W / 16, 6 * H / 14, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f°", G->LWP_Output.PA_GMTPO);
		skp->Text(5 * W / 16, 7 * H / 14, Buffer, strlen(Buffer));

		skp->Text(4 * W / 8, 5 * H / 14, "PLANAR CLOSE", 12);
		skp->Text(4 * W / 8, 6 * H / 14, "L/O", 3);
		skp->Text(4 * W / 8, 7 * H / 14, "PHASE", 5);
		LWPGMT2String(Buffer, G->LWP_Output.GMTPC);
		skp->Text(5 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%.1f°", G->LWP_Output.PA_GMTPC);
		skp->Text(11 * W / 16, 7 * H / 14, Buffer, strlen(Buffer));

		if (G->LWP_Output.LWPERROR)
		{
			GetLWPError(Buffer, G->LWP_Output.LWPERROR);
			skp->Text(2 * W / 32, 12 * H / 14, Buffer, strlen(Buffer));
		}
	}
	else if (screen == 9)
	{
		skp->SetFont(font2);
		skp->SetTextAlign(oapi::Sketchpad::CENTER);
		skp->Text(1 * W / 2, 2 * H / 36, "DEORBIT OPPORTUNITIES TABLE (DOT)", 33);
		skp->SetTextAlign(oapi::Sketchpad::LEFT);
		skp->Text(1 * W / 16, 4 * H / 36, "REV", 3);
		sprintf(Buffer, "%d", G->DOPS_InitialRev);
		skp->Text(2 * W / 16, 4 * H / 36, Buffer, strlen(Buffer));

		skp->Text(7 * W / 32, 4 * H / 36, "GETS", 4);
		DMTMET2String(Buffer, G->DOPS_GETS);
		skp->Text(5 * W / 16, 4 * H / 36, Buffer, strlen(Buffer));

		skp->Text(17 * W / 32, 4 * H / 36, "GETF", 4);
		DMTMET2String(Buffer, G->DOPS_GETF);
		skp->Text(20 * W / 32, 4 * H / 36, Buffer, strlen(Buffer));

		skp->Text(27 * W / 32, 4 * H / 36, "XRNG", 4);
		sprintf_s(Buffer, "%.0f", G->DOPS_MaxXRNG);
		skp->Text(15 * W / 16, 4 * H / 36, Buffer, strlen(Buffer));

		skp->Text(1 * W / 16, 5 * H / 36, "SITES", 5);
		if (G->DOPS_ConUS)
		{
			sprintf_s(Buffer, "Continental US");
		}
		else
		{
			sprintf_s(Buffer, "All Sites");
		}
		skp->Text(3 * W / 16, 5 * H / 36, Buffer, strlen(Buffer));

		skp->Text(12 * W / 16, 5 * H / 36, "PAGE", 4);
		sprintf_s(Buffer, "%d/%d", G->DOPS_Page + 1, G->DOPS_MaxPage + 1);
		skp->Text(14 * W / 16, 5 * H / 36, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::CENTER);

		skp->Text(1 * W / 16, 7 * H / 36, "TIG", 3);
		skp->Text(1 * W / 16, 8 * H / 36, "ORB", 3);

		skp->Text(3 * W / 16, 7 * H / 36, "SITE", 4);

		skp->Text(5 * W / 16, 7 * H / 36, "TIG", 3);
		skp->Text(5 * W / 16, 8 * H / 36, "MET", 3);

		skp->Text(17 * W / 32, 7 * H / 36, "LANDING", 7);
		skp->Text(15 * W / 32, 8 * H / 36, "MET", 3);
		skp->Text(19 * W / 32, 8 * H / 36, "GMT", 3);
		skp->Text(23 * W / 32, 8 * H / 36, "LIGHT", 5);

		skp->Text(14 * W / 16, 7 * H / 36, "XRNG", 4);

		unsigned j;
		for (unsigned i = 0; i < 25; i++)
		{
			j = i + 25U * (unsigned)G->DOPS_Page;
			if (j >= G->DODS_Output.data.size()) break;

			sprintf(Buffer, "%d", G->DODS_Output.data[j].Rev);
			skp->Text(1 * W / 16, (9 + i) * H / 36, Buffer, strlen(Buffer));

			sprintf(Buffer, G->DODS_Output.data[j].Site.c_str());
			skp->Text(3 * W / 16, (9 + i) * H / 36, Buffer, strlen(Buffer));

			MET2String2(Buffer, G->DODS_Output.data[j].TIG_MET);
			skp->Text(5 * W / 16, (9 + i) * H / 36, Buffer, strlen(Buffer));

			MET2String2(Buffer, G->DODS_Output.data[j].Landing_MET);
			skp->Text(15 * W / 32, (9 + i) * H / 36, Buffer, strlen(Buffer));

			GMT2String2(Buffer, G->DODS_Output.data[j].Landing_GMT);
			skp->Text(19 * W / 32, (9 + i) * H / 36, Buffer, strlen(Buffer));

			sprintf(Buffer, G->DODS_Output.data[j].T_Light.c_str());
			skp->Text(23 * W / 32, (9 + i) * H / 36, Buffer, strlen(Buffer));

			sprintf(Buffer, G->DODS_Output.data[j].XRNG.c_str());
			skp->Text(14 * W / 16, (9 + i) * H / 36, Buffer, strlen(Buffer));
		}
	}
	else if (screen == 10)
	{
		skp->Text(1 * W / 4, 2 * H / 36, "DEORBIT MANEUVER PLANNING", 25);
		if (G->DMPOpt.ITIGFR == 1)
		{
			sprintf(Buffer, "TIG free");
		}
		else
		{
			sprintf(Buffer, "TIG fixed");
		}
		skp->Text(1 * W / 8, 2 * H / 14, Buffer, strlen(Buffer));

		if (G->DMPOpt.ITIGFR == 1)
		{
			DMTMET2String(Buffer, G->DMPOpt.TTHRSH);
		}
		else
		{
			DMTMET2String(Buffer, G->DMPOpt.TIG);
		}
		skp->Text(1 * W / 8, 4 * H / 14, Buffer, strlen(Buffer));

		if (G->DMPOpt.WCGOMS != 0.0)
		{
			sprintf(Buffer, "Propellant Wasting: %.0lf lbs", G->DMPOpt.WCGOMS / LBM2KG);
		}
		else
		{
			sprintf(Buffer, "In-Plane");
		}
		skp->Text(1 * W / 8, 6 * H / 14, Buffer, strlen(Buffer));

		if (G->DMPOpt.INGPR == 12)
		{
			sprintf(Buffer, "RCS");
		}
		else if (G->DMPOpt.INGPR == 14)
		{
			sprintf(Buffer, "1OMS");
		}
		else
		{
			sprintf(Buffer, "2OMS");
		}
		skp->Text(1 * W / 8, 8 * H / 14, Buffer, strlen(Buffer));

		if (G->DMPOpt.INGBU == 12)
		{
			sprintf(Buffer, "RCS");
		}
		else if (G->DMPOpt.INGBU == 14)
		{
			sprintf(Buffer, "1OMS");
		}
		else
		{
			sprintf(Buffer, "2OMS");
		}
		skp->Text(1 * W / 8, 10 * H / 14, Buffer, strlen(Buffer));

		sprintf(Buffer, "%s", G->DMPLandingSite.c_str());
		skp->Text(1 * W / 8, 12 * H / 14, Buffer, strlen(Buffer));
	}
	else if (screen == 11)
	{
		skp->SetFont(font2);
		skp->SetTextAlign(oapi::Sketchpad::CENTER);
		skp->Text(1 * W / 2, 2 * H / 36, "LAUNCH TARGETING PROCESSOR OUTPUT", 33);

		skp->Text(10 * W / 32, 4 * H / 36, "GMT L/O", 7);

		skp->Text(8 * W / 32, 6 * H / 36, "CHASER", 6);
		skp->Text(8 * W / 32, 7 * H / 36, "MECO", 4);
		skp->Text(8 * W / 32, 19 * H / 36, "MPS DUMP", 8);
		skp->Text(24 * W / 32, 6 * H / 36, "OMS-2", 5);
		skp->Text(24 * W / 32, 18 * H / 36, "TGT (AT ASCN)", 13);

		skp->SetTextAlign(oapi::Sketchpad::RIGHT);

		skp->Text(4 * W / 32, 9 * H / 36, "MECO", 4);
		skp->Text(4 * W / 32, 10 * H / 36, "VMECO", 5);
		skp->Text(4 * W / 32, 11 * H / 36, "RMECO", 5);
		skp->Text(4 * W / 32, 12 * H / 36, "GMECO", 5);
		skp->Text(4 * W / 32, 13 * H / 36, "IMECO", 5);
		skp->Text(4 * W / 32, 14 * H / 36, "PHASE", 5);
		skp->Text(4 * W / 32, 15 * H / 36, "HA", 2);
		skp->Text(4 * W / 32, 16 * H / 36, "HP", 2);
		skp->Text(4 * W / 32, 17 * H / 36, "LONG", 4);

		skp->Text(4 * W / 32, 21 * H / 36, "TIG", 3);
		skp->Text(4 * W / 32, 22 * H / 36, "DELTA V", 7);
		skp->Text(4 * W / 32, 23 * H / 36, "HA", 2);
		skp->Text(4 * W / 32, 24 * H / 36, "HP", 2);

		skp->Text(20 * W / 32, 9 * H / 36, "TIG", 3);
		skp->Text(20 * W / 32, 10 * H / 36, "DELTA V", 7);
		skp->Text(20 * W / 32, 11 * H / 36, "HA", 2);
		skp->Text(20 * W / 32, 12 * H / 36, "HP", 2);
		skp->Text(20 * W / 32, 14 * H / 36, "NODE", 4);
		skp->Text(20 * W / 32, 15 * H / 36, "PHASE", 5);
		skp->Text(20 * W / 32, 16 * H / 36, "PERIOD", 6);

		skp->Text(20 * W / 32, 20 * H / 36, "HA", 2);
		skp->Text(20 * W / 32, 21 * H / 36, "HP", 2);
		skp->Text(20 * W / 32, 23 * H / 36, "LONG", 4);
		skp->Text(20 * W / 32, 24 * H / 36, "DELN", 4);
		skp->Text(20 * W / 32, 25 * H / 36, "PERIOD", 6);

		skp->SetTextAlign(oapi::Sketchpad::LEFT);

		skp->Text(14 * W / 32, 10 * H / 36, "fps", 3);
		skp->Text(14 * W / 32, 11 * H / 36, "nm", 2);
		skp->Text(14 * W / 32, 12 * H / 36, "deg", 3);
		skp->Text(14 * W / 32, 13 * H / 36, "deg", 3);
		skp->Text(14 * W / 32, 14 * H / 36, "deg", 3);
		skp->Text(14 * W / 32, 15 * H / 36, "nm", 2);
		skp->Text(14 * W / 32, 16 * H / 36, "nm", 2);
		skp->Text(14 * W / 32, 17 * H / 36, "deg", 3);
		skp->Text(14 * W / 32, 22 * H / 36, "fps", 3);
		skp->Text(14 * W / 32, 23 * H / 36, "nm", 2);
		skp->Text(14 * W / 32, 24 * H / 36, "nm", 2);

		skp->Text(30 * W / 32, 10 * H / 36, "fps", 3);
		skp->Text(30 * W / 32, 11 * H / 36, "nm", 2);
		skp->Text(30 * W / 32, 12 * H / 36, "nm", 2);
		skp->Text(30 * W / 32, 14 * H / 36, "deg", 3);
		skp->Text(30 * W / 32, 15 * H / 36, "deg", 3);
		skp->Text(30 * W / 32, 20 * H / 36, "nm", 2);
		skp->Text(30 * W / 32, 21 * H / 36, "nm", 2);
		skp->Text(30 * W / 32, 23 * H / 36, "deg", 3);
		skp->Text(30 * W / 32, 24 * H / 36, "deg", 3);

		skp->SetTextAlign(oapi::Sketchpad::RIGHT);

		LTPGMT2String(Buffer, G->LWP_Settings.GMTLOR);
		skp->Text(22 * W / 32, 4 * H / 36, Buffer, strlen(Buffer));

		double hh, mm, ss;

		SS2HHMMSS(G->LTP_Output.MET_MECO, hh, mm, ss);
		sprintf_s(Buffer, "%02.0f:%04.1f", mm, ss);
		skp->Text(13 * W / 32, 9 * H / 36, Buffer, strlen(Buffer));

		sprintf(Buffer, "%.1lf", G->LTP_Output.V_MECO);
		skp->Text(13 * W / 32, 10 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.4lf", G->LTP_Output.R_MECO);
		skp->Text(13 * W / 32, 11 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.4lf", G->LTP_Output.G_MECO);
		skp->Text(13 * W / 32, 12 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.3lf", G->LTP_Output.I_MECO);
		skp->Text(13 * W / 32, 13 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.PHASE_MECO);
		skp->Text(13 * W / 32, 14 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.HA_MECO);
		skp->Text(13 * W / 32, 15 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.HP_MECO);
		skp->Text(13 * W / 32, 16 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.LONG_MECO);
		skp->Text(13 * W / 32, 17 * H / 36, Buffer, strlen(Buffer));

		SS2HHMMSS(G->LTP_Output.TIG_MPS, hh, mm, ss);
		sprintf_s(Buffer, "%02.0f:%04.1f", mm, ss);
		skp->Text(13 * W / 32, 21 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.DV_MPS);
		skp->Text(13 * W / 32, 22 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.HA_MPS);
		skp->Text(13 * W / 32, 23 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.HP_MPS);
		skp->Text(13 * W / 32, 24 * H / 36, Buffer, strlen(Buffer));

		SS2HHMMSS(G->LTP_Output.TIG_OMS2, hh, mm, ss);
		sprintf_s(Buffer, "%02.0f:%04.1f", mm, ss);
		skp->Text(29 * W / 32, 9 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.DV_OMS2);
		skp->Text(29 * W / 32, 10 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.HA_OMS2);
		skp->Text(29 * W / 32, 11 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.HP_OMS2);
		skp->Text(29 * W / 32, 12 * H / 36, Buffer, strlen(Buffer));

		sprintf(Buffer, "%.1lf", G->LTP_Output.NODE_OMS2);
		skp->Text(29 * W / 32, 14 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.PHASE_OMS2);
		skp->Text(29 * W / 32, 15 * H / 36, Buffer, strlen(Buffer));
		SS2HHMMSS(G->LTP_Output.PERIOD_OMS2, hh, mm, ss);
		sprintf_s(Buffer, "%.0lf:%02.0f:%04.1f", hh, mm, ss);
		skp->Text(29 * W / 32, 16 * H / 36, Buffer, strlen(Buffer));

		//TGT
		sprintf(Buffer, "%.1lf", G->LTP_Output.HA_TGT);
		skp->Text(29 * W / 32, 20 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LTP_Output.HP_TGT);
		skp->Text(29 * W / 32, 21 * H / 36, Buffer, strlen(Buffer));

		sprintf(Buffer, "%.1lf", G->LTP_Output.LONG_TGT);
		skp->Text(29 * W / 32, 23 * H / 36, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.6lf", G->LTP_Output.DELN);
		skp->Text(29 * W / 32, 24 * H / 36, Buffer, strlen(Buffer));
		SS2HHMMSS(G->LTP_Output.PERIOD_TGT, hh, mm, ss);
		sprintf_s(Buffer, "%.0lf:%02.0f:%04.1f", hh, mm, ss);
		skp->Text(29 * W / 32, 25 * H / 36, Buffer, strlen(Buffer));

		if (G->LWP_Output.LWPERROR)
		{
			GetLWPError(Buffer, G->LWP_Output.LWPERROR);
			skp->Text(2 * W / 4, 12 * H / 14, Buffer, strlen(Buffer));
		}
	}
	else if (screen == 12)
	{
		skp->SetTextAlign(oapi::Sketchpad::CENTER);
		skp->Text(1 * W / 2, 2 * H / 36, "LNCH REF TGT SET", 16);
		skp->SetTextAlign(oapi::Sketchpad::LEFT);

		skp->Text(1 * W / 16, 2 * H / 14, "OMS-1 TGTS", 10);
		MET2String(Buffer, G->LWP_Settings.OMS1.DTIG);
		skp->Text(1 * W / 16, 4 * H / 14, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf %.1lf", G->LWP_Settings.OMS1.C1 / 0.3048, G->LWP_Settings.OMS1.C2);
		skp->Text(1 * W / 16, 6 * H / 14, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LWP_Settings.OMS1.HTGT / 1852.0);
		skp->Text(1 * W / 16, 8 * H / 14, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LWP_Settings.OMS1.THETA*DEG);
		skp->Text(1 * W / 16, 10 * H / 14, Buffer, strlen(Buffer));

		skp->Text(9 * W / 16, 2 * H / 14, "OMS-2 TGTS", 10);
		MET2String(Buffer, G->LWP_Settings.OMS2.DTIG);
		skp->Text(9 * W / 16, 4 * H / 14, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf %.1lf", G->LWP_Settings.OMS2.C1 / 0.3048, G->LWP_Settings.OMS2.C2);
		skp->Text(9 * W / 16, 6 * H / 14, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LWP_Settings.OMS2.HTGT / 1852.0);
		skp->Text(9 * W / 16, 8 * H / 14, Buffer, strlen(Buffer));
		sprintf(Buffer, "%.1lf", G->LWP_Settings.OMS2.THETA*DEG);
		skp->Text(9 * W / 16, 10 * H / 14, Buffer, strlen(Buffer));
	}
	else if (screen == 13)
	{
		skp->SetTextAlign(oapi::Sketchpad::CENTER);
		skp->Text(1 * W / 2, 2 * H / 36, "DEORBIT TARGETING SOLUTION", 26);
		skp->SetTextAlign(oapi::Sketchpad::LEFT);

		skp->SetFont(font2);

		skp->Text(1 * W / 16, 7 * H / 32, "SITE", 4);
		skp->Text(1 * W / 16, 8 * H / 32, "TIGMET", 6);
		skp->Text(1 * W / 16, 9 * H / 32, "C1", 2);
		skp->Text(1 * W / 16, 10 * H / 32, "C2", 2);
		skp->Text(1 * W / 16, 11 * H / 32, "HT", 2);
		skp->Text(1 * W / 16, 12 * H / 32, "THETAT", 6);
		skp->Text(1 * W / 16, 13 * H / 32, "PL", 2);

		skp->Text(10 * W / 16, 9 * H / 32, "DVX", 3);
		skp->Text(10 * W / 16, 10 * H / 32, "DVY", 3);
		skp->Text(10 * W / 16, 11 * H / 32, "DVZ", 3);

		skp->Text(3 * W / 16, 7 * H / 32, G->DMPRes.Site.c_str(), G->DMPRes.Site.size());

		DMTMET2String(Buffer, G->DMPRes.TIG);
		skp->Text(3 * W / 16, 8 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.0f", G->DMPRes.C1);
		skp->Text(3 * W / 16, 9 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%+.4f", G->DMPRes.C2);
		skp->Text(3 * W / 16, 10 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f", G->DMPRes.EIALT);
		skp->Text(3 * W / 16, 11 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.3f", G->DMPRes.THETEI);
		skp->Text(3 * W / 16, 12 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.0f", G->DMPRes.WCG);
		skp->Text(3 * W / 16, 13 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%+.1f", G->DMPRes.VGO.x);
		skp->Text(12 * W / 16, 9 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%+.1f", G->DMPRes.VGO.y);
		skp->Text(12 * W / 16, 10 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%+.1f", G->DMPRes.VGO.z);
		skp->Text(12 * W / 16, 11 * H / 32, Buffer, strlen(Buffer));

		skp->Text(1 * W / 16, 15 * H / 32, "DVPR", 4);
		skp->Text(1 * W / 16, 16 * H / 32, "VEI", 3);
		skp->Text(1 * W / 16, 17 * H / 32, "cEI", 3);
		skp->Text(1 * W / 16, 18 * H / 32, "REI", 3);
		skp->Text(1 * W / 16, 19 * H / 32, "XR", 2);
		skp->Text(1 * W / 16, 20 * H / 32, "OOP", 3);
		skp->Text(1 * W / 16, 21 * H / 32, "TFF", 3);
		skp->Text(1 * W / 16, 22 * H / 32, "HP", 2);
		skp->Text(1 * W / 16, 23 * H / 32, "DW", 2);

		sprintf_s(Buffer, "%.1f", G->DMPRes.DVPR);
		skp->Text(3 * W / 16, 15 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.0f", G->DMPRes.VEI);
		skp->Text(3 * W / 16, 16 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.2f", G->DMPRes.cEI);
		skp->Text(3 * W / 16, 17 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.0f", G->DMPRes.REI);
		skp->Text(3 * W / 16, 18 * H / 32, Buffer, strlen(Buffer));

		skp->Text(3 * W / 16, 19 * H / 32, G->DMPRes.XR.c_str(), G->DMPRes.XR.size());

		sprintf_s(Buffer, "%.1f", G->DMPRes.OOP);
		skp->Text(3 * W / 16, 20 * H / 32, Buffer, strlen(Buffer));

		double mm, ss;
		SS2MMSS(G->DMPRes.TFF, mm, ss);
		sprintf_s(Buffer, "%02.0lf:%02.0lf", mm, ss);
		skp->Text(3 * W / 16, 21 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.1f", G->DMPRes.HP);
		skp->Text(3 * W / 16, 22 * H / 32, Buffer, strlen(Buffer));

		sprintf_s(Buffer, "%.0f", G->DMPRes.DW);
		skp->Text(3 * W / 16, 23 * H / 32, Buffer, strlen(Buffer));

		skp->Text(10 * W / 16, 13 * H / 32, "EI - 5 MM303 INRTL ATT", 22);
		skp->Text(10 * W / 16, 14 * H / 32, "R", 1);
		skp->Text(10 * W / 16, 15 * H / 32, "P", 1);
		skp->Text(10 * W / 16, 16 * H / 32, "Y", 1);

		sprintf_s(Buffer, "%03.0f", G->DMPRes.EIminus5Att.x);
		skp->Text(12 * W / 16, 14 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%03.0f", G->DMPRes.EIminus5Att.y);
		skp->Text(12 * W / 16, 15 * H / 32, Buffer, strlen(Buffer));
		sprintf_s(Buffer, "%03.0f", G->DMPRes.EIminus5Att.z);
		skp->Text(12 * W / 16, 16 * H / 32, Buffer, strlen(Buffer));

		skp->SetTextAlign(oapi::Sketchpad::CENTER);
		skp->Text(1 * W / 2, 31 * H / 32, G->DMPRes.ErrorMessage.c_str(), G->DMPRes.ErrorMessage.length());
	}
	else if (screen == 14)
	{
		skp->Text(1 * W / 16, 2 * H / 14, "Maneuver Constraints Table", 27);
		skp->Text(1 * W / 16, 4 * H / 14, "Maneuver Evaluation Table", 25);
		skp->Text(1 * W / 16, 6 * H / 14, "Maneuver Transfer Table", 23);
		skp->Text(1 * W / 16, 8 * H / 14, "Detailed Maneuver Table", 23);
	}
	else if (screen == 15)
	{
		skp->Text(1 * W / 16, 2 * H / 14, "Supersighter Display", 20);
		skp->Text(1 * W / 16, 4 * H / 14, "Instrument Definition Table", 27);
		skp->Text(1 * W / 16, 6 * H / 14, "Ground Targets", 14);
		skp->Text(1 * W / 16, 8 * H / 14, "Instrument Mount Matrix Table", 29);
		skp->Text(1 * W / 16, 12 * H / 14, "Checkout Monitor", 16);
	}
	else if (screen == 16)
	{
		// Supersighter
		if (subscreen == 0)
		{
			skp->SetFont(font2);

			skp->SetTextAlign(oapi::Sketchpad::CENTER);
			skp->Text(W / 2, 2 * H / 36, "Supersighter Inputs", 19);
			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			x = 1;  y = 3; dx = 9;
			xmax = 32;
			ymax = 28;

			Text(skp, x, xmax, marker + y, ymax, "*");
			x++;
			Text(skp, x, xmax, y, ymax, "Mode:");
			switch (G->SSInputs.Mode)
			{
			case 1: sprintf_s(Buffer, "1: Moveable line-of-sight"); break;
			case 2: sprintf_s(Buffer, "2: Fixed attitude/fixed line-of-sight"); break;
			case 3: sprintf_s(Buffer, "3: Fixed line-of-sight rotation"); break;
			case 4: sprintf_s(Buffer, "4: Minimum maneuver"); break;
			case 5: sprintf_s(Buffer, "5: Fixed line-of-sight/MGA"); break;
			case 6: sprintf_s(Buffer, "6: Dual line-of-sight"); break;
			case 7: sprintf_s(Buffer, "7: Fixed line-of-sight/omicron"); break;
			}
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Source matrix:");
			Text(skp, x + dx, xmax, y, ymax, G->SSInputs.INMAT);
			y++;
			Text(skp, x, xmax, y, ymax, "Desired matrix:");
			Text(skp, x + dx, xmax, y, ymax, G->SSInputs.OUTMAT);
			y++;
			Text(skp, x, xmax, y, ymax, "Ephemeris ID:");
			if (G->shuttle)
			{
				sprintf_s(Buffer, G->shuttle->GetName());
			}
			else
			{
				sprintf_s(Buffer, "No Shuttle!");
			}
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Elevation angle:");
			sprintf_s(Buffer, "%+.2lf", G->SSInputs.ELV);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			if (G->SSInputs.Mode == 1 || G->SSInputs.Mode >= 4)
			{
				Text(skp, x, xmax, y, ymax, "Target 1 ID:");
				Text(skp, x + dx, xmax, y, ymax, G->SSInputs.TGT1);
				y++;
				Text(skp, x, xmax, y, ymax, "Target 2 ID:");
				Text(skp, x + dx, xmax, y, ymax, G->SSInputs.TGT2);
				y++;
			}
			else y += 2;
			if (G->SSInputs.Mode != 3)
			{
				Text(skp, x, xmax, y, ymax, "Instrument IA1:");
				Text(skp, x + dx, xmax, y, ymax, G->SSInputs.IA1);
				y++;
				Text(skp, x, xmax, y, ymax, "Instrument IA2:");
				Text(skp, x + dx, xmax, y, ymax, G->SSInputs.IA2);
				y++;
				if (G->SSInputs.Mode == 5)
				{
					Text(skp, x, xmax, y, ymax, "Instrument IB1:");
					Text(skp, x + dx, xmax, y, ymax, G->SSInputs.IB1);
				}
				y++;
				if (G->SSInputs.Mode == 5)
				{
					Text(skp, x, xmax, y, ymax, "Instrument IB2:");
					Text(skp, x + dx, xmax, y, ymax, G->SSInputs.IB2);
				}
				y++;
			}
			else y += 4;
			if (G->SSInputs.Mode < 6)
			{
				Text(skp, x, xmax, y, ymax, "Att Sense:");
				if (G->SSInputs.ATTSense == 0) Text(skp, x + dx, xmax, y, ymax, "+X");
				else if (G->SSInputs.ATTSense == 1) Text(skp, x + dx, xmax, y, ymax, "-X");
				else Text(skp, x + dx, xmax, y, ymax, "-Z");
			}
			y++;
			if (G->SSInputs.Mode <= 4)
			{
				Text(skp, x, xmax, y, ymax, "Attitude:");
				sprintf_s(Buffer, "%06.2lf %06.2lf %06.2lf", G->SSInputs.ATT.x, G->SSInputs.ATT.y, G->SSInputs.ATT.z);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			if (G->SSInputs.Mode == 2 || G->SSInputs.Mode >= 4)
			{
				Text(skp, x, xmax, y, ymax, "IA1 Angles:");
				sprintf_s(Buffer, "%06.2lf %06.2lf", G->SSInputs.IA1_A1, G->SSInputs.IA1_A2);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			if (G->SSInputs.Mode == 6 || (G->SSInputs.IA2 != ""))
			{
				Text(skp, x, xmax, y, ymax, "IA2 Angles:");
				sprintf_s(Buffer, "%06.2lf %06.2lf", G->SSInputs.IA2_A1, G->SSInputs.IA2_A2);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			if (G->SSInputs.Mode == 5 && G->SSInputs.IB1 != "")
			{
				Text(skp, x, xmax, y, ymax, "IB1 Angles:");
				sprintf_s(Buffer, "%06.2lf %06.2lf", G->SSInputs.IB1_A1, G->SSInputs.IB1_A2);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			if (G->SSInputs.Mode == 5 && G->SSInputs.IB2 != "")
			{
				Text(skp, x, xmax, y, ymax, "IB2 Angles:");
				sprintf_s(Buffer, "%06.2lf %06.2lf", G->SSInputs.IB2_A1, G->SSInputs.IB2_A2);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			if (G->SSInputs.Mode == 5)
			{
				Text(skp, x, xmax, y, ymax, "MGA:");
				sprintf_s(Buffer, "%06.2lf", G->SSInputs.MGA);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			if (G->SSInputs.Mode == 7)
			{
				Text(skp, x, xmax, y, ymax, "OMI:");
				sprintf_s(Buffer, "%06.2lf", G->SSInputs.OMI);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			if (G->SSInputs.Mode == 3)
			{
				Text(skp, x, xmax, y, ymax, "Eigen Vector:");
				sprintf_s(Buffer, "%06.2lf %06.2lf %06.2lf", G->SSInputs.EIG.x, G->SSInputs.EIG.y, G->SSInputs.EIG.z);
				Text(skp, x + dx, xmax, y, ymax, Buffer);
			}
			y++;
			Text(skp, x, xmax, y, ymax, "Start time:");
			MET2String(Buffer, G->SSInputs.StartTime);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Source bias matrix:");
			Text(skp, x + dx, xmax, y, ymax, "TBD");
			y++;
			Text(skp, x, xmax, y, ymax, "Desired bias matrix:");
			Text(skp, x + dx, xmax, y, ymax, "TBD");
			y++;
		}
		else
		{
			skp->SetFont(font3);
			GetCharSize(skp, CW, CH);
			skp->SetPen(pen1);

			Line2(skp, 0, 7, 74, 7);
			Line2(skp, 15, 16, 56, 16);
			Line2(skp, 15, 16, 15, 36);
			Line2(skp, 56, 16, 56, 36);
			Line2(skp, 0, 36, 74, 36);

			Text2(skp, 26, 3, "SUPERSIGHTER");
			Text2(skp, 0, 5, "INPUT MATRIX");
			Text2(skp, 26, 5, "ATT SOURCE");
			Text2(skp, 47, 5, "I/P LVLH BIAS");
			Text2(skp, 0, 6, "OUTPUT MATRIX");
			Text2(skp, 26, 6, "EPH");
			Text2(skp, 35, 6, "VID");
			Text2(skp, 47, 6, "O/P LVLH BIAS");
			Text2(skp, 33, 8, "MODE");
			Text2(skp, 56, 9, "EIGEN VECTOR");
			Text2(skp, 57, 10, "P");
			Text2(skp, 1, 11, "ELV");
			Text2(skp, 27, 11, "INPUT ATT SENSE");
			Text2(skp, 57, 11, "Y");
			Text2(skp, 1, 12, "VEH");
			Text2(skp, 56, 12, "EIGEN ANG");
			Text2(skp, 1, 13, "RANGE");
			Text2(skp, 32, 13, "R");
			Text2(skp, 42, 13, "MGA");
			Text2(skp, 1, 14, "MODE 2");
			Text2(skp, 32, 14, "P");
			Text2(skp, 2, 15, "LAT");
			Text2(skp, 32, 15, "Y");
			Text2(skp, 2, 16, "LON");
			Text2(skp, 57, 16, "IA1");
			Text2(skp, 30, 17, "OUTPUT A");
			Text2(skp, 58, 17, "A1");
			Text2(skp, 0, 18, "TARGET 1");
			Text2(skp, 20, 18, "+X");
			Text2(skp, 32, 18, "-X");
			Text2(skp, 44, 18, "-Z");
			Text2(skp, 58, 18, "A2");
			Text2(skp, 1, 19, "RA");
			Text2(skp, 17, 19, "R");
			Text2(skp, 29, 19, "R");
			Text2(skp, 41, 19, "R");
			Text2(skp, 58, 19, "A3");
			Text2(skp, 1, 20, "DEC");
			Text2(skp, 17, 20, "P");
			Text2(skp, 29, 20, "P");
			Text2(skp, 41, 20, "P");
			Text2(skp, 1, 21, "LAT");
			Text2(skp, 17, 21, "Y");
			Text2(skp, 29, 21, "Y");
			Text2(skp, 41, 21, "Y");
			Text2(skp, 57, 21, "IA2");
			Text2(skp, 1, 22, "LON");
			Text2(skp, 58, 22, "A1");
			Text2(skp, 1, 23, "ALT");
			Text2(skp, 17, 23, "R");
			Text2(skp, 29, 23, "R");
			Text2(skp, 41, 23, "R");
			Text2(skp, 58, 23, "A2");
			Text2(skp, 1, 24, "RNG");
			Text2(skp, 17, 24, "P");
			Text2(skp, 29, 24, "P");
			Text2(skp, 41, 24, "P");
			Text2(skp, 52, 24, "LVLH");
			Text2(skp, 17, 25, "Y");
			Text2(skp, 29, 25, "Y");
			Text2(skp, 41, 25, "Y");
			Text2(skp, 57, 26, "IB1");
			Text2(skp, 30, 27, "OUTPUT B");
			Text2(skp, 58, 27, "A1");
			Text2(skp, 20, 28, "+X");
			Text2(skp, 32, 28, "-X");
			Text2(skp, 44, 28, "-Z");
			Text2(skp, 58, 28, "A2");
			Text2(skp, 0, 29, "TARGET 2");
			Text2(skp, 17, 29, "R");
			Text2(skp, 29, 29, "R");
			Text2(skp, 41, 29, "R");
			Text2(skp, 1, 30, "RA");
			Text2(skp, 17, 30, "P");
			Text2(skp, 29, 30, "P");
			Text2(skp, 41, 30, "P");
			Text2(skp, 1, 31, "DEC");
			Text2(skp, 17, 31, "Y");
			Text2(skp, 29, 31, "Y");
			Text2(skp, 41, 31, "Y");
			Text2(skp, 57, 31, "IB2");
			Text2(skp, 58, 32, "A1");
			Text2(skp, 17, 33, "R");
			Text2(skp, 29, 33, "R");
			Text2(skp, 41, 33, "R");
			Text2(skp, 58, 33, "A2");
			Text2(skp, 17, 34, "P");
			Text2(skp, 29, 34, "P");
			Text2(skp, 41, 34, "P");
			Text2(skp, 52, 34, "LVLH");
			Text2(skp, 17, 35, "Y");
			Text2(skp, 29, 35, "Y");
			Text2(skp, 41, 35, "Y");
			Text2(skp, 7, 37, "MET");
			Text2(skp, 23, 37, "GMT");
			Text2(skp, 1, 38, "ST");
			Text2(skp, 36, 38, "+X RA");
			Text2(skp, 56, 38, "E    M    S");
			Text2(skp, 0, 39, "AOS");
			Text2(skp, 39, 39, "DEC");
			Text2(skp, 52, 39, "P");
			Text2(skp, 0, 40, "TCA");
			Text2(skp, 36, 40, "-Z RA");
			Text2(skp, 52, 40, "Y");
			Text2(skp, 0, 41, "LOS");
			Text2(skp, 39, 41, "DEC");
			Text2(skp, 52, 41, "TH");
			Text2(skp, 52, 42, "PH");

			Text2(skp, 1, 42, G->SSOutputs.ErrorMessage);

			skp->SetTextAlign(oapi::Sketchpad::TAlign_horizontal::RIGHT);
			Text2(skp, 21, 5, G->SSOutputs.INMAT);
			Text2(skp, 42, 5, G->SSOutputs.ATT_SOURCE);
			Text2(skp, 68, 5, G->SSOutputs.IN_LVLH_BIAS);
			Text2(skp, 21, 6, G->SSOutputs.OUTMAT);
			Text2(skp, 68, 6, G->SSOutputs.OUT_LVLH_BIAS);
			Text2(skp, 43, 8, G->SSOutputs.MODE);
			Text2(skp, 65, 10, G->SSOutputs.EIGEN_VECTOR_P);
			Text2(skp, 12, 11, G->SSOutputs.ELV);
			Text2(skp, 47, 11, G->SSOutputs.ATT_SENSE);
			Text2(skp, 65, 11, G->SSOutputs.EIGEN_VECTOR_Y);
			Text2(skp, 72, 12, G->SSOutputs.EIGEN_ANG);
			Text2(skp, 14, 13, G->SSOutputs.VEH_RANGE);
			Text2(skp, 40, 13, G->SSOutputs.INPUT_ATT[0]);
			Text2(skp, 52, 13, G->SSOutputs.MGA);
			Text2(skp, 40, 14, G->SSOutputs.INPUT_ATT[1]);
			Text2(skp, 12, 15, G->SSOutputs.MODE2_LAT);
			Text2(skp, 40, 15, G->SSOutputs.INPUT_ATT[2]);
			Text2(skp, 13, 16, G->SSOutputs.MODE2_LON);
			Text2(skp, 64, 16, G->SSOutputs.IA1);
			Text2(skp, 70, 16, G->SSOutputs.IA1_OCC);
			Text2(skp, 68, 17, G->SSOutputs.IA1_A1);
			Text2(skp, 70, 17, G->SSOutputs.IA1_A1_LIM);
			Text2(skp, 68, 18, G->SSOutputs.IA1_A2);
			Text2(skp, 70, 18, G->SSOutputs.IA1_A2_LIM);
			Text2(skp, 68, 19, G->SSOutputs.IA1_A3);
			Text2(skp, 64, 21, G->SSOutputs.IA2);
			Text2(skp, 70, 21, G->SSOutputs.IA2_OCC);
			Text2(skp, 68, 22, G->SSOutputs.IA2_A1);
			Text2(skp, 70, 22, G->SSOutputs.IA2_A1_LIM);
			Text2(skp, 68, 23, G->SSOutputs.IA2_A2);
			Text2(skp, 70, 23, G->SSOutputs.IA2_A2_LIM);
			Text2(skp, 64, 26, G->SSOutputs.IB1);
			Text2(skp, 70, 26, G->SSOutputs.IB1_OCC);
			Text2(skp, 68, 27, G->SSOutputs.IB1_A1);
			Text2(skp, 70, 27, G->SSOutputs.IB1_A1_LIM);
			Text2(skp, 68, 28, G->SSOutputs.IB1_A2);
			Text2(skp, 70, 28, G->SSOutputs.IB1_A2_LIM);
			Text2(skp, 64, 31, G->SSOutputs.IB2);
			Text2(skp, 70, 31, G->SSOutputs.IB2_OCC);
			Text2(skp, 68, 32, G->SSOutputs.IB2_A1);
			Text2(skp, 70, 32, G->SSOutputs.IB2_A1_LIM);
			Text2(skp, 68, 33, G->SSOutputs.IB2_A2);
			Text2(skp, 70, 33, G->SSOutputs.IB2_A2_LIM);

			Text2(skp, 25, 19, G->SSOutputs.OUTPUT_A_ATT[0][0]);
			Text2(skp, 37, 19, G->SSOutputs.OUTPUT_A_ATT[1][0]);
			Text2(skp, 49, 19, G->SSOutputs.OUTPUT_A_ATT[2][0]);
			Text2(skp, 25, 20, G->SSOutputs.OUTPUT_A_ATT[0][1]);
			Text2(skp, 37, 20, G->SSOutputs.OUTPUT_A_ATT[1][1]);
			Text2(skp, 49, 20, G->SSOutputs.OUTPUT_A_ATT[2][1]);
			Text2(skp, 25, 21, G->SSOutputs.OUTPUT_A_ATT[0][2]);
			Text2(skp, 37, 21, G->SSOutputs.OUTPUT_A_ATT[1][2]);
			Text2(skp, 49, 21, G->SSOutputs.OUTPUT_A_ATT[2][2]);

			Text2(skp, 55, 20, G->SSOutputs.OUTPUT_A_ATT_REF);

			Text2(skp, 25, 23, G->SSOutputs.OUTPUT_A_ATT[3][0]);
			Text2(skp, 37, 23, G->SSOutputs.OUTPUT_A_ATT[4][0]);
			Text2(skp, 49, 23, G->SSOutputs.OUTPUT_A_ATT[5][0]);
			Text2(skp, 25, 24, G->SSOutputs.OUTPUT_A_ATT[3][1]);
			Text2(skp, 37, 24, G->SSOutputs.OUTPUT_A_ATT[4][1]);
			Text2(skp, 49, 24, G->SSOutputs.OUTPUT_A_ATT[5][1]);
			Text2(skp, 25, 25, G->SSOutputs.OUTPUT_A_ATT[3][2]);
			Text2(skp, 37, 25, G->SSOutputs.OUTPUT_A_ATT[4][2]);
			Text2(skp, 49, 25, G->SSOutputs.OUTPUT_A_ATT[5][2]);

			Text2(skp, 25, 29, G->SSOutputs.OUTPUT_B_ATT[0][0]);
			Text2(skp, 37, 29, G->SSOutputs.OUTPUT_B_ATT[1][0]);
			Text2(skp, 49, 29, G->SSOutputs.OUTPUT_B_ATT[2][0]);
			Text2(skp, 25, 30, G->SSOutputs.OUTPUT_B_ATT[0][1]);
			Text2(skp, 37, 30, G->SSOutputs.OUTPUT_B_ATT[1][1]);
			Text2(skp, 49, 30, G->SSOutputs.OUTPUT_B_ATT[2][1]);
			Text2(skp, 25, 31, G->SSOutputs.OUTPUT_B_ATT[0][2]);
			Text2(skp, 37, 31, G->SSOutputs.OUTPUT_B_ATT[1][2]);
			Text2(skp, 49, 31, G->SSOutputs.OUTPUT_B_ATT[2][2]);

			Text2(skp, 55, 30, G->SSOutputs.OUTPUT_B_ATT_REF);

			Text2(skp, 25, 33, G->SSOutputs.OUTPUT_B_ATT[3][0]);
			Text2(skp, 37, 33, G->SSOutputs.OUTPUT_B_ATT[4][0]);
			Text2(skp, 49, 33, G->SSOutputs.OUTPUT_B_ATT[5][0]);
			Text2(skp, 25, 34, G->SSOutputs.OUTPUT_B_ATT[3][1]);
			Text2(skp, 37, 34, G->SSOutputs.OUTPUT_B_ATT[4][1]);
			Text2(skp, 49, 34, G->SSOutputs.OUTPUT_B_ATT[5][1]);
			Text2(skp, 25, 35, G->SSOutputs.OUTPUT_B_ATT[3][2]);
			Text2(skp, 37, 35, G->SSOutputs.OUTPUT_B_ATT[4][2]);
			Text2(skp, 49, 35, G->SSOutputs.OUTPUT_B_ATT[5][2]);

			Text2(skp, 13, 18, G->SSOutputs.TGT1);
			Text2(skp, 11, 19, G->SSOutputs.TGT1_RA);
			Text2(skp, 11, 20, G->SSOutputs.TGT1_DEC);
			Text2(skp, 11, 21, G->SSOutputs.TGT1_LAT);
			Text2(skp, 12, 22, G->SSOutputs.TGT1_LON);
			Text2(skp, 11, 23, G->SSOutputs.TGT1_ALT);
			Text2(skp, 12, 24, G->SSOutputs.TGT1_RNG);
			Text2(skp, 13, 29, G->SSOutputs.TGT2);
			Text2(skp, 11, 30, G->SSOutputs.TGT2_RA);
			Text2(skp, 11, 31, G->SSOutputs.TGT2_DEC);

			Text2(skp, 16, 38, G->SSOutputs.ST_MET);
			Text2(skp, 16, 39, G->SSOutputs.AOS_MET);
			Text2(skp, 16, 40, G->SSOutputs.TCA_MET);
			Text2(skp, 16, 41, G->SSOutputs.LOS_MET);

			Text2(skp, 32, 38, G->SSOutputs.ST_GMT);
			Text2(skp, 32, 39, G->SSOutputs.AOS_GMT);
			Text2(skp, 32, 40, G->SSOutputs.TCA_GMT);
			Text2(skp, 32, 41, G->SSOutputs.LOS_GMT);

			Text2(skp, 49, 38, G->SSOutputs.RA_PX);
			Text2(skp, 49, 39, G->SSOutputs.DEC_PX);
			Text2(skp, 49, 40, G->SSOutputs.RA_MZ);
			Text2(skp, 49, 41, G->SSOutputs.DEC_MZ);

			Text2(skp, 58, 39, G->SSOutputs.Pitch_E);
			Text2(skp, 63, 39, G->SSOutputs.Pitch_M);
			Text2(skp, 68, 39, G->SSOutputs.Pitch_S);

			Text2(skp, 58, 40, G->SSOutputs.Yaw_E);
			Text2(skp, 63, 40, G->SSOutputs.Yaw_M);
			Text2(skp, 68, 40, G->SSOutputs.Yaw_S);

			Text2(skp, 58, 41, G->SSOutputs.Theta_E);
			Text2(skp, 63, 41, G->SSOutputs.Theta_M);
			Text2(skp, 68, 41, G->SSOutputs.Theta_S);

			Text2(skp, 58, 42, G->SSOutputs.Phi_E);
			Text2(skp, 63, 42, G->SSOutputs.Phi_M);
			Text2(skp, 68, 42, G->SSOutputs.Phi_S);
		}
	}
	else if (screen == 17)
	{
		if (subscreen == 0)
		{
			skp->SetFont(font3);
			GetCharSize(skp, CW, CH);
			skp->SetPen(pen1);

			skp->SetTextAlign(oapi::Sketchpad::CENTER);
			skp->Text(1 * W / 2, 2 * H / 36, "INSTRUMENT DEFINITION TABLE", 27);
			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			Line2(skp, 5, 5, 5, 30);
			Line2(skp, 22, 5, 22, 30);
			Line2(skp, 33, 5, 33, 30);
			Line2(skp, 54, 5, 54, 30);

			Text2(skp, 2, 6, "ID       COMMENT     TYPE/MT/RP   P      T      P     MIN1   MAX1   MIN2   MAX2");
			skp->SetTextAlign(oapi::Sketchpad::TAlign_horizontal::RIGHT);
			int j = 0;
			InstrumentDefinitionTableEntry inp;

			for (int i = 0; i < 25; i++)
			{
				if (G->IDT[i].IsInitialized() == false) continue;
				inp = G->IDT[i].GetInputs();
				// Print, using j
				sprintf_s(Buffer, "S%02d", i + 1);
				Text2(skp, 5, 7 + j, Buffer);
				sprintf_s(Buffer, "%-16s", inp.Comment);
				Text2(skp, 22, 7 + j, Buffer);
				sprintf_s(Buffer, "%03d  %02d %02d", inp.FormatInstrumentType(), inp.Mount, inp.RET_ID);
				Text2(skp, 33, 7 + j, Buffer);
				sprintf_s(Buffer, "%06.2lf %06.2lf %06.2lf", inp.Phi1, inp.Theta, inp.Phi2);
				Text2(skp, 54, 7 + j, Buffer);
				sprintf_s(Buffer, "%06.2lf %06.2lf %06.2lf %06.2lf", inp.A1_MIN, inp.A1_MAX, inp.A2_MIN, inp.A2_MAX);
				Text2(skp, 82, 7 + j, Buffer);
				j++;
			}
		}
		else
		{
			skp->SetFont(font2);

			skp->SetTextAlign(oapi::Sketchpad::CENTER);
			skp->Text(W / 2, 2 * H / 36, "IDT Inputs", 19);
			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			x = 1;  y = 3; dx = 9;
			xmax = 32;
			ymax = 28;

			Text(skp, x, xmax, marker + y, ymax, "*");
			x++;

			Text(skp, x, xmax, y, ymax, "Number:");
			sprintf_s(Buffer, "%d", G->IDT_Input_Num);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Name:");
			Text(skp, x + dx, xmax, y, ymax, G->IDT_Input_Comment);
			y++;
			Text(skp, x, xmax, y, ymax, "Type:");
			sprintf_s(Buffer, "%03d", G->IDT_Input_Type);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Phi 1:");
			sprintf_s(Buffer, "%06.2lf", G->IDT_Input.Phi1);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Theta:");
			sprintf_s(Buffer, "%06.2lf", G->IDT_Input.Theta);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Phi 2:");
			sprintf_s(Buffer, "%06.2lf", G->IDT_Input.Phi2);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "A1 Min:");
			sprintf_s(Buffer, "%06.2lf", G->IDT_Input.A1_MIN);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "A1 Max:");
			sprintf_s(Buffer, "%06.2lf", G->IDT_Input.A1_MAX);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "A2 Min:");
			sprintf_s(Buffer, "%06.2lf", G->IDT_Input.A2_MIN);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "A2 Max:");
			sprintf_s(Buffer, "%06.2lf", G->IDT_Input.A2_MAX);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "Reticle:");
			if (G->IDT_Input.RET_ID)
			{
				sprintf_s(Buffer, "Yes (Horizontal, Vertical)");
			}
			else
			{
				sprintf_s(Buffer, "No Reticle");
			}
			Text(skp, x + dx, xmax, y, ymax, Buffer);
		}
	}
	else if (screen == 18)
	{
		skp->SetFont(font4);
		GetCharSize(skp, CW, CH);
		//skp->SetTextColor(COLOR_CYAN);

		Text2(skp, 20, 1, "CHECKOUT MONITOR");
		Text2(skp, 1, 4, "GMT");
		Text2(skp, 1, 5, "MET");
		Text2(skp, 1, 9, "M50 State Vector");
		Text2(skp, 3, 11, "POSITION (FT)");
		Text2(skp, 2, 12, "X");
		Text2(skp, 2, 13, "Y");
		Text2(skp, 2, 14, "Z");
		Text2(skp, 3, 16, "VELOCITY (FPS)");
		Text2(skp, 1, 17, "VX");
		Text2(skp, 1, 18, "VY");
		Text2(skp, 1, 19, "VZ");
		Text2(skp, 3, 22, "POSITION (M)");
		Text2(skp, 2, 23, "X");
		Text2(skp, 2, 24, "Y");
		Text2(skp, 2, 25, "Z");
		Text2(skp, 3, 27, "VELOCITY (MPS)");
		Text2(skp, 1, 28, "VX");
		Text2(skp, 1, 29, "VY");
		Text2(skp, 1, 30, "VZ");

		Text2(skp, 20, 4, "HA");
		Text2(skp, 20, 5, "METHA");
		Text2(skp, 20, 6, "HP");
		Text2(skp, 20, 7, "METHP");
		Text2(skp, 20, 9, "Vi");
		Text2(skp, 20, 10, "Vrel");
		Text2(skp, 20, 11, "FPAi");
		Text2(skp, 20, 12, "AZr");
		Text2(skp, 20, 13, "AZi");
		Text2(skp, 20, 14, "LATc");
		Text2(skp, 20, 15, "LATc");
		Text2(skp, 20, 16, "LATd");
		Text2(skp, 20, 17, "LATd");
		Text2(skp, 20, 18, "LONG");
		Text2(skp, 20, 19, "LONG");
		Text2(skp, 20, 20, "Hs");
		Text2(skp, 20, 21, "Ho");
		Text2(skp, 20, 22, "Ho");
		Text2(skp, 20, 23, "R");
		Text2(skp, 20, 25, "Tan");
		Text2(skp, 20, 26, "Lam");
		Text2(skp, 20, 27, "BETA ANG");
		Text2(skp, 20, 28, "PERIOD");
		Text2(skp, 20, 29, "RAm50");
		Text2(skp, 20, 30, "DECm50");

		Text2(skp, 40, 8, "STOP OPTION");
		Text2(skp, 40, 9, "GMTTH");
		Text2(skp, 40, 10, "METTH");

		Text2(skp, 40, 12, "REFDAY");
		Text2(skp, 52, 12, "/");
		Text2(skp, 55, 12, "/");
		Text2(skp, 40, 13, "LO");

		Text2(skp, 40, 20, "KEPLERIAN ELEMENTS");
		Text2(skp, 40, 22, "A");
		Text2(skp, 40, 23, "E");
		Text2(skp, 40, 24, "Im50");
		Text2(skp, 40, 25, "Iteg");
		Text2(skp, 40, 26, "WPm50");
		Text2(skp, 40, 27, "WPteg");
		Text2(skp, 40, 28, "RAANm50");
		Text2(skp, 40, 29, "N");
		Text2(skp, 40, 30, "M");

		skp->SetTextAlign(oapi::Sketchpad::RIGHT);
		skp->SetTextColor(GetDefaultColour(2));

		MET2String3(Buffer, G->CO_MON_Time);
		Text2(skp, 60, 10, Buffer);

		Text2(skp, 19, 4, G->CO_DISP.GMT);
		Text2(skp, 19, 5, G->CO_DISP.MET);
		Text2(skp, 17, 12, G->CO_DISP.M50_POS_FT[0]);
		Text2(skp, 17, 13, G->CO_DISP.M50_POS_FT[1]);
		Text2(skp, 17, 14, G->CO_DISP.M50_POS_FT[2]);
		Text2(skp, 17, 17, G->CO_DISP.M50_VEL_FPS[0]);
		Text2(skp, 17, 18, G->CO_DISP.M50_VEL_FPS[1]);
		Text2(skp, 17, 19, G->CO_DISP.M50_VEL_FPS[2]);

		Text2(skp, 17, 23, G->CO_DISP.M50_POS_M[0]);
		Text2(skp, 17, 24, G->CO_DISP.M50_POS_M[1]);
		Text2(skp, 17, 25, G->CO_DISP.M50_POS_M[2]);
		Text2(skp, 17, 28, G->CO_DISP.M50_VEL_MPS[0]);
		Text2(skp, 17, 29, G->CO_DISP.M50_VEL_MPS[1]);
		Text2(skp, 17, 30, G->CO_DISP.M50_VEL_MPS[2]);

		Text2(skp, 37, 4, G->CO_DISP.HA);
		Text2(skp, 37, 6, G->CO_DISP.HP);
		Text2(skp, 37, 9, G->CO_DISP.V_I);
		Text2(skp, 37, 10, G->CO_DISP.V_REL);
		Text2(skp, 37, 11, G->CO_DISP.GAMMA);
		Text2(skp, 37, 12, G->CO_DISP.PSI_REL);
		Text2(skp, 37, 13, G->CO_DISP.PSI_TEI);
		Text2(skp, 37, 14, G->CO_DISP.PHI_C[0]);
		Text2(skp, 37, 15, G->CO_DISP.PHI_C[1]);
		Text2(skp, 37, 16, G->CO_DISP.PHI_D[0]);
		Text2(skp, 37, 17, G->CO_DISP.PHI_D[1]);
		Text2(skp, 37, 18, G->CO_DISP.LAMBDA[0]);
		Text2(skp, 37, 19, G->CO_DISP.LAMBDA[1]);
		Text2(skp, 37, 20, G->CO_DISP.h_s);
		Text2(skp, 37, 21, G->CO_DISP.h_o[0]);
		Text2(skp, 37, 22, G->CO_DISP.h_o[1]);
		Text2(skp, 37, 23, G->CO_DISP.R);

		Text2(skp, 37, 27, G->CO_DISP.BETA_ANG);
		Text2(skp, 37, 28, G->CO_DISP.PERIOD);
		Text2(skp, 37, 29, G->CO_DISP.RA_M50);
		Text2(skp, 37, 30, G->CO_DISP.DEC_M50);

		Text2(skp, 52, 12, G->CO_DISP.REF_DAY_D);
		Text2(skp, 55, 12, G->CO_DISP.REF_DAY_M);
		Text2(skp, 60, 12, G->CO_DISP.REF_DAY_Y);
		Text2(skp, 60, 13, G->CO_DISP.LO);

		Text2(skp, 58, 22, G->CO_DISP.A);
		Text2(skp, 58, 23, G->CO_DISP.E);
		Text2(skp, 58, 24, G->CO_DISP.I_M50);
		Text2(skp, 58, 25, G->CO_DISP.I_TEI);
		Text2(skp, 58, 26, G->CO_DISP.WP_M50);
		Text2(skp, 58, 27, G->CO_DISP.WP_TEI);
		Text2(skp, 58, 28, G->CO_DISP.RAAN_M50);
		Text2(skp, 58, 29, G->CO_DISP.N);
		Text2(skp, 58, 30, G->CO_DISP.M);
	}
	else if (screen == 19)
	{
		if (subscreen == 0)
		{
			skp->SetFont(font4);
			GetCharSize(skp, CW, CH);

			skp->SetTextAlign(oapi::Sketchpad::CENTER);
			skp->Text(1 * W / 2, 2 * H / 36, "GROUND TARGET DISPLAY", 21);
			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			Text2(skp, 4, 5, "ID          NAME         LAT     LONG    ALT");

			GroundTargetFileEntry* inp;
			int j = 0;
			skp->SetTextAlign(oapi::Sketchpad::TAlign_horizontal::RIGHT);

			for (int i = 0; i < 100; i++)
			{
				if (G->GTF.targets[i].Name == "") continue;
				inp = &G->GTF.targets[i];
				// Print, using j
				sprintf_s(Buffer, "G%03d", i + 1);
				Text2(skp, 7, 7 + j, Buffer);
				sprintf_s(Buffer, "%-20s", inp->Name.c_str());
				Text2(skp, 28, 7 + j, Buffer);
				sprintf_s(Buffer, "%.3lf", inp->Lat);
				Text2(skp, 34, 7 + j, Buffer);
				sprintf_s(Buffer, "%.3lf", inp->Lng);
				Text2(skp, 43, 7 + j, Buffer);
				sprintf_s(Buffer, "%.0lf", inp->Alt);
				Text2(skp, 50, 7 + j, Buffer);
				j++;
			}
		}
		else
		{
			skp->SetFont(font2);

			skp->SetTextAlign(oapi::Sketchpad::CENTER);
			skp->Text(W / 2, 2 * H / 36, "Ground Target Inputs", 20);
			skp->SetTextAlign(oapi::Sketchpad::LEFT);

			x = 1;  y = 3; dx = 8;
			xmax = 32;
			ymax = 16;

			Text(skp, x, xmax, marker + y, ymax, "*");
			x++;
			Text(skp, x, xmax, y, ymax, "GRD TGT:");
			sprintf_s(Buffer, "%d", G->GTF_Input_Num);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "NAME:");
			sprintf_s(Buffer, "%s", G->GTF_Input.Name.c_str());
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "LAT:");
			sprintf_s(Buffer, "%.2lf", G->GTF_Input.Lat);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "LNG:");
			sprintf_s(Buffer, "%.2lf", G->GTF_Input.Lng);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
			Text(skp, x, xmax, y, ymax, "ALT:");
			sprintf_s(Buffer, "%.0lf", G->GTF_Input.Alt);
			Text(skp, x + dx, xmax, y, ymax, Buffer);
			y++;
		}
	}
	else if (screen == 20)
	{
		skp->SetFont(font3);
		GetCharSize(skp, CW, CH);

		Text2(skp, 15, 1, "INSTRUMENT MOUNT MATRIX TABLE");

		Text2(skp, 1, 3, "ID COMMENT       MATRIX");

		int j = 0;
		for (int i = 0; i < 16; i++)
		{
			if (G->InstMountMat[i].Comment == "") continue;

			sprintf(Buffer, "%02d %-8s %+.7lf %+.7lf %+.7lf %+.7lf", i + 1, G->InstMountMat[i].Comment.c_str(),
				G->InstMountMat[i].MAT.m11, G->InstMountMat[i].MAT.m12, G->InstMountMat[i].MAT.m13, G->InstMountMat[i].MAT.m21);

			Text2(skp, 1, 5 + j, Buffer);
			sprintf(Buffer, "%+.7lf %+.7lf %+.7lf %+.7lf %+.7lf", G->InstMountMat[i].MAT.m22, G->InstMountMat[i].MAT.m23,
				G->InstMountMat[i].MAT.m31, G->InstMountMat[i].MAT.m32, G->InstMountMat[i].MAT.m33);
			Text2(skp, 1, 6 + j, Buffer);

			j += 3;
		}
	}
	return true;
}

void ShuttleFDOMFD::menuSetMainMenu()
{
	SetScreen(0);
}

void ShuttleFDOMFD::menuSetMCTPage()
{
	SetScreen(1);
}

void ShuttleFDOMFD::menuSetMETPage()
{
	METScroll = 0;
	SetScreen(2);
}

void ShuttleFDOMFD::menuSetLWPPage()
{
	SetScreen(3);
}

void ShuttleFDOMFD::menuSetMTTPage()
{
	SetScreen(4);

	MTTFlag = false;
}

void ShuttleFDOMFD::menuSetDMTPage()
{
	SetScreen(5);
}

void ShuttleFDOMFD::menuSetConfigurationMenu()
{
	SetScreen(6);
}

void ShuttleFDOMFD::menuSetLWPPage2()
{
	SetScreen(7);
}

void ShuttleFDOMFD::menuSetLWPPage3()
{
	SetScreen(8);
}

void ShuttleFDOMFD::menuSetDOPSPage()
{
	SetScreen(9);
}

void ShuttleFDOMFD::menuSetDMPPage()
{
	SetScreen(10);
}

void ShuttleFDOMFD::menuSetLTPPage()
{
	SetScreen(11);
}

void ShuttleFDOMFD::menuLWPOMSTargetSetsPage()
{
	SetScreen(12);
}

void ShuttleFDOMFD::menuSetDMPSolutionPage()
{
	SetScreen(13);
}

void ShuttleFDOMFD::menuSetOMPMenuPage()
{
	SetScreen(14);
}

void ShuttleFDOMFD::menuSetAttitudeAndPointingPage()
{
	SetScreen(15);
}

void ShuttleFDOMFD::menuSetSupersighterDisplayPage()
{
	SetScreen(16);
	subscreen = 0;
	subscreenmax = 1;
	marker = 0;
	markermax = 22;
}

void ShuttleFDOMFD::menuSetInstrumentDefinitionPage()
{
	SetScreen(17);
	subscreen = 0;
	subscreenmax = 1;
	marker = 0;
	markermax = 10;
}

void ShuttleFDOMFD::menuSetCheckoutMonitorPage()
{
	SetScreen(18);
}

void ShuttleFDOMFD::menuSetGroundTargetPage()
{
	SetScreen(19);
	subscreen = 0;
	subscreenmax = 1;
	marker = 0;
	markermax = 4;
}

void ShuttleFDOMFD::menuSetMountMatrixTablePage()
{
	SetScreen(20);
}

void ShuttleFDOMFD::SetScreen(int s)
{
	screen = s;
	coreButtons.SelectPage(this, screen);
}

void ShuttleFDOMFD::menuCycleSubscreen()
{
	if (subscreen < subscreenmax)
	{
		subscreen++;
	}
	else
	{
		subscreen = 0;
	}
}

void ShuttleFDOMFD::menuCycleMarkerUp()
{
	if (marker >= markermax)
	{
		marker = 0;
	}
	else
	{
		marker++;
	}
}

void ShuttleFDOMFD::menuCycleMarkerDown()
{
	if (marker <= 0)
	{
		marker = markermax;
	}
	else
	{
		marker--;
	}
}

void ShuttleFDOMFD::Text(oapi::Sketchpad* skp, int x, int y, std::string val)
{
	skp->Text(x, y, val.c_str(), val.size());
}

void ShuttleFDOMFD::Text(oapi::Sketchpad* skp, int x, int xmax, int y, int ymax, std::string val)
{
	Text(skp, x * W / xmax, y * H / ymax, val);
}

void ShuttleFDOMFD::Text2(oapi::Sketchpad* skp, int x, int y, std::string val)
{
	// Format in terms of character width/height
	Text(skp, CW * x, CH * y, val);
}

void ShuttleFDOMFD::Line2(oapi::Sketchpad* skp, int x0, int y0, int x1, int y1)
{
	skp->Line((CW * (2 * x0 + 1)) / 2 , (CH * (2 * y0 + 1)) / 2 , (CW * (2 * x1 + 1)) / 2 , (CH * (2 * y1 + 1)) / 2);
}

void ShuttleFDOMFD::MET2String(char *buf, double MET)
{
	// Format: DDD:HH:MM:SS.SSS
	MET = round(MET*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%06.3f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void ShuttleFDOMFD::MET2String2(char *buf, double MET)
{
	//Format: DD/HH:MM
	MET = round(MET*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%02.0f/%02.0f:%02.0f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0));
}

void ShuttleFDOMFD::MET2String3(char* buf, double MET)
{
	// Format: DDD:HH:MM:SS.SS
	MET = round(MET * 100.0) / 100.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%05.2f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void ShuttleFDOMFD::MET2String4(char* buf, double MET)
{
	// Format: DDD:HH:MM:SS
	MET = round(MET);
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%02.0f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void ShuttleFDOMFD::DMTMET2String(char *buf, double MET)
{
	MET = round(MET*10.0) / 10.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%04.1f", floor(MET / 86400.0), floor(fmod(MET, 86400.0) / 3600.0), floor(fmod(MET, 3600.0) / 60.0), fmod(MET, 60.0));
}

void ShuttleFDOMFD::GMT2String(char *buf, double GMT)
{
	// Format: DDD:HH:MM:SS.SSS
	GMT = round(GMT*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%06.3f", floor(GMT / 86400.0) + (double)G->sescnst.DayOfYear, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
}

void ShuttleFDOMFD::GMT2String2(char *buf, double GMT)
{
	//Format: DDD:HH:MM
	GMT = round(GMT*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f", floor(GMT / 86400.0) + (double)G->sescnst.DayOfYear, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0));
}

void ShuttleFDOMFD::LWPGMT2String(char *buf, double GMT)
{
	GMT = round(GMT*1000.0) / 1000.0;
	sprintf_s(buf, 100, "%03.0f:%02.0f:%02.0f:%04.1f", floor(GMT / 86400.0) + (double)G->sescnst.DayOfYear, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
}

void ShuttleFDOMFD::LTPGMT2String(char *buf, double GMT)
{
	sprintf_s(buf, 100, "%d:%03.0f:%02.0f:%02.0f:%06.3f", G->sescnst.Year, floor(GMT / 86400.0) + (double)G->sescnst.DayOfYear, floor(fmod(GMT, 86400.0) / 3600.0), floor(fmod(GMT, 3600.0) / 60.0), fmod(GMT, 60.0));
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
	char type[11], name[11];

	if (sscanf_s(str, "%s %s", type, 11, name, 11) == 2)
	{
		return ((ShuttleFDOMFD*)data)->add_OMPManeuver(type, name, 0);
	}
	return false;
}

bool ShuttleFDOMFD::add_OMPManeuver(char *type, char *name, unsigned ins)
{
	return G->AddManeuver(type, name, ins);
}

void ShuttleFDOMFD::menuModifySecondary()
{
	if (G->MCT.Table.size() == 0U) return;

	bool ModifyOMPSecondaryInput(void *id, char *str, void *data);
	oapiOpenInputBox("Modify Secondary (format: Sec Type Value)", ModifyOMPSecondaryInput, 0, 20, (void*)this);
}

bool ModifyOMPSecondaryInput(void *id, char *str, void *data)
{
	unsigned sec;
	char type[32];
	double val;

	if (sscanf_s(str, "%d %s %lf", &sec, type, 32, &val) == 3)
	{
		return ((ShuttleFDOMFD*)data)->modify_OMPManeuverSecondary(sec, type, val);
	}
	return false;
}

bool ShuttleFDOMFD::modify_OMPManeuverSecondary(unsigned sec, char * str, double val)
{
	unsigned man = MCTSelectedManeuver + 1;
	if (man <= G->MCT.Table.size() && man >= 1)
	{
		if (sec <= G->MCT.Table[man - 1].secondaries.size() && sec >= 1)
		{
			OMP::OMPDefs::SECONDARIES type = OMP::GetSecondaryType(str);
			if (type == OMP::OMPDefs::NOSEC) return false;

			G->MCT.Table[man - 1].secondaries[sec - 1].type = type;
			G->MCT.Table[man - 1].secondaries[sec - 1].value = val;
			return true;
		}
	}
	return false;
}


void ShuttleFDOMFD::menuModifyOMPManeuver()
{
	if (G->MCT.Table.size() == 0U) return;

	char Buff1[64], Buff2[64];

	sprintf_s(Buff1, 64, OMP::GetOPMManeuverType(G->MCT.Table[MCTSelectedManeuver].type).c_str());
	sprintf_s(Buff2, 64, G->MCT.Table[MCTSelectedManeuver].name.c_str());
	sprintf_s(Buffer, "%s %s", Buff1, Buff2);

	bool ModifyOMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Modify Maneuver (format: type name)", ModifyOMPManeuverInput, Buffer, 20, (void*)this);
}

bool ModifyOMPManeuverInput(void *id, char *str, void *data)
{
	char type[32], name[32];

	if (sscanf_s(str, "%s %s", type, 32, name, 32) == 2)
	{
		return ((ShuttleFDOMFD*)data)->modify_OMPManeuver(type, name);
	}
	return false;
}

bool ShuttleFDOMFD::modify_OMPManeuver(char *type, char *name)
{
	unsigned num = MCTSelectedManeuver + 1;
	OMP::OMPDefs::MANTYPE man = OMP::GetOPMManeuverType(type);
	if (man == OMP::OMPDefs::MANTYPE::NOMAN)
	{
		return false;
	}

	G->ModifyManeuver(num - 1, man, name);
	return true;
}

void ShuttleFDOMFD::menuAddOMPThreshold()
{
	if (G->MCT.Table.size() == 0U) return;

	if (G->MCT.Table[MCTSelectedManeuver].threshold == OMP::OMPDefs::THRESHOLD::NOTHR)
	{
		sprintf(Buffer, "");
	}
	else
	{
		char Buff1[64], Buff2[63];

		sprintf_s(Buff1, 64, OMP::GetOPMManeuverThreshold(G->MCT.Table[MCTSelectedManeuver].threshold).c_str());
		GetOPMManeuverThresholdTime(Buff2, G->MCT.Table[MCTSelectedManeuver].threshold, G->MCT.Table[MCTSelectedManeuver].thresh_num);
		sprintf(Buffer, "%s %s", Buff1, Buff2);
	}

	bool AddOMPThresholdInput(void *id, char *str, void *data);
	oapiOpenInputBox("Set Maneuver Threshold (format: Type Value)", AddOMPThresholdInput, Buffer, 25, (void*)this);
}

bool AddOMPThresholdInput(void *id, char *str, void *data)
{
	char type[32], time[32];

	if (sscanf_s(str, "%s %s", type, 32, time, 32) == 2)
	{
		return ((ShuttleFDOMFD*)data)->add_OMPManeuverThreshold(type, time);
	}
	return false;
}

bool ShuttleFDOMFD::add_OMPManeuverThreshold(char *type, char * str)
{
	unsigned num = MCTSelectedManeuver + 1;
	if (num <= G->MCT.Table.size() && num >= 1)
	{
		OMP::OMPDefs::THRESHOLD thres = OMP::GetOPMThresholdType(type);

		switch (thres)
		{
		case OMP::OMPDefs::THRESHOLD::THRES_APS:
		case OMP::OMPDefs::THRESHOLD::THRES_CAN:
		case OMP::OMPDefs::THRESHOLD::THRES_M:
		case OMP::OMPDefs::THRESHOLD::THRES_N:
		case OMP::OMPDefs::THRESHOLD::THRES_REV:
		case OMP::OMPDefs::THRESHOLD::THRES_WT:
		{
			double val;

			if (sscanf_s(str, "%lf", &val) == 1)
			{
				if (thres == OMP::OMPDefs::THRESHOLD::THRES_CAN || thres == OMP::OMPDefs::THRESHOLD::THRES_WT)
				{
					val *= RAD;
				}

				G->AddManeuverThreshold(num - 1, thres, val);
				return true;
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
			if (sscanf_s(str, "%d:%d:%d:%lf", &dd, &hh, &mm, &ss) == 4)
			{
				G->AddManeuverThreshold(num - 1, thres, DDDHHHMMSS2MET(dd, hh, mm, ss));
				return true;
			}
		}
		break;
		}

		return false;
	}

	return false;
}

void ShuttleFDOMFD::menuAddOMPSecondary()
{
	if (G->MCT.Table.size() == 0U) return;

	bool AddOMPSecondaryInput(void *id, char *str, void *data);
	oapiOpenInputBox("Add Secondary Constraint (format: Type Value)", AddOMPSecondaryInput, 0, 20, (void*)this);
}

bool AddOMPSecondaryInput(void *id, char *str, void *data)
{
	char type[32];
	double val;

	if (sscanf_s(str, "%s %lf", type, 32, &val) == 2)
	{
		return ((ShuttleFDOMFD*)data)->add_OMPManeuverSecondary(type, val);
	}
	return false;
}

bool ShuttleFDOMFD::add_OMPManeuverSecondary(char * str, double val)
{
	unsigned num = MCTSelectedManeuver + 1;
	if (num <= G->MCT.Table.size() && num >= 1)
	{
		if (G->MCT.Table[num - 1].secondaries.size() < 6)
		{
			G->AddManeuverSecondary(num - 1, str, val);
			return true;
		}
	}
	return false;
}

void ShuttleFDOMFD::GetOPMManeuverThresholdTime(char *buf, OMP::OMPDefs::THRESHOLD type, double num)
{
	if (type == OMP::OMPDefs::THRESHOLD::THRES_T)
	{
		MET2String(buf, num);
	}
	else if (type == OMP::OMPDefs::THRESHOLD::THRES_DT || type == OMP::OMPDefs::THRESHOLD::THRES_DLT || type == OMP::OMPDefs::THRESHOLD::THRES_DTL)
	{
		MET2String(buf, num);
	}
	else if (type == OMP::OMPDefs::THRESHOLD::THRES_M)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMP::OMPDefs::THRESHOLD::THRES_APS)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMP::OMPDefs::THRESHOLD::THRES_CAN)
	{
		sprintf_s(buf, 100, "%.1f", num*DEG);
	}
	else if (type == OMP::OMPDefs::THRESHOLD::THRES_N)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMP::OMPDefs::THRESHOLD::THRES_REV)
	{
		sprintf_s(buf, 100, "%.1f", num);
	}
	else if (type == OMP::OMPDefs::THRESHOLD::THRES_WT)
	{
		sprintf_s(buf, 100, "%.1f", num*DEG);
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOMFD::GetOPMManeuverSecondary(char *buf, OMP::OMPDefs::SECONDARIES type, double num)
{
	if (type != OMP::OMPDefs::SECONDARIES::NOSEC)
	{
		if (type == OMP::OMPDefs::SECONDARIES::CXYZ)
		{
			sprintf_s(buf, 100, "%s =%.4f", OMP::GetSecondaryName(type).c_str(), num);
		}
		else
		{
			sprintf_s(buf, 100, "%s =%.1f", OMP::GetSecondaryName(type).c_str(), num);
		}
	}
	else
	{
		sprintf_s(buf, 100, "");
	}
}

void ShuttleFDOMFD::menuDeleteOMPManeuver()
{
	if (G->MCT.Table.size() == 0U) return;

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
	if (num >= 1 && num <= G->MCT.Table.size())
	{
		MCTScroll = 0;
		METScroll = 0;
		MCTSelectedManeuver = 0;
		G->MCT.Table.erase(G->MCT.Table.begin() + num - 1);
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

void ShuttleFDOMFD::menuCalcLTP()
{
	G->CalcLTP();
}

void ShuttleFDOMFD::menuExportLTP()
{
	G->ExportLTP();
}

void ShuttleFDOMFD::menuCalcDeorbitOpportunities()
{
	G->CalcDeorbitOpportunities();
}

void ShuttleFDOMFD::menuCalcDMP()
{
	G->CalcDMP();
}

void ShuttleFDOMFD::menuTransferToMTT()
{
	if (G->MET2MTT())
	{
		menuSetMTTPage();
	}
}

void ShuttleFDOMFD::CalcSupersighter()
{
	G->startSubthread(6);
}

void ShuttleFDOMFD::menuSetIDTInputs()
{
	switch (marker)
	{
	case 0:
		GenericIntInput(&G->IDT_Input_Num, "Instrument identification number (1-25):");
		break;
	case 1:
		GenericStringInput(&G->IDT_Input_Comment, "Instrument identifier (16 char max):");
		break;
	case 2:
		GenericIntInput(&G->IDT_Input_Type, "Instrument type. 3 digits: first axis of rotation, second axis of rotation, axis along center FOV:");
		break;
	case 3:
		GenericIntInput(&G->IDT_Input.Mount, "ID from instrument mount matrix table (1 to 16):");
		break;
	case 4:
		GenericDoubleInput(&G->IDT_Input.Phi1, "Euler angle of rotation about X mount axis to the X', Y', Z' coordinate system:");
		break;
	case 5:
		GenericDoubleInput(&G->IDT_Input.Theta, "Euler angle of rotation about Y' axis to the X'', Y'', Z'' coordinate system:");
		break;
	case 6:
		GenericDoubleInput(&G->IDT_Input.Phi2, "Euler angle of rotation about X'' axis to the instrument coordinate system:");
		break;
	case 7:
		GenericDoubleInput(&G->IDT_Input.A1_MIN, "The minimum angle limit for the first instrument rotation angle:");
		break;
	case 8:
		GenericDoubleInput(&G->IDT_Input.A1_MAX, "The maximum angle limit for the first instrument rotation angle:");
		break;
	case 9:
		GenericDoubleInput(&G->IDT_Input.A2_MIN, "The minimum angle limit for the second instrument rotation angle:");
		break;
	case 10:
		GenericDoubleInput(&G->IDT_Input.A2_MAX, "The maximum angle limit for the second instrument rotation angle:");
		break;
	case 11:
		GenericIntInput(&G->IDT_Input.RET_ID, "Reticle pattern of instrument (0 = no reticle, 1 = hor/vert)");
		break;
	}
}

void ShuttleFDOMFD::IDTCalc()
{
	// Checks
	if (G->IDT_Input_Num < 1 || G->IDT_Input_Num > 25) return;
	if (G->IDT_Input_Comment.size() > 16) return;

	G->IDT[G->IDT_Input_Num - 1].BuildInstrumentData(G->IDT_Input_Comment, G->IDT_Input_Type, G->IDT_Input.Mount, G->IDT_Input.Phi1, G->IDT_Input.Theta, G->IDT_Input.Phi2,
		G->IDT_Input.A1_MIN, G->IDT_Input.A1_MAX, G->IDT_Input.A2_MIN, G->IDT_Input.A2_MAX, G->IDT_Input.RET_ID);
}

void ShuttleFDOMFD::menuSetGroundTargetInputs()
{
	switch (marker)
	{
	case 0:
		GenericIntInput(&G->GTF_Input_Num, "Ground target identification number (1-100):");
		break;
	case 1:
		GenericStringInput(&G->GTF_Input.Name, "Target identifier (20 char max):");
		break;
	case 2:
		GenericDoubleInput(&G->GTF_Input.Lat, "Latitude of the target in degrees (-90 to 90):");
		break;
	case 3:
		GenericDoubleInput(&G->GTF_Input.Lng, "Longitude of the target in degrees (-180 to 180):");
		break;
	case 4:
		GenericDoubleInput(&G->GTF_Input.Alt, "Altitude of target in feet:");
		break;
	}
}

void ShuttleFDOMFD::GroundTargetCalc()
{
	// Check if inputs are valid
	if (G->GTF_Input_Num < 1 || G->GTF_Input_Num >100) return;
	if (G->GTF_Input.Name == "") return;
	if (G->GTF_Input.Lat < -90.0 || G->GTF_Input.Lat > 90.0) return;
	if (G->GTF_Input.Lng < -180.0 || G->GTF_Input.Lng > 180.0) return;

	G->GTF.targets[G->GTF_Input_Num - 1] = G->GTF_Input;
}

void ShuttleFDOMFD::menuSetCheckoutMonitorTime()
{
	MET2String3(Buffer, G->CO_MON_Time);
	GenericMETInput(&G->CO_MON_Time, "Enter desired time. Format: DDD:MM:SS.SSS", Buffer);
}

void ShuttleFDOMFD::CalcCheckoutMonitor()
{
	G->startSubthread(7);
}

void ShuttleFDOMFD::GetMTTThrusterType(char *buf, FDODefs::THRUSTERS type)
{
	G->GetMTTThrusterType(buf, type);
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

void ShuttleFDOMFD::menuMTTModify()
{
	bool MTTChangeManeuverInput(void* id, char* str, void* data);
	oapiOpenInputBox("Change maneuver data (format: MNVR TYPE VALUE)", MTTChangeManeuverInput, 0, 20, (void*)this);
}

bool MTTChangeManeuverInput(void* id, char* str, void* data)
{
	unsigned mnvr;
	char type[128];
	char value[128];

	sprintf_s(type, "");
	sprintf_s(value, "");

	if (sscanf_s(str, "%d %s %s", &mnvr, type, 128, value, 128) == 3)
	{
		return ((ShuttleFDOMFD*)data)->set_MTTManeuverData(mnvr, type, value);
	}
	return false;
}

bool ShuttleFDOMFD::set_MTTManeuverData(unsigned mnvr, const std::string& type, const std::string& value)
{
	return G->ModifyMTTManeuverData(mnvr, type, value);
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
	oapiOpenInputBox("Secondary to delete (format: SEC)", DeleteOMPSecondaryInput, 0, 20, (void*)this);
}

bool DeleteOMPSecondaryInput(void *id, char *str, void *data)
{
	unsigned sec;

	if (sscanf_s(str, "%d", &sec) == 1)
	{
		return ((ShuttleFDOMFD*)data)->delete_OMPSecondary(sec);
	}
	return false;
}

bool ShuttleFDOMFD::delete_OMPSecondary(unsigned sec)
{
	unsigned num = MCTSelectedManeuver + 1;
	if (num >= 1 && num <= G->MCT.Table.size())
	{
		if (sec >= 1 && sec <= G->MCT.Table[num - 1].secondaries.size())
		{
			G->MCT.Table[num - 1].secondaries.erase(G->MCT.Table[num - 1].secondaries.begin() + sec - 1);
			return true;
		}
	}
	return false;
}

void ShuttleFDOMFD::menuInsertOMPManeuver()
{
	if (G->MCT.Table.size() == 0U) return;

	bool InsertMPManeuverInput(void *id, char *str, void *data);
	oapiOpenInputBox("Insert maneuver before currently selected maneuver (format: type name)", InsertMPManeuverInput, 0, 20, (void*)this);
}

bool InsertMPManeuverInput(void *id, char *str, void *data)
{
	char type[32], name[32];

	if (sscanf_s(str, "%s %s", type, 32, name, 32) == 2)
	{
		return ((ShuttleFDOMFD*)data)->insert_OMPManeuver(type, name);
	}
	return false;
}

bool ShuttleFDOMFD::insert_OMPManeuver(char *type, char *name)
{
	unsigned ins = MCTSelectedManeuver + 1;
	if (ins >= 1 && ins <= G->MCT.Table.size() + 1)
	{
		return add_OMPManeuver(type, name, ins);
	}
	return false;
}

void ShuttleFDOMFD::menuSetLaunchDay()
{
	bool LaunchDayInput(void *id, char *str, void *data);
	oapiOpenInputBox("Set launch day (YYYY:DD) or leave blank for current day", LaunchDayInput, 0, 20, (void*)this);
}

bool LaunchDayInput(void *id, char *str, void *data)
{
	int yy, dd;

	if (sscanf_s(str, "%d:%d", &yy, &dd) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LaunchDay(yy, dd);
		return true;
	}
	else if (sscanf_s(str, "") == 0)
	{
		((ShuttleFDOMFD*)data)->set_LaunchDay();
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LaunchDay()
{
	G->SetLaunchDay();
}

void ShuttleFDOMFD::set_LaunchDay(int YY, int DD)
{
	G->SetLaunchDay(YY, DD);
}

void ShuttleFDOMFD::menuSetLaunchTime()
{
	bool LaunchTimeInput(void *id, char *str, void *data);
	oapiOpenInputBox("Set time of liftoff HH:MM:SS.SSS:", LaunchTimeInput, 0, 20, (void*)this);
}

bool LaunchTimeInput(void *id, char *str, void *data)
{
	int hh, mm;
	double ss;

	if (sscanf_s(str, "%d:%d:%lf", &hh, &mm, &ss) == 3)
	{
		((ShuttleFDOMFD*)data)->set_LiftoffTime(hh, mm, ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LiftoffTime(int HH, int MM, double SS)
{
	G->SetLaunchTime(HH, MM, SS);
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

void ShuttleFDOMFD::GetLWPError(char *buf, int err)
{
	switch (err)
	{
	case 3:
		sprintf_s(buf, 100, "Error: ITERV terminated");
		break;
	case 4:
		sprintf_s(buf, 100, "Error: NPLAN did not converge");
		break;
	case 5:
		sprintf_s(buf, 100, "Error: Star table filled");
		break;
	case 6:
		sprintf_s(buf, 100, "Error: GMTLS did not converge");
		break;
	case 8:
		sprintf_s(buf, 100, "Error: RLOT not converging on TYAW");
		break;
	case 9:
		sprintf_s(buf, 100, "Error: PEG4 did not converge");
		break;
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
		papiWriteLine_int(myfile, "LAUNCHDATE0", G->sescnst.Year);
		papiWriteLine_int(myfile, "LAUNCHDATE1", G->sescnst.DayOfYear);
		papiWriteLine_int(myfile, "LAUNCHDATE2", G->sescnst.Hours);
		papiWriteLine_int(myfile, "LAUNCHDATE3", G->sescnst.Minutes);
		papiWriteLine_double(myfile, "LAUNCHDATE4", G->sescnst.launchdateSec);
		if (G->shuttle)
			papiWriteLine_string(myfile, "SHUTTLE", G->shuttle->GetName());
		if (G->target)
			papiWriteLine_string(myfile, "TARGET", G->target->GetName());
		papiWriteLine_bool(myfile, "NONSPHERICAL", G->useNonSphericalGravity);
		myfile << "START_MCT" << std::endl;
		for (unsigned i = 0;i < G->MCT.Table.size();i++)
		{
			WriteMCTLine(myfile, G->MCT.Table[i]);
		}
		myfile << "END_MCT" << std::endl;

		myfile.close();

		// Also save as name for MCT
		G->MCT.Header.Name.assign(filename);
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
		G->MCT.Table.clear();
		G->MCT.Header.Name.assign(filename);

		int Year, Day, Hour, Minute;
		double launchdateSec;
		bool founddate = false, foundtime = false;

		std::string line;
		while (std::getline(myfile, line))
		{
			if (papiReadScenario_int(line.c_str(), "LAUNCHDATE0", Year)) founddate = true;
			if (papiReadScenario_int(line.c_str(), "LAUNCHDATE1", Day)) founddate = true;
			if (papiReadScenario_int(line.c_str(), "LAUNCHDATE2", Hour)) foundtime = true;
			if (papiReadScenario_int(line.c_str(), "LAUNCHDATE3", Minute)) foundtime = true;
			if (papiReadScenario_double(line.c_str(), "LAUNCHDATE4", launchdateSec)) foundtime = true;
			if (papiReadScenario_string(line.c_str(), "SHUTTLE", shuttlebuff))
			{
				G->chaserSVOption = false;

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
			}
			if (papiReadScenario_string(line.c_str(), "TARGET", targetbuff))
			{
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
			papiReadScenario_bool(line.c_str(), "NONSPHERICAL", G->useNonSphericalGravity);
			if (strcmp(line.c_str(), "END_MCT") == 0) isMCT = false;
			if (isMCT) ReadMCTLine(line.c_str());
			if (strcmp(line.c_str(), "START_MCT") == 0) isMCT = true;
		}

		myfile.close();

		//Process times
		if (founddate) G->SetLaunchDay(Year, Day);
		if (foundtime) G->SetLaunchTime(Hour, Minute, launchdateSec);

		return true;
	}

	return false;
}

void ShuttleFDOMFD::WriteMCTLine(std::ofstream &file, OMP::ManeuverConstraints &constr)
{
	std::string sectype[OMP::MAXSECONDARIES];
	double secnum[OMP::MAXSECONDARIES];
	for (unsigned i = 0;i < OMP::MAXSECONDARIES;i++)
	{
		sectype[i] = "NSEC";
		secnum[i] = 0.0;
	}
	for (unsigned i = 0;i < constr.secondaries.size();i++)
	{
		sectype[i] = OMP::GetSecondaryName(constr.secondaries[i].type);
		secnum[i] = constr.secondaries[i].value;
	}

	sprintf_s(Buffer, 100, "%s %d %d %lf %s %lf %s %lf %s %lf %s %lf", constr.name.c_str(), constr.type, constr.threshold, constr.thresh_num,
		sectype[0].c_str(), secnum[0], sectype[1].c_str(), secnum[1], sectype[2].c_str(), secnum[2], sectype[3].c_str(), secnum[3]);
	file << Buffer << std::endl;
}

void ShuttleFDOMFD::ReadMCTLine(const char *line)
{
	OMP::SecData sec;
	unsigned i = 0;
	char name[64];
	char sectype[OMP::MAXSECONDARIES][5];
	double secnum[OMP::MAXSECONDARIES];
	OMP::ManeuverConstraints temp;

	sprintf(name, "");
	for (i = 0;i < OMP::MAXSECONDARIES;i++)
	{
		sprintf_s(sectype[i], 5, "NSEC");
		secnum[i] = 0.0;
	}
	

	if (sscanf_s(line, "%s %d %d %lf %s %lf %s %lf %s %lf %s %lf", name, 64, &temp.type, &temp.threshold, &temp.thresh_num,
		sectype[0], 5, &secnum[0], sectype[1], 5, &secnum[1], sectype[2], 5, &secnum[2], sectype[3], 5, &secnum[3]) == 12)
	{
		temp.name.assign(name);
		i = 0;
		G->MCT.Table.push_back(temp);
		while (strcmp(sectype[i], "NSEC") && i < OMP::MAXSECONDARIES)
		{
			sec.type = OMP::GetSecondaryType(sectype[i]);
			sec.value = secnum[i];
			G->MCT.Table.back().secondaries.push_back(sec);
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
	if (METScroll + FDOMFD_MET_MAX_MANEUVERS < G->ManeuverEvaluationTable.Maneuvers.size()) METScroll++;
}

void ShuttleFDOMFD::menuScrollMCTUp()
{
	if (MCTSelectedManeuver > 0) MCTSelectedManeuver--;
	if (MCTSelectedManeuver < MCTScroll) MCTScroll--;
}

void ShuttleFDOMFD::menuScrollMCTDown()
{
	if ((MCTSelectedManeuver + 1) < G->MCT.Table.size()) MCTSelectedManeuver++;

	if ((MCTSelectedManeuver + 1) > (MCTScroll + FDOMFD_MCT_MAX_MANEUVERS)) MCTScroll++;
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
	if (G->LWP_LaunchSite < 3)
	{
		G->LWP_LaunchSite++;
		set_LWP_LS(G->LWP_LaunchSite);
	}
	else
	{
		G->LWP_LaunchSite = 0;
		G->LWP_Settings.LATLS = 0.0;
		G->LWP_Settings.LONGLS = 0.0;
	}
}

void ShuttleFDOMFD::set_LWP_LS(int ls)
{
	G->LWP_Settings.LATLS = LAUNCHSITE_LATITUDE[ls - 1]*RAD;
	G->LWP_Settings.LONGLS = LAUNCHSITE_LONGITUDE[ls - 1]*RAD;
}

void ShuttleFDOMFD::menuLWPSetLSLatLng()
{
	if (G->LWP_LaunchSite == 0)
	{
		bool LWPLSLatLngInput(void *id, char *str, void *data);
		oapiOpenInputBox("Input launch site latitude and longitude in degrees:", LWPLSLatLngInput, 0, 20, (void*)this);
	}
}

bool LWPLSLatLngInput(void *id, char *str, void *data)
{
	double lat, lng;

	if (sscanf_s(str, "%lf %lf", &lat, &lng) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_LSLatLng(lat, lng);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_LSLatLng(double lat, double lng)
{
	G->LWP_Settings.LATLS = lat * RAD;
	G->LWP_Settings.LONGLS = lng * RAD;
}

void ShuttleFDOMFD::menuLWPLaunchAzimuthDirectionFlag()
{
	if (G->LWP_Settings.NS == 0)
	{
		G->LWP_Settings.NS = 1;
	}
	else
	{
		G->LWP_Settings.NS = 0;
	}
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
	oapiOpenInputBox("Input insertion altitude in nautical miles:", LWPRADInput, 0, 20, (void*)this);
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

void ShuttleFDOMFD::set_LWP_RAD(double alt)
{
	G->LWP_Settings.RINS = OrbMech::EARTH_RADIUS_EQUATOR + alt * NM2M;
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
	G->LWP_Settings.VINS = vel * FPS2MPS;
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
	G->LWP_Settings.DTIG_ET_SEP = dt;
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
	G->LWP_Settings.DTIG_MPS = dt;
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
	G->LWP_Settings.DV_ET_SEP = DV * FPS2MPS;
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
	G->LWP_Settings.DV_MPS = DV * FPS2MPS;
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
	oapiOpenInputBox("Input phase flag (0 = 0° to 360°, 1 = -360° to 0°, 2 = -180° to 180°):", LWPPHASEFLAGInput, 0, 20, (void*)this);
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
	oapiOpenInputBox("Input wrap flag (adds N*360° to phase angle):", LWPWRAPFLAGInput, 0, 20, (void*)this);
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

void ShuttleFDOMFD::menuSetLTPLaunchTime()
{
	bool LTPLaunchTimeInput(void *id, char *str, void *data);
	oapiOpenInputBox("Set time of liftoff HH:MM:SS.SSS:", LTPLaunchTimeInput, 0, 20, (void*)this);
}

bool LTPLaunchTimeInput(void *id, char *str, void *data)
{
	int hh, mm;
	double ss;

	if (sscanf_s(str, "%d:%d:%lf", &hh, &mm, &ss) == 3)
	{
		((ShuttleFDOMFD*)data)->set_LTPLiftoffTime(hh*3600.0 + mm*60.0 + ss);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LTPLiftoffTime(double gmt)
{
	G->LWP_Settings.GMTLOR = gmt;
}

void ShuttleFDOMFD::menuSetLW_OMS1_DTIG()
{
	GenericMETInput(&G->LWP_Settings.OMS1.DTIG, "OMS-1 TIG from ET sep in DD:HH:MM:SS.SSS");
}

void ShuttleFDOMFD::menuSetLW_OMS2_DTIG()
{
	GenericMETInput(&G->LWP_Settings.OMS2.DTIG, "OMS-2 TIG from ET sep in DD:HH:MM:SS.SSS");
}

void ShuttleFDOMFD::menuSetLWOMS1_C1_C2()
{
	bool LWP_OMS1_C1_C2_Input(void *id, char *str, void *data);
	oapiOpenInputBox("OMS-1 target intercept and slope:", LWP_OMS1_C1_C2_Input, 0, 20, (void*)this);
}

bool LWP_OMS1_C1_C2_Input(void *id, char *str, void *data)
{
	double C1, C2;

	if (sscanf_s(str, "%lf %lf", &C1, &C2) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_OMS_C1_C2(true, C1, C2);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::menuSetLWOMS2_C1_C2()
{
	bool LWP_OMS2_C1_C2_Input(void *id, char *str, void *data);
	oapiOpenInputBox("OMS-2 target intercept and slope:", LWP_OMS2_C1_C2_Input, 0, 20, (void*)this);
}

bool LWP_OMS2_C1_C2_Input(void *id, char *str, void *data)
{
	double C1, C2;

	if (sscanf_s(str, "%lf %lf", &C1, &C2) == 2)
	{
		((ShuttleFDOMFD*)data)->set_LWP_OMS_C1_C2(false, C1, C2);
		return true;
	}
	return false;
}

void ShuttleFDOMFD::set_LWP_OMS_C1_C2(bool OMS1, double C1, double C2)
{
	if (OMS1)
	{
		G->LWP_Settings.OMS1.C1 = C1 * 0.3048;
		G->LWP_Settings.OMS1.C2 = C2;
	}
	else
	{
		G->LWP_Settings.OMS2.C1 = C1 * 0.3048;
		G->LWP_Settings.OMS2.C2 = C2;
	}
}

void ShuttleFDOMFD::menuSetLW_OMS1_HT()
{
	GenericDoubleInput(&G->LWP_Settings.OMS1.HTGT, "OMS-1 height target in nautical miles:", 1852.0);
}

void ShuttleFDOMFD::menuSetLW_OMS2_HT()
{
	GenericDoubleInput(&G->LWP_Settings.OMS2.HTGT, "OMS-2 height target in nautical miles:", 1852.0);
}

void ShuttleFDOMFD::menuSetLW_OMS1_Theta()
{
	GenericDoubleInput(&G->LWP_Settings.OMS1.THETA, "OMS-1 target downrange angle from launch site in degrees:", RAD);
}

void ShuttleFDOMFD::menuSetLW_OMS2_Theta()
{
	GenericDoubleInput(&G->LWP_Settings.OMS2.THETA, "OMS-2 target downrange angle from launch site in degrees:", RAD);
}

void ShuttleFDOMFD::menuDOPSSetGETS()
{
	GenericMETInput(&G->DOPS_GETS, "MET for start of search in DD:HH:MM:SS");
}

void ShuttleFDOMFD::menuDOPSSetGETF()
{
	GenericMETInput(&G->DOPS_GETF, "MET for end of search in DD:HH:MM:SS");
}

void ShuttleFDOMFD::menuDOPSSetRev()
{
	GenericIntInput(&G->DOPS_InitialRev, "Set initial rev counter:");
}

void ShuttleFDOMFD::menuDOPSSetMaxXRNG()
{
	GenericDoubleInput(&G->DOPS_MaxXRNG, "Maximum crossrange in nautical miles:");
}

void ShuttleFDOMFD::menuCycleDOPSSites()
{
	G->DOPS_ConUS = !G->DOPS_ConUS;
}

void ShuttleFDOMFD::menuCycleDOPSPage()
{
	if (G->DOPS_Page < G->DOPS_MaxPage)
	{
		G->DOPS_Page++;
	}
	else
	{
		G->DOPS_Page = 0;
	}
}

void ShuttleFDOMFD::menuDMPCycleTIGOption()
{
	if (G->DMPOpt.ITIGFR < 1)
	{
		G->DMPOpt.ITIGFR = 1;
	}
	else
	{
		G->DMPOpt.ITIGFR = 0;
	}
}

void ShuttleFDOMFD::menuDMPInputTIG()
{
	if (G->DMPOpt.ITIGFR == 0)
	{
		GenericMETInput(&G->DMPOpt.TIG, "Input time of ignition in DD:HH:MM:SS");
	}
	else
	{
		GenericMETInput(&G->DMPOpt.TTHRSH, "Input threshold time in DD:HH:MM:SS");
	}
}

void ShuttleFDOMFD::menuDMPInputPropellantWaste()
{
	GenericDoubleInput(&G->DMPOpt.WCGOMS, "Enter propellant to be wasted in pounds (0 for in-plane):", LBM2KG);
}

void ShuttleFDOMFD::menuDMPCyclePrimaryThruster()
{
	if (G->DMPOpt.INGPR < 16)
	{
		G->DMPOpt.INGPR += 2;
	}
	else
	{
		G->DMPOpt.INGPR = 12;
	}
}

void ShuttleFDOMFD::menuDMPCycleBackupThruster()
{
	if (G->DMPOpt.INGBU < 16)
	{
		G->DMPOpt.INGBU += 2;
	}
	else
	{
		G->DMPOpt.INGBU = 12;
	}
}

void ShuttleFDOMFD::menuDMPLandingSite()
{
	GenericStringInput(&G->DMPLandingSite, "Input landing site:");
}

void ShuttleFDOMFD::menuSetSupersighterInputs()
{
	switch (marker)
	{
	case 0: // Mode
		if (G->SSInputs.Mode < 7) G->SSInputs.Mode++;
		else G->SSInputs.Mode = 1;
		break;
	case 1: // Source matrix
		if (G->SSInputs.INMAT == "RLMT01")
		{
			G->SSInputs.INMAT = "RFMT01";
		}
		else if (G->SSInputs.INMAT == "RFMT01")
		{
			G->SSInputs.INMAT = "LPYR";
		}
		else if (G->SSInputs.INMAT == "LPYR")
		{
			G->SSInputs.INMAT = "LYPR";
		}
		else
		{
			G->SSInputs.INMAT = "RLMT01";
		}
		break;
	case 2: // Desired matrix
		if (G->SSInputs.OUTMAT == "RFMT01")
		{
			G->SSInputs.OUTMAT = "RLMT01";
		}
		else
		{
			G->SSInputs.OUTMAT = "RFMT01";
		}
		break;
	case 3: // Ephemeris ID
		break;
	case 4: // Elevation angle
		GenericDoubleInput(&G->SSInputs.ELV, "Enter elevation angle for AOS calculations in degrees:");
		break;
	case 5: // Target 1 ID
		GenericStringInput(&G->SSInputs.TGT1, "Input target 1:");
		break;
	case 6: // Target 2 ID
		GenericStringInput(&G->SSInputs.TGT2, "Input target 2:");
		break;
	case 7: // Instrument IA1
		GenericStringInput(&G->SSInputs.IA1, "Input instrument IA1:");
		break;
	case 8: // Instrument IA2
		GenericStringInput(&G->SSInputs.IA2, "Input instrument IA2:");
		break;
	case 9: // Instrument IB1
		GenericStringInput(&G->SSInputs.IB1, "Input instrument IB1:");
		break;
	case 10: // Instrument IB2
		GenericStringInput(&G->SSInputs.IB2, "Input instrument IB2:");
		break;
	case 11: // Att Sense
		if (G->SSInputs.ATTSense < 2) G->SSInputs.ATTSense++;
		else G->SSInputs.ATTSense = 0;
		break;
	case 12: // ATT
		GenericVectorInput(&G->SSInputs.ATT, "Enter desired attitude in degrees. Format: Roll, Pitch, Yaw.");
		break;
	case 13: // IA1 A1, A2
		GenericDouble2Input(&G->SSInputs.IA1_A1, &G->SSInputs.IA1_A2, "Enter angles for instrument IA1 in degrees. Format: A1 A2");
		break;
	case 14: // IA2 A1, A2
		GenericDouble2Input(&G->SSInputs.IA2_A1, &G->SSInputs.IA2_A2, "Enter angles for instrument IA2 in degrees. Format: A1 A2");
		break;
	case 15: // IB1 A1, A2
		GenericDouble2Input(&G->SSInputs.IB1_A1, &G->SSInputs.IB1_A2, "Enter angles for instrument IB1 in degrees. Format: A1 A2");
		break;
	case 16: // IB2 A1, A2
		GenericDouble2Input(&G->SSInputs.IB2_A1, &G->SSInputs.IB2_A2, "Enter angles for instrument IB2 in degrees. Format: A1 A2");
		break;
	case 17: // MGA
		GenericDoubleInput(&G->SSInputs.MGA, "Enter desired middle gimbal angle in degrees:");
		break;
	case 18: // OMI
		GenericDoubleInput(&G->SSInputs.OMI, "Enter desired omicron angle in degrees:");
		break;
	case 19: // EIG
		GenericVectorInput(&G->SSInputs.EIG, "Enter desired eigen axis and angle in degrees. Format: Pitch, Yaw, Eigen angle.");
		break;
	case 20: // Start time
		GenericMETInput(&G->SSInputs.StartTime, "Start time for computations in MET. Format: DDD:HH:MM:SS");
		break;
	case 21: // Source bias matrix
		break;
	case 22: // Desired bias matrix
		break;
	}
}

void ShuttleFDOMFD::GenericStringInput(std::string *val, char* message)
{
	bool GenericStringInputBox(void *id, char *str, void *data);
	oapiOpenInputBox(message, GenericStringInputBox, 0, 25, (void*)(val));
}

bool GenericStringInputBox(void *id, char *str, void *data)
{
	std::string *str2 = static_cast<std::string*>(data);

	std::string str3;

	str3.assign(str);
	*str2 = str3;

	return true;
}

void ShuttleFDOMFD::GenericMETInput(double *get, char *message, char* default_string)
{
	bool GenericMETInputBox(void *id, char *str, void *data);
	oapiOpenInputBox(message, GenericMETInputBox, default_string, 25, (void*)(get));
}

bool GenericMETInputBox(void *id, char *str, void *data)
{
	double *get2 = static_cast<double*>(data);

	int dd, hh, mm;
	double ss, get;

	if (sscanf(str, "%d:%d:%d:%lf", &dd, &hh, &mm, &ss) == 4)
	{
		get = ss + 60 * (mm + 60 * (hh + 24 * dd));
		*get2 = get;

		return true;

	}
	return false;
}

void ShuttleFDOMFD::GenericIntInput(int *val, char *message)
{
	void *data2;

	tempData.iVal = val;
	data2 = &tempData;

	bool GenericIntInputBox(void *id, char *str, void *data);
	oapiOpenInputBox(message, GenericIntInputBox, 0, 25, data2);
}

bool GenericIntInputBox(void *id, char *str, void *data)
{
	ShuttleFDOMFDInputBoxData *arr = static_cast<ShuttleFDOMFDInputBoxData*>(data);
	int val;

	if (sscanf(str, "%d", &val) == 1)
	{
		*arr->iVal = val;
		return true;
	}
	return false;
}

void ShuttleFDOMFD::GenericDoubleInput(double *val, char *message, double factor)
{
	void *data2;

	tempData.dVal = val;
	tempData.factor = factor;
	data2 = &tempData;

	bool GenericDoubleInputBox(void *id, char *str, void *data);
	oapiOpenInputBox(message, GenericDoubleInputBox, 0, 25, data2);
}

bool GenericDoubleInputBox(void *id, char *str, void *data)
{
	ShuttleFDOMFDInputBoxData *arr = static_cast<ShuttleFDOMFDInputBoxData*>(data);
	double val;

	if (sscanf(str, "%lf", &val) == 1)
	{
		*arr->dVal = val * arr->factor;
		return true;
	}
	return false;
}

void ShuttleFDOMFD::GenericDouble2Input(double* val1, double* val2, char* message, double factor1, double factor2)
{
	void* data2;

	tempData.dVal = val1;
	tempData.dVal2 = val2;
	tempData.factor = factor1;
	tempData.factor2 = factor2;
	data2 = &tempData;

	bool GenericDouble2InputBox(void* id, char* str, void* data);
	oapiOpenInputBox(message, GenericDouble2InputBox, 0, 30, data2);
}

bool GenericDouble2InputBox(void* id, char* str, void* data)
{
	ShuttleFDOMFDInputBoxData* arr = static_cast<ShuttleFDOMFDInputBoxData*>(data);
	double val1, val2;

	if (sscanf(str, "%lf %lf", &val1, &val2) == 2)
	{
		*arr->dVal = val1 * arr->factor;
		*arr->dVal2 = val2 * arr->factor2;
		return true;
	}
	return false;
}

void ShuttleFDOMFD::GenericVectorInput(VECTOR3* val, char* message, double factor)
{
	void* data2;

	tempData.vVal = val;
	tempData.factor = factor;
	data2 = &tempData;

	bool GenericVectorInputBox(void* id, char* str, void* data);
	oapiOpenInputBox(message, GenericVectorInputBox, 0, 25, data2);
}

bool GenericVectorInputBox(void* id, char* str, void* data)
{
	ShuttleFDOMFDInputBoxData* arr = static_cast<ShuttleFDOMFDInputBoxData*>(data);
	double val1, val2, val3;

	if (sscanf(str, "%lf %lf %lf", &val1, &val2, &val3) == 3)
	{
		arr->vVal->x = val1 * arr->factor;
		arr->vVal->y = val2 * arr->factor;
		arr->vVal->z = val3 * arr->factor;
		return true;
	}
	return false;
}

void ShuttleFDOMFD::GetCharSize(oapi::Sketchpad* skp, int& CW, int& CH)
{
	DWORD charsize = skp->GetCharSize();
	CW = HIWORD(charsize);
	CH = LOWORD(charsize);
}