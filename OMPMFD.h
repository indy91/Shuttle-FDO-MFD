/****************************************************************************
  This file is part of OMP MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  OMP MFD (Header)

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

#include "OMPCore.h"

class OMPMFD : public MFD2 {
public:
	OMPMFD(DWORD w, DWORD h, VESSEL *v, UINT im);
	~OMPMFD();
	char *ButtonLabel (int bt);
	int ButtonMenu (const MFDBUTTONMENU **menu) const;
	bool Update(oapi::Sketchpad *skp);
	bool ConsumeButton(int bt, int event);
	bool ConsumeKeyBuffered(DWORD key);
	void WriteStatus(FILEHANDLE scn) const;
	void ReadStatus(FILEHANDLE scn);

	void menuVoid() {};
	void menuAddOMPManeuver();
	void menuAddOMPThreshold();
	void menuAddOMPSecondary();
	void menuModifyOMPManeuver();
	void menuInsertOMPManeuver();
	void menuDeleteOMPManeuver();
	void menuCalculateOMPPlan();
	void menuCalcLaunchTime();
	void menuTransferToMTT();
	void menuMTTChangeSlot();
	void menuExecuteMTT();
	void menuDMTChooseManeuver();
	void menuCalcDMT();
	void menuDeleteOMPSecondary();
	void menuSetLiftoffTime();
	void set_target();
	void menuCycleGravityOption();

	void menuSetMainMenu();
	void menuSetMCTPage();
	void menuSetMETPage();
	void menuSetMTTPage();
	void menuSetLaunchWindowPage();
	void menuSetDMTPage();
	void menuSetOMPExeMenu();

	bool add_OMPManeuver(char *type, char *name, unsigned ins);
	bool delete_OMPManeuver(unsigned num);
	bool modify_OMPManeuver(unsigned num, char *type, char *name);
	bool add_OMPManeuverThreshold(unsigned num, char *type, char * str);
	bool add_OMPManeuverSecondary(unsigned num, char * str, double val);
	bool set_MTTManeuverSlot(unsigned mnvr, int slot);
	void set_DMTManeuver(unsigned mnvr);
	bool delete_OMPSecondary(unsigned num, unsigned sec);
	bool insert_OMPManeuver(unsigned ins, char *type, char *name);
	void set_LiftoffTime(int YY, int DD, int HH, int MM, double SS);

	void MET2String(char *buf, double MET);
	void DMTMET2String(char *buf, double MET);
	void GMT2String(char *buf, double GMT);
	double DDDHHHMMSS2MET(int dd, int hh, int mm, double ss);
	void SS2HHMMSS(double val, double &hh, double &mm, double &ss);

	void GetOPMManeuverType(char *buf, OMPDefs::MANTYPE type);
	void GetOPMManeuverThreshold(char *buf, OMPDefs::THRESHOLD type);
	void GetOPMManeuverThresholdTime(char *buf, OMPDefs::THRESHOLD type, double num);
	void GetOPMManeuverSecondary(char *buf, char *type, double num);
	void GetMTTThrusterType(char *buf, OMPDefs::THRUSTERS type);
	void GetMTTGuidanceType(char *buf, OMPDefs::GUID type);
	void GetOMPError(char *buf, int err);

protected:
	oapi::Font *font;
	oapi::Font *font2;

	int screen;

	OMPMFDButtons coreButtons;
	OMPCore* G;

	char Buffer[100];

	bool MTTFlag;
};