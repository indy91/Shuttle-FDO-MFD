/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD (Header)

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

#include "ShuttleFDOCore.h"

struct ShuttleFDOMFDInputBoxData
{
	double *dVal;
	int *iVal;
	VECTOR3 *vVal;
	double factor;
};

class ShuttleFDOMFD : public MFD2 {
public:
	ShuttleFDOMFD(DWORD w, DWORD h, VESSEL *v, UINT im);
	~ShuttleFDOMFD();
	char *ButtonLabel (int bt);
	int ButtonMenu (const MFDBUTTONMENU **menu) const;
	bool Update(oapi::Sketchpad *skp);
	bool ConsumeButton(int bt, int event);
	bool ConsumeKeyBuffered(DWORD key);
	//void WriteStatus(FILEHANDLE scn) const;
	//void ReadStatus(FILEHANDLE scn);

	void menuSaveState();
	bool SaveState(char *filename);
	void menuLoadState();
	bool LoadState(char *filename);

	void menuVoid() {};
	void menuAddOMPManeuver();
	void menuAddOMPThreshold();
	void menuAddOMPSecondary();
	void menuModifyOMPManeuver();
	void menuInsertOMPManeuver();
	void menuDeleteOMPManeuver();
	void menuCalculateOMPPlan();
	void menuCalcLaunchTime();
	void menuCalcLTP();
	void menuExportLTP();
	void menuTransferToMTT();
	void menuMTTChangeSlot();
	void menuExecuteMTT();
	void menuDMTChooseManeuver();
	void menuCalcDMT();
	void menuDeleteOMPSecondary();
	void menuSetLaunchDay();
	void menuSetLaunchTime();
	void set_target();
	void set_shuttle();
	void menuCycleGravityOption();
	void menuModifySecondary();
	void menuScrollMCTUp();
	void menuScrollMCTDown();
	void menuScrollMETUp();
	void menuScrollMETDown();
	void menuLWPSetDELNO();
	void menuLWPSetDTOPT();
	void menuLWPSetLS();
	void menuLWPSetLSLatLng();
	void menuLWPLaunchAzimuthDirectionFlag();
	void menuLWPSetYS();
	void menuLWPSetPFA();
	void menuLWPSetPFT();
	void menuLWPSetFPA();
	void menuLWPSetRAD();
	void menuLWPSetVEL();
	void menuLWPSetDTETSEP();
	void menuLWPSetDVETSEP();
	void menuLWPSetDTMPS();
	void menuLWPSetDVMPS();
	void menuLWPSetWT();
	void menuLWPSetDTO();
	void menuLWPSetDTC();
	void menuLWPSetPHASEFLAG();
	void menuLWPSetWRAPFLAG();
	void menuSetLTPLaunchTime();

	void menuCalcDeorbitOpportunities();
	void menuDOPSSetGETS();
	void menuDOPSSetGETF();
	void menuDOPSSetRev();
	void menuDOPSSetMaxXRNG();

	void menuCalcDMP();
	void menuDMPCycleTIGOption();
	void menuDMPInputTIG();
	void menuDMPCyclePrimaryThruster();
	void menuDMPCycleBackupThruster();
	void menuDMPLandingSite();

	void menuSetMainMenu();
	void menuSetMCTPage();
	void menuSetMETPage();
	void menuSetMTTPage();
	void menuSetLWPPage();
	void menuSetDMTPage();
	void menuSetConfigurationMenu();
	void menuSetLWPPage2();
	void menuSetLWPPage3();
	void menuSetDOPSPage();
	void menuSetDMPPage();
	void menuSetLTPPage();
	void SetScreen(int s);

	bool add_OMPManeuver(char *type, char *name, unsigned ins);
	bool delete_OMPManeuver(unsigned num);
	bool modify_OMPManeuver(unsigned num, char *type, char *name);
	bool add_OMPManeuverThreshold(unsigned num, char *type, char * str);
	bool add_OMPManeuverSecondary(unsigned num, char * str, double val);
	bool modify_OMPManeuverSecondary(unsigned man, unsigned sec, char * str, double val);
	bool set_MTTManeuverSlot(unsigned mnvr, int slot);
	void set_DMTManeuver(unsigned mnvr);
	bool delete_OMPSecondary(unsigned num, unsigned sec);
	bool insert_OMPManeuver(unsigned ins, char *type, char *name);
	void set_LaunchDay();
	void set_LaunchDay(int YY, int DD);
	void set_LiftoffTime(int HH, int MM, double SS);
	void WriteMCTLine(std::ofstream &file, ManeuverConstraints &constr);
	void ReadMCTLine(const char *line);
	void set_LWP_DELNO(double delno);
	void set_LWP_DTOPT(double dtopt);
	void set_LWP_LS(int ls);
	void set_LWP_LSLatLng(double lat, double lng);
	void set_LWP_YS(double ys);
	void set_LWP_PFA(double pfa);
	void set_LWP_PFT(double pft);
	void set_LWP_FPA(double fpa);
	void set_LWP_RAD(double rad);
	void set_LWP_VEL(double vel);
	void set_LWP_DTETSEP(double dt);
	void set_LWP_DVETSEP(VECTOR3 DV);
	void set_LWP_DTMPS(double dt);
	void set_LWP_DVMPS(VECTOR3 DV);
	void set_LWP_WT(double wt);
	void set_LWP_DTO(double dt);
	void set_LWP_DTC(double dt);
	void set_LWP_PHASEFLAG(int flag);
	void set_LWP_WRAPFLAG(int flag);
	void set_LTPLiftoffTime(double gmt);

	void MET2String(char *buf, double MET);
	void MET2String2(char *buf, double MET);
	void DMTMET2String(char *buf, double MET);
	void GMT2String(char *buf, double GMT);
	void GMT2String2(char *buf, double GMT);
	void LWPGMT2String(char *buf, double GMT);
	void LTPGMT2String(char *buf, double GMT);
	double DDDHHHMMSS2MET(int dd, int hh, int mm, double ss);
	void SS2HHMMSS(double val, double &hh, double &mm, double &ss);
	void SS2MMSS(double val, double &mm, double &ss);

	void GetOPMManeuverType(char *buf, OMPDefs::MANTYPE type);
	void GetOPMManeuverThreshold(char *buf, OMPDefs::THRESHOLD type);
	void GetOPMManeuverThresholdTime(char *buf, OMPDefs::THRESHOLD type, double num);
	void GetOPMManeuverSecondary(char *buf, char *type, double num);
	void GetMTTThrusterType(char *buf, OMPDefs::THRUSTERS type);
	void GetMTTGuidanceType(char *buf, OMPDefs::GUID type);
	void GetOMPError(char *buf, int err);
	void GetLWPError(char *buf, int err);

protected:

	void GenericIntInput(int *val, char* message);
	void GenericMETInput(double *get, char *message);
	void GenericDoubleInput(double *val, char* message, double factor = 1.0);
	void GenericStringInput(std::string *val, char* message);

	oapi::Font *font;
	oapi::Font *font2;

	int screen;

	ShuttleFDOMFDButtons coreButtons;
	ShuttleFDOCore* G;
	ShuttleFDOMFDInputBoxData tempData;

	char Buffer[100];

	bool MTTFlag;
	unsigned MCTScroll;
	unsigned METScroll;
};