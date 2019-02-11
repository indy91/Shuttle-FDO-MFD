/****************************************************************************
  This file is part of Shuttle FDO MFD for Orbiter Space Flight Simulator
  Copyright (C) 2019 Niklas Beug

  Shuttle FDO MFD Module (Header)

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

#include "Orbitersdk.h"

class ShuttleFDOoapiModule : public oapi::Module {
public:
	ShuttleFDOoapiModule(HINSTANCE hDLL);
	~ShuttleFDOoapiModule();
	void clbkSimulationStart(RenderMode mode);
	void clbkSimulationEnd();
	void clbkPreStep(double simt, double simdt, double mjd);
	void clbkPostStep(double simt, double simdt, double mjd);
	void clbkDeleteVessel(OBJHANDLE hVessel);
	static int MsgProc(UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);

};