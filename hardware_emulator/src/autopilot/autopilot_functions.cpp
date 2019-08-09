/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#include "autopilot_functions.h"

const char *AutopilotFunction::module_name = "Java_simulator_integration_AutopilotAdapter_";

AutopilotFunction AutopilotFunction::autopilot_inputs[AUTOPILOT_INPUT_COUNT] = {
#include "autopilot_inputs.h"
};


AutopilotFunction AutopilotFunction::autopilot_outputs[AUTOPILOT_OUTPUT_COUNT] = {
#include "autopilot_outputs.h"
};
