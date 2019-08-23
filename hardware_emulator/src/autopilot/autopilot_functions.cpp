/* (c) https://github.com/MontiCore/monticore */
#include "autopilot_functions.h"

const char *AutopilotFunction::module_name = "Java_simulator_integration_AutopilotAdapter_";

AutopilotFunction AutopilotFunction::autopilot_inputs[AUTOPILOT_INPUT_COUNT] = {
#include "autopilot_inputs.h"
};


AutopilotFunction AutopilotFunction::autopilot_outputs[AUTOPILOT_OUTPUT_COUNT] = {
#include "autopilot_outputs.h"
};
