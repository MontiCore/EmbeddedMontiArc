#pragma once

#include "buffer.h"
#include "json.h"


// The interface to the autopilot
namespace autopilot {

extern DynamicBuffer buffer; // Buffer used by all "writers" (json & binary) & Socket input buffer

void set_port_json(int32_t i, const char* data);
void get_port_json(int32_t i, JsonWriter &writer);
void set_port_binary(int32_t i, BinaryReader &br);
void get_port_binary(int32_t i, BinaryWriter &bw);

// Methods
void init();
void execute(double delta_sec);

extern const bool IS_SOCKET[];
extern const bool IS_OUTPUT[];
extern const int32_t PORT_COUNT;
extern const char* PROGRAM_INTERFACE;

}


