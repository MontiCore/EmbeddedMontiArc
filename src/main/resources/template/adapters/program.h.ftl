#pragma once

#include "buffer.h"
#include "json.h"


// The interface to the autopilot
namespace autopilot {

extern DynamicBuffer buffer; // Buffer used by all "writers" (json & binary) & Socket input buffer

void set_port_json(int i, const char* data);
void get_port_json(int i, JsonWriter &writer);
void set_port_binary(int i, BinaryReader &br);
void get_port_binary(int i, BinaryWriter &bw);

// Methods
void init();
void execute(double delta_sec);

extern const bool IS_SOCKET[];
extern const bool IS_OUTPUT[];
extern const int PORT_COUNT;
extern const char* PROGRAM_INTERFACE;

}


