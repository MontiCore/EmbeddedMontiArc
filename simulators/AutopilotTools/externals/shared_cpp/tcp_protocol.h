#pragma once
#include <string>
#include <exception>

// SEE https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/dev-docs/concepts/TCP-protocol

/*
    PACKET IDS
*/
constexpr int PACKET_END = 0; // The simulator is closing the connection after
constexpr int PACKET_ERROR = 1; // As payload: An error message
constexpr int PACKET_INIT = 2; // As payload: TimeMode string ("measured" or "realtime")
constexpr int PACKET_INTERFACE = 3; // As payload: TimeMode string ("measured" or "realtime")
// Payload for INPUT and OUTPUT packets:
// uint16_t: Port ID (as defined in the DynamicInterface -> Order of appearance)
// Type dependent payload
constexpr int PACKET_INPUT_BINARY = 4;
constexpr int PACKET_OUTPUT_BINARY = 5;
constexpr int PACKET_RUN_CYCLE = 6; // Payload: double: delta_sec
constexpr int PACKET_TIME = 7; // Payload: double: seconds
constexpr int PACKET_REF_ID = 8; // Payload: uint32_t: reference id for the DDC exchange
constexpr int PACKET_PING = 9; // No Payload
constexpr int PACKET_EMU_ID = 10; // Payload: byte with ID for the new remote emulator. The emulator expects PACKET_CONFIG as response. This is sent before the PACKET_INTERFACE
constexpr int PACKET_CONFIG = 11; // Payload: JSON string, response to PACKET_REQUEST_CONFIG
constexpr int PACKET_RECONNECT = 12; // Payload: Emulator ID (byte), sent to remote hardware_emulator. Expects remote emulator to respond with PACKET_INTERFACE.
constexpr int PACKET_INPUT_JSON = 13;
constexpr int PACKET_OUTPUT_JSON = 14;