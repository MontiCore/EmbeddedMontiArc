#pragma once
#include <string>
#include <exception>
#include <chrono>




using HRClock = std::chrono::high_resolution_clock;
using TimePoint = HRClock::time_point;
using duration = std::chrono::duration<double>;


enum TimeMode {
    REALTIME,
    MEASURED
};


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
constexpr int PACKET_INPUT = 4;
constexpr int PACKET_OUTPUT = 5;
constexpr int PACKET_RUN_CYCLE = 6; // Payload: double: delta_sec
constexpr int PACKET_TIME = 7; // Payload: double: seconds
constexpr int PACKET_REF_ID = 8; // Payload: uint32_t: reference id for the DDC exchange
constexpr int PACKET_PING = 9; // No payload


