#pragma once


constexpr int MIN_DDC_ID = 0x4000;
constexpr int DDC_MAX_LENGTH = 195;

constexpr int ID_RUNNING = 0;
constexpr int ID_INTERFACE_TYPE = 1;
constexpr int ID_TIME_MODE = 2;
constexpr int ID_RUN_CYCLE_SWITCH = 3;
constexpr int ID_DELTA_SEC = 4;
constexpr int ID_EXEC_TIME = 5;
constexpr int ID_SIM_RUNNING = 6;
constexpr int ID_INPUT_START_ID = 7;
constexpr int ID_OUTPUT_START_ID = 8;
constexpr int ID_SLOT_TABLE_START = 7;
constexpr int ID_INTERFACE_STRING_COUNT = 8;
constexpr int ID_INTERFACE_STRING = 9;

// The folowing are placed at the 'data slot' pointed to by the slot table for a SOCKET entry
constexpr int ID_SOCKET_OUTPUT_SLOT_ID = 0; // As in the slot table: relative to 'ref_id'
constexpr int ID_SOCKET_OUTPUT_MAX_ENTRIES = 1;
constexpr int ID_SOCKET_OUTPUT_COUNT = 2;
constexpr int ID_SOCKET_INPUT_SLOT_ID = 3; // As in the slot table: relative to 'ref_id'
constexpr int ID_SOCKET_INPUT_MAX_ENTRIES = 4;
constexpr int ID_SOCKET_INPUT_COUNT = 5;