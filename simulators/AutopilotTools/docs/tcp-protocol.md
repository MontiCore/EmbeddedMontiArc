# TCP communication protocol

TODO put in the simulation wiki

The TCP communication is used between the Simulator and the program on the VCG directly in *Server Mode*,

## Communication sequence

- Connect
- Simulation sends "PACKET_INIT" with "TimeMode" config string.
- Server responds with "DynamicInterface" string. (PACKET_INTERFACE)

### Simulation cycles for *measured mode*

- Sim sends input packets. (PACKET_INPUT)
- Sim sends run_cycle request. (PACKET_RUN_CYCLE)
- Server runs program.
- Server sends output packets. (PACKET_OUTPUT)
- Server sends exec time -> Simulation continues. (PACKET_TIME)

### Simulation cycles for *realtime mode*

Sim sends "running" packet when running (with "1" payload)

Sim:

- On Input Event: Send packet
- Async receive output packets -> create events with current time

Server:

- Read input packets
- Run cycle
- Write output packets (with max per second limit?)

## Packet format

- 1 byte: packet ID
- 1 short (uint16_t): packet payload length
- *length* bytes: payload

See [network.h file](../externals/shared_cpp/network.h) as example.

## Packet Payloads

See [tcp_protocol.h file](../externals/shared_cpp/tcp_protocol.h) for the details.
And [The Java Dynamic Interface files](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/commons/-/tree/master/src/main/java/de/rwth/montisim/commons/dynamicinterface) or the [sample_autopilot/program_adapter.cpp set_input() / send_output() functions](../sample_autopilot/src/program_adapter.cpp) for the transformation of Input/Output messages to packet payload.
