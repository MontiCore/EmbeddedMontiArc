# Network Update Overview

New [basic-simulator](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator) update.

## EEComponents

There is some changes in how to describe the connectivity of the EEComponents in the Vehicle.
**Components** and **buses** alike can now have a `connected_to` entry in their properties.
This allows to list all the components connected to a bus directly in the bus properties instead of specifying the associated bus at every component.

All example scenarios in the basic-simulator have been updated accordingly.
**Make sure to update your own scenarios**.

## VCG/Computer

Since they were very similar, the `Computer` and `VCG` components have been merged to only the `Computer` component.
To use the **VCG**, just specify a "TCP backend" in the computer component. (See the [vcg.json](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator/-/blob/master/install/scenarios/vcg.json) scenario as example.)

**Make sure to update your own scenarios**.

## ProgramInterface/PortInformation

There are some syntactic changes to the `PortInformation`.
Changes to the ProgramInterface json: `type` field now called `data_type`. **Make sure to update this in your autopilots**.

Ports can now be `DATA` ports (as before) or `SOCKETS`, where messages of the same name can be *sent* and *received*.
The new `port_type` field can be `DATA` or `SOCKET` and is *optional* (defaults to `DATA`).

Ports can also now have **tags**, currently used for networking: give the port the `network` tag.

The [sample_autopilot](../build_environment/sample_autopilot/src/program_adapter.cpp#L313) has been updated accordingly.

## New SimpleNetwork

The `SimpleNetwork` allows to send messages to neighboring cars, without modelling complex transmission behavior (similar to how the `ConstantBus` transmits messages).

To use it, it must be added as a **Simulation module** in the simulation configuration. Here are the [SimpleNetworkProperties](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/blob/master/eecomponents/src/main/java/de/rwth/montisim/simulation/eecomponents/simple_network/SimpleNetworkProperties.java).

For a vehicle to be able to communicate with the `SimpleNetwork`, it must have a `SimpleCommunicationGateway` (with its [SCGProperties](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/blob/master/eecomponents/src/main/java/de/rwth/montisim/simulation/eecomponents/simple_network/SCGProperties.java)).

See the new [network_test.json](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator/-/blob/master/install/scenarios/network_test.json) scenario as example for the configuration of the `SimpleNetork` and according `SimpleCommunicationGateway` components.

## Network Messages

There are the requirements for an autopilot to send Network messages:

- The `port_type` must be `SOCKET`.
- The port must have the `network` tag to be discovered by the `SimpleCommunicationGateway`.
- The `data_type` must be the new `SimplePacketType` with according payload type.

The [network_example](../build_environment/network_example/src) in this repository is the `sample_autopilot` with two `SOCKET` ports that randomly broadcasts and responds on these ports. The [network_test.json](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/basic-simulator/-/blob/master/install/scenarios/network_test.json) scenario (basic-simulator) can be run with two instances of the `network_example` started on the ports specified in the scenario. It then logs the message exchange.

There is currently no formal documentation for the different `DataTypes`, but [here are the sources](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/commons/-/tree/master/src/main/java/de/rwth/montisim/commons/dynamicinterface). Note that not everything is currently implemented for all data types.

## EMA

EMA models can now communicate across vehicles through the SimpleNetwork by using suffixes on the ports.

The `Autopilot.emam` file in the `test_emam_ap` and `test_emadl_ap` directories now contains a few (unconnected) example "socket"-ports:

```ema
in Q test1_socket_in [2],
out Q test1_socket_out [2],
in Z test2_socket_bc_in [2],
out Z test2_socket_bc_out,
```

This example allows 3 vehicles to communicate. The ports must have the `_socket_*` suffix to be recognized as communication sockets. Note that the output broadcast port is not a port-array.

An associated example scenario has been pushed to the basic-simulator (just make sure to enable the `simple_network` module and have sufficient communication range).
