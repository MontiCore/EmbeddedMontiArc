<!-- (c) https://github.com/MontiCore/monticore -->

# Bus

This part of the project includes the classes for the simulation of diffrent buses.

# An exaple of a bussystem

This
picture ![picture](/docs/busexample.jpg) [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/blob/ee-infrastructre/bus/docs/BusExample.jpg)
shows an example bus system with the buses A,B,C , the actuator 1-6 and the bridges ab and bc. The times, shown in red,
describes the delay by using the specific component, the arrows describe which actuator sends or want to recieve the
message Mes1 (green), send by actuator one, and Mess2 (blue), send by actuator five.

# How does the bus is used

This sequence
diagram ![diagram](/docs/sequencediagram.jpg) [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/blob/ee-infrastructre/bus/docs/SequenceDiagram.jpg)
describes the function calls to transmit Mes2 from actuator five to actuator two. Actuator act5 creates an event of type
busMessage and register it at the simulator by using the function `addEvent()` of the simulator. In regular time steps,
the simulator got the function call `simualteFor(time)`. The simulator will start to process all registered events with
an event time lower than the argument `time`. For processing, the simulator uses the function `processEvent()` of the
target component saved in the busMessage. This function will process the event individually, depending on the target
component type.
Bridges create new events with a new target bus.
Actuators use the information send in the busMessage.
Buses create new keepAlive events for themselves with the event time when the incoming busMessage will be transmitted
completely. KeepAlive events will be processed by the bus by sending the original busMessage to all next receivers.

# How does the event queue of the simulator works

This
diagram ![diagram](/docs/eventqueue.jpg) [here](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/blob/ee-infrastructre/bus/docs/EventQueue.jpg)
describes how the event queue of the simulator develops over a time period after the actuators one and five have added
their messages and the simulator have gotten the `simulateFor()` call. The arrows show which event is created based on
the actual event. The red dashed lines mark the events that are not processed this time, depending on the time of the
events.
For example, at the beginning, the fist entry of the event queue ist the message 1 with target bus A at the time 0. The
simulator will use the function `processEvent(Mes1)` of the bus A. In the following bus A will create a KeepAlive event
with target bus A and time 2. The time of this keepAlive event is the time when the bus would have transmitted the whole
message Mes1 to the next components. In the next step, the simulator will choose the first event again. It will
call `processEvent(KeepAlive)` of the bus and the bus will create two new busMessages to each component, wich want to
receive the message 1.
