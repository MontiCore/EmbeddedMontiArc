/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;


import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.*;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.simulation.commons.*;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.*;
import de.rwth.montisim.simulation.eesimulator.*;
import de.rwth.montisim.simulation.eesimulator.events.*;
import de.rwth.montisim.simulation.eesimulator.message.Message;


public class Computer extends EEComponent implements Inspectable {



    public static class SocketQueues {
        Vector<Object> in = new Vector<>(); // To the computer
        Vector<Object> out = new Vector<>(); // From the computer
    }



    transient final ComputerProperties properties;

    transient ProgramInterface program;
    transient int msgIds[];
    transient HashMap<String, Integer> portIdByName = new HashMap<>();
    transient Instant lastExec = null;
    transient boolean constantTime = false;
    transient boolean realTime = false;
    Object buffer[]; // Buffer for incoming inputs / Last output values
    // TODO handle null in arrays for serialization
    // TODO block state serialization if not supported

    transient final ComputerBackend backend;

    public Computer(ComputerProperties properties, EESystem eesystem, Destroyer destroyer, Popper popper) throws Exception {
        super(properties, eesystem);
        this.properties = properties;

        if (properties.backend instanceof TCP) {
            backend = new TCPBackend((TCP) properties.backend, properties, properties.time_model);
        } else if (properties.backend instanceof Direct) {
            if (((Direct)properties.backend).remote != null) {
                backend = new TCPBackend(((Direct)properties.backend).remote, properties, properties.time_model);
            } else {
                backend = new HardwareEmulatorBackend(properties);
            }
        } else if (properties.backend instanceof HardwareEmulator) {
            if (((HardwareEmulator)properties.backend).remote != null) {
                backend = new TCPBackend(((HardwareEmulator)properties.backend).remote, properties, properties.time_model);
            } else {
                backend = new HardwareEmulatorBackend(properties);
            }
        } else throw new IllegalArgumentException("Missing case");

        program = backend.getInterface();
        destroyer.addDestroyable(backend);
        popper.addPoppable(backend);

        this.constantTime = properties.time_model instanceof ConstantTime;
        this.realTime = properties.time_model instanceof Realtime;

        //System.out.println("ProgramInterface:");
        //System.out.println(program.toString());

        
        int i = 0;
        msgIds = new int[program.ports.size()];
        buffer = new Object[program.ports.size()];
        for (PortInformation p : program.ports) {
            msgIds[i] = addPort(p);
            portIdByName.put(p.name, i);
            if (p.port_type == PortType.SOCKET) buffer[i] = new SocketQueues();
            ++i;
        }
        if (!realTime)
            eesystem.simulator.addEvent(new ExecuteEvent(this, eesystem.simulator.getSimulationTime()));
    }

    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == MessageSendEvent.type) {
            dispatchMessage((MessageSendEvent) event);
        } else if (type == MessageReceiveEvent.type) {
            receive((MessageReceiveEvent) event);
        } else if (type == ExecuteEvent.type) {
            try {
                execute(event.getEventTime());
            } catch (Exception e) {
                e.printStackTrace();
                throw new IllegalArgumentException(e.getMessage());
            }
        } else throw new UnexpectedEventException(this.toString(), event);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        String name = msg.msgInfo.name;
        Integer i = portIdByName.get(name);
        if (i == null) throw new IllegalArgumentException("Received unknown Message at Computer: " + name);
        PortInformation inf = program.ports.elementAt(i);
        if (!inf.isInput()) throw new IllegalArgumentException("Received Message at Computer Output port: " + name);

        
        if (realTime) {
            // TODO directly send input ? or scheduled task ?
            throw new IllegalStateException("REALTIME TimeMode is Unimplemented");
        } else {
            if (inf.port_type == PortType.SOCKET) {
                SocketQueues sq = (SocketQueues)buffer[i];
                sq.in.add(msg.message);
            } else {
                buffer[i] = msg.message;
            }
        }
    }

    protected void execute(Instant time) throws Exception {
        double delta_sec = 0;
        if (lastExec != null) {
            Duration dt = Duration.between(lastExec, time);
            delta_sec = Time.secondsFromDuration(dt);
        }
        lastExec = time;

        Duration duration = backend.measuredCycle(buffer, delta_sec);
        
        if (constantTime) {
            duration = properties.cycle_duration; // Always use the same cycle duration
        } else if (duration.compareTo(properties.cycle_duration) < 0) {
            duration = properties.cycle_duration;
        }

        Instant newExecTime = time.plus(duration);
        Instant sendTime = time.plus(duration); // TODO plus 'send time' ?
        
        int i = 0;
        for (PortInformation port : program.ports) {
            if (port.isOutput()) {
                if (port.port_type == PortType.SOCKET) {
                    SocketQueues sq = (SocketQueues)buffer[i];
                    for (Object o : sq.out) {
                        sendMessage(sendTime, msgIds[i], o);
                    }
                } else {
                    if (buffer[i] != null) {
                        sendMessage(sendTime, msgIds[i], buffer[i]);
                    }
                }
            }
            ++i;
        }

        eesystem.simulator.addEvent(new ExecuteEvent(this, newExecTime));
    }

    @Override
    public String getType() {
        return "autopilot";
    }

    @Override
    public String getName() {
        return properties.name;
    }

    @Override
    public List<String> getEntries() {
        List<String> entries = new ArrayList<>();
        int i = 0;
        for (PortInformation p : program.ports) {
            if (p.port_type == PortType.DATA) {
                String res = p.direction == PortDirection.INPUT ? "input: " : "output: ";
                res += p.name + ": ";
                Object val = buffer[i];
                if (val == null) entries.add(res + "null");
                else {
                    List<String> toStr = p.data_type.toString(val);
                    if (toStr.size() == 0) entries.add(res + "No toString()");
                    if (toStr.size() == 1) entries.add(res + toStr.get(0));
                    else {
                        entries.add(res);
                        for (String s : toStr) {
                            entries.add("  "+s);
                        }
                    }
                }
            } else {
                SocketQueues sq = (SocketQueues)buffer[i];
                String res = "socket";
                if (p.isInput()) {
                    if (sq.in.size() > 0) {
                        entries.add(res + " IN [");
                        for (Object o : sq.in) {
                            List<String> toStr = p.data_type.toString(o);
                            for (String s : toStr) {
                                entries.add("  "+s);
                            }
                        }
                        res = "]";
                    } else {
                        res += " IN: []";
                    }
                }
                if (p.isOutput()) {
                    if (sq.out.size() > 0) {
                        entries.add(res + " OUT [");
                        for (Object o : sq.out) {
                            List<String> toStr = p.data_type.toString(o);
                            for (String s : toStr) {
                                entries.add("  "+s);
                            }
                        }
                        res = "]";
                    } else {
                        res += " OUT: []";
                    }
                }
                entries.add(res);
            }
            ++i;
        }
        return entries;
    }


    
 // To generate the "basic interface" string
 public static void main(String[] args) throws Exception {
    ProgramInterface basicInterface = new ProgramInterface();
    basicInterface.name = "basic_interface";
    basicInterface.version = "1.0";
    VectorType trajType = new VectorType(BasicType.Q, 10);
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("true_velocity", BasicType.Q, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("true_position", BasicType.VEC2, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("true_compass", BasicType.Q, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("trajectory_length", BasicType.N, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("trajectory_x", trajType, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("trajectory_y", trajType, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("steering", BasicType.Q, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("gas", BasicType.Q, false));
    basicInterface.ports.add(PortInformation.newRequiredInputDataPort("braking", BasicType.Q, false));
    basicInterface.ports.add(PortInformation.newRequiredOutputDataPort("set_steering", BasicType.Q));
    basicInterface.ports.add(PortInformation.newRequiredOutputDataPort("set_gas", BasicType.Q));
    basicInterface.ports.add(PortInformation.newRequiredOutputDataPort("set_braking", BasicType.Q));

    System.out.println(Json.toJson(basicInterface));
}

}