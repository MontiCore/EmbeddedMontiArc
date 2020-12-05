/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.vcg;

import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import de.rwth.montisim.commons.dynamicinterface.*;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.commons.simulation.Destroyable;
import de.rwth.montisim.commons.simulation.Destroyer;
import de.rwth.montisim.commons.simulation.Inspectable;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.hardware_emulator.vcg.VCGProperties.*;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.events.*;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.*;

/**
 * Assumes DDC communication ("DDC interface")
 */
public class VCG extends EEComponent implements Inspectable, Destroyable {

 // To generate the "basic interface" string
    public static void main(String[] args) throws Exception {
        ProgramInterface basicInterface = new ProgramInterface();
        basicInterface.name = "basic_interface";
        basicInterface.version = "1.0";
        basicInterface.ports.add(new PortInformation("true_velocity", BasicType.Q, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("true_position", BasicType.VEC2, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("true_compass", BasicType.Q, PortDirection.INPUT, false));
        VectorType trajType = new VectorType(BasicType.Q, 10);
        basicInterface.ports.add(new PortInformation("trajectory_length", BasicType.N, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("trajectory_x", trajType, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("trajectory_y", trajType, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("steering", BasicType.Q, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("gas", BasicType.Q, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("braking", BasicType.Q, PortDirection.INPUT, false));
        basicInterface.ports.add(new PortInformation("set_steering", BasicType.Q, PortDirection.OUTPUT, false));
        basicInterface.ports.add(new PortInformation("set_gas", BasicType.Q, PortDirection.OUTPUT, false));
        basicInterface.ports.add(new PortInformation("set_braking", BasicType.Q, PortDirection.OUTPUT, false));

        System.out.println(Json.toJson(basicInterface));
    }



    transient final VCGProperties properties;

    static public enum ProgInterface {
        BASIC,
        DYNAMIC
    }
    
    transient ProgramInterface program;
    transient MessageInformation msgInfos[];
    transient HashMap<String, Integer> portIdByName = new HashMap<>();
    transient Instant lastExec = null;

    transient ProgInterface interfaceMode;
    transient CommunicationInterface comm;
    Object buffer[]; // Buffer for incoming inputs / Last output values
    //Object currentValues[]; // Buffer the state of the Program Ports

    public VCG(VCGProperties properties, Destroyer destroyer) throws Exception {
        super(properties);
        this.properties = properties;

        if (properties.communication instanceof TCP) {
            comm = new TCPClient((TCP)properties.communication);
        } else throw new IllegalArgumentException("Missing case");

        program = comm.init(properties.time, properties.ref_id); // Also checks if the Program is running

        System.out.println("ProgramInterface:");
        System.out.println(program.toString());

        buffer = new Object[program.ports.size()];
        //currentValues = new Object[program.ports.size()];
        
        if (properties.time == TimeMode.REALTIME) throw new IllegalArgumentException("Unimplemented: TimeMode.REALTIME");
        destroyer.addDestroyable(this);
    }

    @Override
    protected void init() throws EEMessageTypeException {
        int i = 0;
        msgInfos = new MessageInformation[program.ports.size()];
        for (PortInformation p : program.ports) {
            if (p.direction == PortDirection.INPUT) {
                msgInfos[i] = addInput(p.name, p.type, p.allows_multiple_inputs, p.optional);
            } else {
                msgInfos[i] = addOutput(p.name, p.type);
            }
            portIdByName.put(p.name, i);
            ++i;
        }
        // Start the Execute event chain (only in measured time mode)
        if (properties.time == TimeMode.MEASURED) {
            eesystem.simulator.addEvent(new ExecuteEvent(this, eesystem.simulator.getSimulationTime()));
        }
    }

    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == MessageSendEvent.type) {
            dispatchMessage((MessageSendEvent) event);
        } else if (type == MessageReceiveEvent.type) {
            receive((MessageReceiveEvent) event);
        } else if (type == ExecuteEvent.type) {
            execute(event.getEventTime());
        } else throw new UnexpectedEventException(this.toString(), event);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        String name = msg.msgInfo.name;
        Integer i = portIdByName.get(name);
        if (i == null) {
            throw new IllegalArgumentException("Received unknown Message at VCG: " + name);
        }
        PortInformation inf = program.ports.elementAt(i);
        if (inf.direction == PortDirection.OUTPUT) {
            throw new IllegalArgumentException("Received Message at VCG Output port: " + name);
        }

        if (properties.time == TimeMode.MEASURED) {
            //System.out.println("VCG received MsgEvent: " + msg.msgInfo.name + " Payload: "+msg.message);
            buffer[i] = msg.message;
        } else if (properties.time == TimeMode.REALTIME) {
            // TODO directly send input ? or scheduled task ?
            throw new IllegalStateException("REALTIME TimeMode is Unimplemented");
        }
    }

    protected void execute(Instant time) {
        double deltaSec = 0;
        if (lastExec != null) {
            Duration dt = Duration.between(lastExec, time);
            deltaSec = Time.secondsFromDuration(dt);
        }

        Duration duration;
        try {
            duration = comm.measuredCycle(buffer, deltaSec);
        } catch (IOException e) {
            e.printStackTrace();
            throw new IllegalStateException("Exception with VCG communication");
        }

        Instant sendTime = time.plus(duration);
        int i = 0;
        for (PortInformation port : program.ports) {
            if (port.direction == PortDirection.OUTPUT) {
                if (buffer[i] != null) {
                    //System.out.println("VCG sending MsgEvent: " + msgInfos[i].name + " Payload: "+buffer[i]);
                    sendMessage(sendTime, msgInfos[i], buffer[i]);
                }
            }
            ++i;
        }

        lastExec = time;
        
        
        if (duration.compareTo(properties.cycle_duration) < 0) {
            duration = properties.cycle_duration;
        }
        Instant newExecTime = time.plus(duration);
        eesystem.simulator.addEvent(new ExecuteEvent(this, newExecTime));
    }

    protected void finalize() {
        // Set "simulation running" to false
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
            String res = p.direction == PortDirection.INPUT ? "input: " : "output: ";
            res += p.name + ": ";
            Object val = buffer[i];
            if (val == null) entries.add(res + "null");
            else {
                List<String> toStr = p.type.toString(val);
                if (toStr.size() == 0) entries.add(res + "No toString()");
                if (toStr.size() == 1) entries.add(res + toStr.get(0));
                else {
                    entries.add(res);
                    for (String s : toStr) {
                        entries.add("  "+s);
                    }
                }
            }
            ++i;
        }
        return entries;
    }

    @Override
    public void destroy() {
        comm.close();
    }

}