/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator;

import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.HashMap;
import java.util.logging.Logger;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.ProgramInterface;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.events.ExecuteEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class Computer extends EEComponent {
    transient final ComputerProperties properties;

    
    int id = -2;
    transient ProgramInterface program;
    transient MessageInformation msgInfos[];
    transient HashMap<String, Integer> portIdByName = new HashMap<>();
    transient JsonWriter writer = new JsonWriter(false);
    transient JsonTraverser traverser = new JsonTraverser();
    transient Instant lastExec = null;    

    public Computer(ComputerProperties properties) throws HardwareEmulatorException, SerializationException {
        super(properties);
        this.properties = properties;
        this.id = CppBridge.allocSimulator(Json.toJson(properties));
        System.out.println("Allocated SoftwareSimulator (id: " + this.id + ")");
        String interface_description = CppBridge.getInterface(id);
        program = Json.instantiateFromJson(interface_description, ProgramInterface.class);
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
            } catch (HardwareEmulatorException | IOException | SerializationException e) {
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
        if (i == null) {
            Logger.getGlobal().warning("Received unknown Message at Computer: " + name);
            return;
        }
        PortInformation inf = program.ports.elementAt(i);
        if (inf.direction == PortDirection.OUTPUT) {
            Logger.getGlobal().warning("Received Message at Output port: " + name);
            return;
        }
        writer.init();
        try {
            msg.msgInfo.type.toJson(writer, msg.message, null);
            CppBridge.setPort(id, i, writer.getString());
        } catch (SerializationException | HardwareEmulatorException e) {
            e.printStackTrace();
            throw new IllegalArgumentException(e.getMessage());
        }
    }

    protected void execute(Instant time) throws HardwareEmulatorException, IOException, SerializationException {
        double delta_sec = 0;
        if (lastExec != null) {
            Duration dt = Duration.between(lastExec, time);
            delta_sec = Time.secondsFromDuration(dt);
        }
        // Execute software and measure time
        CppBridge.startTimer(id);
        CppBridge.execute(id, delta_sec);
        long microsecs = CppBridge.getTimerMicrosec(id);
        long nanos = (microsecs*1000)%Time.SECOND_TO_NANOSEC;
        long secs = microsecs / 1000000;
        Duration duration = Duration.ofSeconds(secs, nanos);

        // TODO compute send times and new Execute event based on Time Model
        Instant sendTime = time.plus(Duration.ofMillis(10));


        // Send output messages
        int i = 0;
        for (PortInformation p : program.ports) {
            if (p.direction == PortDirection.OUTPUT) {
                String data = CppBridge.getPort(id, i);
                traverser.init(data);
                Object msg = p.type.fromJson(traverser, null);
                sendMessage(sendTime, msgInfos[i], msg);
            }
            ++i;
        }
        lastExec = time;
        
        Instant newExecTime = time.plus(Duration.ofMillis(100));
        eesystem.simulator.addEvent(new ExecuteEvent(this, newExecTime));
    }

    protected void finalize() {
        try {
            CppBridge.freeSimulator(id);
        } catch (HardwareEmulatorException e) {
            e.printStackTrace();
        }
    }

}