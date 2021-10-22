/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.time.Duration;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.ProgramInterface;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.hardware_emulator.CppBridge;
import de.rwth.montisim.hardware_emulator.computer.Computer.SocketQueues;

public class HardwareEmulatorBackend implements ComputerBackend {

    int id = -2;
    ProgramInterface program;
    transient JsonWriter writer = new JsonWriter(false);
    transient JsonTraverser traverser = new JsonTraverser();
    ByteArrayOutputStream bout = new ByteArrayOutputStream();
    DataOutputStream out = new DataOutputStream(bout);
    boolean json_data_exchange = false;

    protected HardwareEmulatorBackend(ComputerProperties properties)
            throws HardwareEmulatorException, SerializationException {
        this.id = CppBridge.allocSimulator(Json.toJson(properties));
        System.out.println("Allocated SoftwareSimulator (id: " + this.id + ")");
        String interface_description = CppBridge.getInterface(id);
        program = Json.instantiateFromJson(interface_description, ProgramInterface.class);
        if (!program.isVersionValid()) throw new IllegalArgumentException("Program '"+program.name+"' uses an outdated version of the DynamicInterface. ('"+program.version+"' instead of '"+ProgramInterface.CURRENT_VERSION+"')");
        this.json_data_exchange = properties.json_data_exchange;
    }

    @Override
    public ProgramInterface getInterface() {
        return program;
    }

    @Override
    public Duration measuredCycle(Object[] portData, double deltaSec) throws IOException {
        try {
            // Set inputs
            int i = 0;
            //System.out.println("inputs:");
            for (PortInformation port : program.ports) {
                if (port.isInput()) {
                    if (port.port_type == PortType.SOCKET) {
                        SocketQueues sq = (SocketQueues)portData[i];
                        for (Object o : sq.in) {
                            if (json_data_exchange) {
                                writer.init();
                                port.data_type.toJson(writer, o, null);
                                CppBridge.setPortJson(id, i, writer.getString()); // Must correctly add to queue
                            } else {
                                bout.reset();
                                out.writeInt(0); // Fill 4 bytes with zero: the cpp_bridge.cpp fills it with the length of the rest after (avoids reallocating and copying the data)
                                port.data_type.toBinary(out, o);
                                CppBridge.setPortBinary(id, i, bout.toByteArray());
                            }
                        }
                        sq.in.clear();
                    } else {
                        if (portData[i] != null) {
                            if (json_data_exchange) {
                                writer.init();
                                port.data_type.toJson(writer, portData[i], null);
                                CppBridge.setPortJson(id, i, writer.getString()); // Must correctly add to queue
                            } else {
                                bout.reset();
                                port.data_type.toBinary(out, portData[i]);
                                CppBridge.setPortBinary(id, i, bout.toByteArray());
                            }
                        }
                    }
                }
                ++i;
            }

            // Measured execute
            CppBridge.startTimer(id);
            CppBridge.execute(id, deltaSec);
            long microsecs = CppBridge.getTimerMicrosec(id);
            long nanos = (microsecs*1000)%Time.SECOND_TO_NANOSEC;
            long secs = microsecs / 1000000;
            Duration duration = Duration.ofSeconds(secs, nanos);

            // Get outputs
            i = 0;
            for (PortInformation p : program.ports) {
                if (p.isOutput()) {
                    if (p.port_type == PortType.SOCKET) {
                        SocketQueues sq = (SocketQueues)portData[i];
                        sq.out.clear();
                        while (true) {
                            if (json_data_exchange) {
                                String data = CppBridge.getPortJson(id, i);
                                if (data.length() == 0) break;
                                traverser.init(data);
                                sq.out.add(p.data_type.fromJson(traverser, null));
                            } else {
                                byte[] data = CppBridge.getPortBinary(id, i);
                                if (data.length == 0) break; // No data
                                DataInputStream din = new DataInputStream(new ByteArrayInputStream(data));
                                sq.out.add(p.data_type.fromBinary(din));
                            }
                        }
                    } else {
                        if (json_data_exchange) {
                            String data = CppBridge.getPortJson(id, i);
                            traverser.init(data);
                            portData[i] = p.data_type.fromJson(traverser, null);
                        } else {
                            byte[] data = CppBridge.getPortBinary(id, i);
                            DataInputStream din = new DataInputStream(new ByteArrayInputStream(data));
                            portData[i] = p.data_type.fromBinary(din);
                        }
                    }
                }
                ++i;
            }
            
            return duration;
        } catch (HardwareEmulatorException | SerializationException e) {
            throw new IllegalStateException(e);
        }
    }
    
    
    @Override
    public void destroy() {
        if (id < 0) throw new IllegalStateException("Calling Computer.destroy() on uninitialized/already destroyed component.");
        clean();
    }
    protected void finalize() {
        clean();
    }

    void clean() {
        if (id < 0) return;
        System.out.println("Freed SoftwareSimulator (id: " + this.id + ")");
        try {
            CppBridge.freeSimulator(id);
            id = -1;
        } catch (HardwareEmulatorException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void pop() {
        throw new IllegalArgumentException("Tried to pop a vehicle using a local hardware-emulator: this is not supported.");
    }

}
