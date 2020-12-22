/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.io.IOException;
import java.time.Duration;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.ProgramInterface;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.hardware_emulator.CppBridge;

public class HardwareEmulatorBackend implements ComputerBackend {

    int id = -2;
    ProgramInterface program;
    transient JsonWriter writer = new JsonWriter(false);
    transient JsonTraverser traverser = new JsonTraverser();

    protected HardwareEmulatorBackend(ComputerProperties properties)
            throws HardwareEmulatorException, SerializationException {
        this.id = CppBridge.allocSimulator(Json.toJson(properties));
        System.out.println("Allocated SoftwareSimulator (id: " + this.id + ")");
        String interface_description = CppBridge.getInterface(id);
        program = Json.instantiateFromJson(interface_description, ProgramInterface.class);
        if (!program.isVersionValid()) throw new IllegalArgumentException("Program '"+program.name+"' uses an outdated version of the DynamicInterface. ('"+program.version+"' instead of '"+ProgramInterface.CURRENT_VERSION+"')");
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
                if (port.direction == PortDirection.INPUT && portData[i] != null) {
                    writer.init();
                    port.data_type.toJson(writer, portData[i], null);
                    String res = writer.getString();
                    //System.out.println("  " +res);
                    CppBridge.setPort(id, i, res);
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
                if (p.direction == PortDirection.OUTPUT) {
                    String data = CppBridge.getPort(id, i);
                    traverser.init(data);
                    Object msg = p.data_type.fromJson(traverser, null);
                    portData[i] = msg;
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

}
