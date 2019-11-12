/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;

import java.io.Serializable;
import java.time.Duration;
import java.util.HashMap;

import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.ControllerConfig;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces.SoftwareSimulator;

public class SoftwareSimulatorImpl implements SoftwareSimulator {

    int id;
    public SoftwareSimulatorImpl(ControllerConfig config) throws Exception {
        this.id = CppBridge.allocSimulator(config.get_config_string());
    }

    
    @Override
    public void setInputs(HashMap<String, Serializable> inputs) throws Exception {
        for (HashMap.Entry<String, Serializable> entry : inputs.entrySet()){
            CppBridge.addOneInput(id, entry.getKey(), entry.getValue());
        }
    }
    @Override
    public HashMap<String, Serializable> getOutputs() throws Exception {
        HashMap<String, Serializable> outputs = new HashMap<String, Serializable>();
        CppBridge.queryOutputs(id, outputs);
		return outputs;	
    }
    @Override
    public String query(String msg) throws Exception {
        return CppBridge.querySimulator(id, msg);
    }
    @Override
    public Duration runCycle() throws Exception {
        return Duration.ofNanos(CppBridge.runCycle(id));
    }
    @Override
    public void free() throws Exception {
        CppBridge.freeSimulator(id);
    }
}
