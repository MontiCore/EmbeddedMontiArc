/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;

import java.io.File;
import java.io.Serializable;
import java.time.Instant;
import java.util.HashMap;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.ControllerConfig;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.ControllerConfig.EmulatorType;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.ControllerConfig.OS;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.SoftwareSimulatorConfig;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces.SoftwareSimulator;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces.SoftwareSimulatorManager;

public class HardwareEmulatorTest {
    @Test
    public void basic_test() throws Exception {
        SoftwareSimulatorManager manager = new DirectSoftwareSimulatorManager(new SoftwareSimulatorConfig().set_softwares_folder("autopilots"));
        

        String querry = "get_available_autopilots\nget_available_threads";
        String res = manager.query(querry);
        System.out.println("EmulatorManager querry response: " + res);

        String lines[] = res.split("\n");
        Assert.assertEquals("Querry result line count", 2, lines.length);
        String line1[] = lines[0].split("=");
        Assert.assertEquals("available_autopilots two values (=)", 2, line1.length);
        Assert.assertEquals("available_autopilots command", new String("available_autopilots"), line1[0]);

        String line2[] = lines[1].split("=");
        Assert.assertEquals("available_threads two values (=)", 2, line2.length);
        Assert.assertEquals("available_threads command", new String("available_threads"), line2[0]);

        SoftwareSimulator simulator = manager.newSimulator(new ControllerConfig(EmulatorType.HARDWARE_EMULATOR, "AutopilotAdapter").set_os(OS.WINDOWS));

        HashMap<String, Serializable> inputs = new HashMap<String, Serializable>();
        inputs.put("timeIncrement", 1.0);
        inputs.put("currentVelocity", 0.0);
        inputs.put("x", 0.01);
        inputs.put("y", 0.01);
        inputs.put("compass", 0.0);
        inputs.put("currentEngine", 0.0);
        inputs.put("currentSteering", 0.0);
        inputs.put("currentBrakes", 0.0);
        inputs.put("trajectory_length", 5);
        inputs.put("trajectory_x", new double[]{0.01, 0.02, 0.03, 0.04, 0.05, 0.06});
        inputs.put("trajectory_y", new double[]{0.01, 0.01, 0.02, 0.02, 0.01, 0.01});
        simulator.setInputs(inputs);
        simulator.runCycle();
        HashMap<String, Serializable> outputs = simulator.getOutputs();
        String emu_querry = simulator.query("get_avg_runtime");
        System.out.println("Emu querry: " + emu_querry);
        Assert.assertEquals("output count", outputs.size(), 3);
        for (HashMap.Entry<String, Serializable> entry : outputs.entrySet()) {
            System.out.println(entry.getKey() + " = " + entry.getValue());
        }
        simulator.free();
    }
}
