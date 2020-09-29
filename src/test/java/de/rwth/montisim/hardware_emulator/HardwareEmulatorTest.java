/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator;

import java.util.logging.Logger;

import org.junit.Test;

import de.rwth.montisim.hardware_emulator.config.SoftwareSimulatorConfig;

public class HardwareEmulatorTest {

    @Test
    public void native_test() throws Exception {
        test_software("{"+
            "\"software_name\": \"autopilots/cppautopilot\","+
            "\"emulator_type\": \"direct\"" +
        "}");
    }

    @Test
    public void emu_test_windows() throws Exception {
        test_software("{" +
            "\"software_name\": \"autopilots/cppautopilot\"," +
            "\"emulator_type\": \"emu\"," +
            "\"os\": \"windows\"," +
            "\"time_model\": {" +
                "\"type\": \"models\"," +
                "\"cpu_frequency\": 4000000000," +
                "\"memory_frequency\": 2500000000," +
                "\"caches\": [" +
                    "{\"type\": \"I\", \"level\": 1, \"size\": 262144, \"read_ticks\": 4, \"write_ticks\": 4}," +
                    "{\"type\": \"D\", \"level\": 1, \"size\": 262144, \"read_ticks\": 4, \"write_ticks\": 4}," +
                    "{\"type\": \"shared\", \"level\": 2, \"size\": 2097152, \"read_ticks\": 6, \"write_ticks\": 6}," +
                    "{\"type\": \"shared\", \"level\": 3, \"size\": 12582912, \"read_ticks\": 40, \"write_ticks\": 40}" +
                "]" +
            "}" +
        "}");
    }

    private void test_software(String config) throws Exception {
        CppBridge.init("{}");
        int id = CppBridge.allocSimulator(config);
        CppBridge.setPort(id, 0, "5.6");
        CppBridge.setPort(id, 1, "[0,0]");
        CppBridge.setPort(id, 2, "0");
        CppBridge.setPort(id, 3, "[0,1,2]");
        CppBridge.setPort(id, 4, "[0,0,-1]");

        CppBridge.execute(id, 0.1);

        String steering = CppBridge.getPort(id, 5);
        String gas = CppBridge.getPort(id, 6);
        String brakes = CppBridge.getPort(id, 7);

        Logger.getGlobal().info("Result (raw strings): [gas=" + gas + ", steering=" + steering + ", brakes=" + brakes + "]");

        Logger.getGlobal().info("Result (parsed):      [gas=" + Double.parseDouble(gas) + ", steering=" + Double.parseDouble(steering) + ", brakes=" + Double.parseDouble(brakes) + "]");
    }
    // @Test
    // public void basic_test() throws Exception {
    //     SoftwareSimulatorManager manager = new DirectSoftwareSimulatorManager(new SoftwareSimulatorConfig().set_softwares_folder("autopilots"));
        

    //     String querry = "get_available_autopilots\nget_available_threads";
    //     String res = manager.query(querry);
    //     System.out.println("EmulatorManager querry response: " + res);

    //     String lines[] = res.split("\n");
    //     Assert.assertEquals("Querry result line count", 2, lines.length);
    //     String line1[] = lines[0].split("=");
    //     Assert.assertEquals("available_autopilots two values (=)", 2, line1.length);
    //     Assert.assertEquals("available_autopilots command", new String("available_autopilots"), line1[0]);

    //     String line2[] = lines[1].split("=");
    //     Assert.assertEquals("available_threads two values (=)", 2, line2.length);
    //     Assert.assertEquals("available_threads command", new String("available_threads"), line2[0]);

    //     SoftwareSimulator simulator = manager.newSimulator(new ControllerConfig(EmulatorType.HARDWARE_EMULATOR, "AutopilotAdapter").set_os(OS.WINDOWS));

    //     HashMap<String, Serializable> inputs = new HashMap<String, Serializable>();
    //     inputs.put("timeIncrement", 1.0);
    //     inputs.put("currentVelocity", 0.0);
    //     inputs.put("x", 0.01);
    //     inputs.put("y", 0.01);
    //     inputs.put("compass", 0.0);
    //     inputs.put("currentEngine", 0.0);
    //     inputs.put("currentSteering", 0.0);
    //     inputs.put("currentBrakes", 0.0);
    //     inputs.put("trajectory_length", 5);
    //     inputs.put("trajectory_x", new double[]{0.01, 0.02, 0.03, 0.04, 0.05, 0.06});
    //     inputs.put("trajectory_y", new double[]{0.01, 0.01, 0.02, 0.02, 0.01, 0.01});
    //     simulator.setInputs(inputs);
    //     simulator.runCycle();
    //     HashMap<String, Serializable> outputs = simulator.getOutputs();
    //     String emu_querry = simulator.query("get_avg_runtime");
    //     System.out.println("Emu querry: " + emu_querry);
    //     Assert.assertEquals("output count", outputs.size(), 3);
    //     for (HashMap.Entry<String, Serializable> entry : outputs.entrySet()) {
    //         System.out.println(entry.getKey() + " = " + entry.getValue());
    //     }
    //     simulator.free();
    // }
}
