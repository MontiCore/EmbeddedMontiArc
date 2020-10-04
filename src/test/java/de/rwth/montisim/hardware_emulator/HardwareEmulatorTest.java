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

    @Test
    public void emu_test_linux() throws Exception {
        test_software("{" +
            "\"software_name\": \"autopilots/cppautopilot\"," +
            "\"emulator_type\": \"emu\"," +
            "\"os\": \"linux\"," +
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
}
