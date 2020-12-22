/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.util.logging.Logger;

import org.junit.Test;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.hardware_emulator.CppBridge;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.HardwareTimeModel;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.HardwareEmulator.OS;

public class HardwareEmulatorTest {
    static {
        TypedHardwareEmu.registerTypedHardwareEmu();
    }

    @Test
    public void native_test() throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = "autopilots/cppautopilot";
        //config.software_name = "src/test/resources/autopilots/cppautopilot";
        config.backend = new ComputerProperties.Direct();
        config.time_model = new HardwareTimeModel();
        test_software(config);
    }

    @Test
    public void emu_test_windows() throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = "autopilots/cppautopilot";
        ComputerProperties.HardwareEmulator backend = new ComputerProperties.HardwareEmulator();
        backend.os = OS.WINDOWS;
        config.backend = backend;
        config.time_model = new HardwareTimeModel();
        test_software(config);
    }

    @Test
    public void emu_test_linux() throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = "autopilots/cppautopilot";
        ComputerProperties.HardwareEmulator backend = new ComputerProperties.HardwareEmulator();
        backend.os = OS.LINUX;
        config.backend = backend;
        config.time_model = new HardwareTimeModel();
        config.debug_flags.add("p_syscalls");
        config.debug_flags.add("p_unsupported_syscalls");
        config.debug_flags.add("p_call");
        test_software(config);
    }

    private void test_software(ComputerProperties config) throws Exception {
        CppBridge.init("{}");

        //System.out.println(Json.toFormattedJson(config));

        HardwareEmulatorBackend backend = new HardwareEmulatorBackend(config);
        Object portData[] = new Object[backend.getInterface().ports.size()];
        portData[0] = Double.valueOf(5.6);
        portData[1] = new Vec2(0,0);
        portData[2] = Double.valueOf(0);
        portData[3] = new double[]{0,1,2};
        portData[4] = new double[]{0,0,-1};

        backend.measuredCycle(portData, 0.1);

        Double steering = (Double)portData[5];
        Double gas = (Double)portData[6];
        Double brakes = (Double)portData[7];

        Logger.getGlobal().info("Result (raw strings): [gas=" + gas + ", steering=" + steering + ", brakes=" + brakes + "]");

        Logger.getGlobal().info("Result (parsed):      [gas=" + gas + ", steering=" + steering + ", brakes=" + brakes + "]");
        backend.clean();
    }
}
