/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.util.logging.Logger;

import org.junit.Test;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.hardware_emulator.CppBridge;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.HardwareTimeModel;
import de.rwth.montisim.hardware_emulator.computer.ComputerProperties.HardwareEmulator.OS;

public class HardwareEmulatorTest {
    static {
        TypedHardwareEmu.registerTypedHardwareEmu();
    }

    static final String ZIGZAG_AP_PATH = "autopilots/zigzag_autopilot_lib";
    static final String EMA_AP_PATH = "autopilots/ema_autopilot_lib";

    @Test
    public void zigzag_autopilot_native_json() throws Exception {
        perform_native_json(ZIGZAG_AP_PATH);
    }
    // @Test
    // public void ema_autopilot_native_json() throws Exception {
    //     perform_native_json(EMA_AP_PATH);
    // }

    private void perform_native_json(String autopilot_path) throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = autopilot_path;
        config.backend = new ComputerProperties.Direct();
        config.time_model = new HardwareTimeModel();
        config.json_data_exchange = true;
        test_software(config);
    }

    @Test
    public void zigzag_autopilot_native_binary() throws Exception {
        perform_native_binary(ZIGZAG_AP_PATH);
    }
    // @Test
    // public void ema_autopilot_native_binary() throws Exception {
    //     perform_native_binary(EMA_AP_PATH);
    // }

    public void perform_native_binary(String autopilot_path) throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = autopilot_path;
        config.backend = new ComputerProperties.Direct();
        config.time_model = new HardwareTimeModel();
        test_software(config);
    }
    
    @Test
    public void zigzag_autopilot_emu_windows_json() throws Exception {
        perform_emu_windows_json(ZIGZAG_AP_PATH);
    }
    // @Test
    // public void ema_autopilot_emu_windows_json() throws Exception {
    //     perform_emu_windows_json(EMA_AP_PATH);
    // }

    public void perform_emu_windows_json(String autopilot_path) throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = autopilot_path;
        ComputerProperties.HardwareEmulator backend = new ComputerProperties.HardwareEmulator();
        backend.os = OS.WINDOWS;
        config.backend = backend;
        config.time_model = new HardwareTimeModel();
        config.json_data_exchange = true;
        test_software(config);
    }
    
    @Test
    public void zigzag_autopilot_emu_windows_binary() throws Exception {
        perform_emu_windows_binary(ZIGZAG_AP_PATH);
    }
    // @Test
    // public void ema_autopilot_emu_windows_binary() throws Exception {
    //     perform_emu_windows_binary(EMA_AP_PATH);
    // }

    public void perform_emu_windows_binary(String autopilot_path) throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = autopilot_path;
        ComputerProperties.HardwareEmulator backend = new ComputerProperties.HardwareEmulator();
        backend.os = OS.WINDOWS;
        config.backend = backend;
        config.time_model = new HardwareTimeModel();
        test_software(config);
    }

    
    @Test
    public void zigzag_autopilot_emu_linux_json() throws Exception {
        perform_emu_linux_json(ZIGZAG_AP_PATH);
    }
    // @Test
    // public void ema_autopilot_emu_linux_json() throws Exception {
    //     perform_emu_linux_json(EMA_AP_PATH);
    // }

    public void perform_emu_linux_json(String autopilot_path) throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = autopilot_path;
        ComputerProperties.HardwareEmulator backend = new ComputerProperties.HardwareEmulator();
        backend.os = OS.LINUX;
        config.backend = backend;
        config.time_model = new HardwareTimeModel();
        //config.debug_flags.add("p_syscalls");
        config.debug_flags.add("p_unsupported_syscalls");
        config.debug_flags.add("p_call");
        config.json_data_exchange = true;
        test_software(config);
    }
    
    @Test
    public void zigzag_autopilot_emu_linux_binary() throws Exception {
        perform_emu_linux_binary(ZIGZAG_AP_PATH);
    }
    // @Test
    // public void ema_autopilot_emu_linux_binary() throws Exception {
    //     perform_emu_linux_binary(EMA_AP_PATH);
    // }

    public void perform_emu_linux_binary(String autopilot_path) throws Exception {
        ComputerProperties config = new ComputerProperties();
        config.software_name = autopilot_path;
        ComputerProperties.HardwareEmulator backend = new ComputerProperties.HardwareEmulator();
        backend.os = OS.LINUX;
        config.backend = backend;
        config.time_model = new HardwareTimeModel();
        //config.debug_flags.add("p_syscalls");
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

        Double gas = (Double)portData[5];
        Double steering = (Double)portData[6];
        Double brakes = (Double)portData[7];

        Logger.getGlobal().info("Result: [gas=" + gas + ", steering=" + steering + ", brakes=" + brakes + "]");
        backend.clean();
    }
}
