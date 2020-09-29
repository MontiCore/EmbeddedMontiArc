/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator;

import java.time.Duration;
import java.util.Vector;

import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

@Typed(ComputerProperties.TYPE)
public class ComputerProperties extends BusUserProperties {
    public static final String TYPE = "hardware_emulator";

    static {
        try {
            Json.registerType(InstantTime.class);
            Json.registerType(ConstantTime.class);
            Json.registerType(HardwareTimeModel.class);
        } catch (SerializationException e) {
            e.printStackTrace();
            System.exit(-1);
        }
    }

    public String software_name;
    public EmulatorType emulator_type = EmulatorType.HARDWARE_EMULATOR;
    public OS os = OS.AUTO;
    public TimeModel time_model = new ConstantTime();
    public Vector<String> debug_flags = new Vector<>();

    static public enum EmulatorType {
        @JsonEntry("direct")
        DIRECT, // No Emulator at all: the Autopilot library (DLL/SO/...) is loaded directly by
                // the OS.
        @JsonEntry("emu")
        HARDWARE_EMULATOR // The Autopilot is loaded in a Virtual Computer (can load autopilots
                          // cross-platform).
    }

    static public enum OS {
        @JsonEntry("auto")
        AUTO, // The Emulator tries to deduce the OS from the Autopilot.
        @JsonEntry("windows")
        WINDOWS, @JsonEntry("linux")
        LINUX
    }

    static public interface TimeModel {
    }

    @Typed("instant")
    static public class InstantTime implements TimeModel {
    }

    @Typed("constant")
    static public class ConstantTime implements TimeModel {
        public Duration execution_time = Duration.ofMillis(100);
    }

    @Typed("models")
    static public class HardwareTimeModel implements TimeModel {
        public long cpu_frequency = 4000000000L;
        public long memory_frequency = 2500000000L;
        public Vector<CacheProperties> caches = new Vector<>();

        public HardwareTimeModel() {
            // Setup default Cache Config
            caches.add(new CacheProperties(CacheType.I, 1, 262144, 4, 4));
            caches.add(new CacheProperties(CacheType.D, 1, 262144, 4, 4));
            caches.add(new CacheProperties(CacheType.SHARED, 2, 2097152, 6, 6));
            caches.add(new CacheProperties(CacheType.SHARED, 3, 12582912, 40, 40));
        }
    }

    static public enum CacheType {
        @JsonEntry("shared")
        SHARED, I, D
    }

    static public class CacheProperties {
        public CacheType type;
        public int level;
        public long size; // Cache Size in bytes
        public long read_ticks; // Number of CPU cycles for a read action
        public long write_ticks; // Number of CPU cycles for a write action
        public long line_length = 64;

        public CacheProperties() {
        }

        public CacheProperties(CacheType type, int level, long size, long read_ticks, long write_ticks) {
            this.type = type;
            this.level = level;
            this.size = size;
            this.read_ticks = read_ticks;
            this.write_ticks = write_ticks;
        }
    }

    public static class DebugFlags {
        public static final String PRINT_MEMORY_OPS = "p_mem"; // Prints read/write actions on the memory
        public static final String PRINT_REGISTER_OPS = "p_regs"; // Prints the state of the registers after every
                                                                  // instruction
        public static final String PRINT_REGISTER_UPDATES = "p_reg_update"; // Only prints changed registers
        public static final String PRINT_SYSTEM_CALLS = "p_syscalls"; // Prints calls from the emulated program to the
                                                                      // OS emulation
        public static final String PRINT_UNSUPPORTED_SYSTEM_CALLS = "p_unsupported_syscalls"; // Prints calls to OS
                                                                                              // functions that are not
                                                                                              // supported
        public static final String PRINT_CODE_OPS = "p_code"; // Prints the instructions executed by the program
        public static final String PRINT_FUNCTION_CALLS = "p_call"; // Prints calls from the outside to the emulated
                                                                    // program
        public static final String PRINT_TIME = "p_time"; // Prints the evaluated time
        public static final String TEST_REAL = "test_real"; // Load the Autopilot as DIRECT library in parallel and
                                                            // verify that the outputs of the emulated version match the
                                                            // real one.
        public static final String TEST_REAL_REQUIRED = "test_real_required";
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.COMPUTER;
    }

    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    public EEEventProcessor build(ComponentBuildContext context) {
        try {
            return new Computer(this);
        } catch (HardwareEmulatorException | SerializationException e) {
            e.printStackTrace();
            throw new IllegalArgumentException(e.getMessage());
        }
    }
    
}