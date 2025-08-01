/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.computer;

import java.time.Duration;
import java.util.Vector;

import de.rwth.montisim.commons.simulation.Destroyer;
import de.rwth.montisim.simulation.commons.Popper;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

@Typed(ComputerProperties.TYPE)
public class ComputerProperties extends EEComponentProperties {
    public static final String TYPE = "computer";

    public String software_name;
    public Backend backend = new HardwareEmulator();
    public TimeModel time_model = new ConstantTime();
    // The maximum update speed/minimum cycle time at which the computer should run. (Will not run faster than 'cycle_duration' even if the measured time is smaller.)
    public Duration cycle_duration = Duration.ofMillis(20);
    public Vector<String> debug_flags = new Vector<>();

    public boolean json_data_exchange = false; // Set this to exchange autopilot port data with JSON instead of binary (use only to debug)

    static public interface Backend {
    }

    // Runs through the hardware emulator library, but no emulator is used:
    // the Autopilot library (DLL/SO/...) is loaded directly by the OS.
    @Typed("direct")
    static public class Direct implements Backend {
        TCP remote = null;
    }

    // The Autopilot is loaded in a Virtual Computer (can load autopilots
    // cross-platform).
    @Typed("emu")
    static public class HardwareEmulator implements Backend {
        static public enum OS {
            @JsonEntry("auto")
            AUTO, // The Emulator tries to deduce the OS from the Autopilot.
            @JsonEntry("windows")
            WINDOWS, @JsonEntry("linux")
            LINUX
        }

        public OS os = OS.AUTO;
        TCP remote = null; // If set, the computer component will connect to the hardware_emulator through the TCP protocol
    }

    @Typed("tcp")
    static public class TCP implements Backend {
        public String host;
        public int port;
        public int ref_id = 0;
        public long emu_id = 0; // Ignore, this is used by the TCPBackend connected to a remote hardware_emulator to track the EMULATOR id.
    }

    static public interface TimeModel {
    }

    @Typed("measured")
    static public class MeasuredTime implements TimeModel {
    }

    // An asynchronous mode (only TCP for now)
    @Typed("realtime")
    static public class Realtime implements TimeModel {
    }

    @Typed("constant")
    static public class ConstantTime implements TimeModel {
    }

    // Only supported by the 'HardwareEmulator' backend in 'HARDWARE_EMULATOR' mode
    @Typed("models")
    static public class HardwareTimeModel implements TimeModel {
        public long cpu_frequency = 4000000000L;
        public long memory_frequency = 2500000000L;
        public Vector<CacheProperties> caches = new Vector<>();

        public HardwareTimeModel() {
            // Setup default Cache Config
            caches.add(new CacheProperties(CacheType.I, 1, 8, 262144, 4, 4));
            caches.add(new CacheProperties(CacheType.D, 1, 8, 262144, 4, 4));
            caches.add(new CacheProperties(CacheType.SHARED, 2, 8, 2097152, 6, 6));
            caches.add(new CacheProperties(CacheType.SHARED, 3, 8, 16777216, 40, 40));
        }
    }

    static public enum CacheType {
        @JsonEntry("shared")
        SHARED, I, D
    }

    static public class CacheProperties {
        public CacheType type;
        public int level;
        public int way_count = 8;
        public long size; // Cache Size in bytes
        public long read_ticks; // Number of CPU cycles for a read action
        public long write_ticks; // Number of CPU cycles for a write action
        public long line_length = 64;

        public CacheProperties() {
        }

        public CacheProperties(CacheType type, int level, int way_count, long size, long read_ticks, long write_ticks) {
            this.type = type;
            this.level = level;
            this.way_count = way_count;
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
        public static final String PRINT_INSTRUCTION_OPERANDS = "p_instruction_operands"; // Prints operands of instructions 
        public static final String PRINT_OPERANDS_DETAILS = "p_operands_details"; // Prints detailed information for operands
        public static final String PRINT_CACHE_HIT_RATIO = "p_cache_hit_ratio"; // Prints cache access/hit/miss information
        public static final String PRINT_MEM_ACCESS = "p_mem_access"; // Prints access information of each memory access
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
    public EEComponent build(EESystem eesystem, BuildContext context) throws EEMessageTypeException {
        try {
            Destroyer destroyer = context.getObject(EESystem.COMPONENT_DESTROYER_CONTEXT_KEY);
            Popper popper = context.getObject(EESystem.COMPONENT_POPPER_CONTEXT_KEY);
            return new Computer(this, eesystem, destroyer, popper);
        } catch (Exception e) {
            e.printStackTrace();
            throw new IllegalArgumentException(e.getMessage());
        }
    }
    
}