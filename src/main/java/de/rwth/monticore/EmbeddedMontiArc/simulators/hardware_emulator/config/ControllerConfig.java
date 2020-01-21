/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config;

import java.io.Serializable;
import java.time.Duration;

public class ControllerConfig implements Serializable {
    private static final long serialVersionUID = 646999434800913765L;

    static public enum EmulatorType {
        DIRECT("direct"), //No Emulator at all: the Autopilot library (DLL/SO/...) is loaded directly by the OS.
        HARDWARE_EMULATOR("emu"); //The Autopilot is loaded in a Virtual Computer (can load autopilots cross-platform).
        String value;
        EmulatorType(String value) {
            this.value = value;
        }
        public String get_config_entry(){
            return "mode=" + value + "\n";
        }
    }

    static public enum TimeModel {
        INSTANT("instant"), //The execution of the virtual is instantaneous in the virtual world
        CONSTANT("constant"), //The execution of one Cycle uses a constant virtual time.
        TIMEMODELS("models"); //Time models are used in the virtual computer to estimate the execution time of the Autopilot.
        String value;
        TimeModel(String value) {
            this.value = value;
        }
        public String get_config_entry(){
            return "time_model=" + value + "\n";
        }
    }

    static public enum OS {
        AUTO("auto"), //The Emulator tries to deduce the OS from the Autopilot.
        WINDOWS("windows"), 
        LINUX("linux");
        String value;
        OS(String value) {
            this.value = value;
        }
        public String get_config_entry(){
            return "os=" + value + "\n";
        }
    }


    static public enum DebugFlag {
        MEMORY("mem"), //Prints read/write actions on the memory
        REGISTERS("regs"), //Prints the state of the registers after every instruction
        REGISTER_UPDATES("reg_update"), //Only prints changed registers
        SYSTEM_CALLS("syscalls"), //Prints calls from the emulated program to the OS emulation
        UNSUPPORTED_SYSTEM_CALLS("unsupported_syscalls"), //Prints calls to OS functions that are not supported
        CODE("code"), //Prints the instructions executed by the program
        FUNCTION_CALLS("call"), //Prints calls from the outside to the emulated program
        TIME("time"); //Prints the evaluated time
        String value;
        DebugFlag(String value) {
            this.value = value;
        }
        public String get_name(){
            return value;
        }
    }
    
    public static class CacheOption implements Serializable {
        private static final long serialVersionUID = 3500766901354278880L;
        private long size; // Cache Size in bytes
        private long read_ticks; // Number of CPU cycles for a read action
        private long write_ticks; //Number of CPU cycles for a write action

        public CacheOption(long size, long read_ticks, long write_ticks){
            this.size = size;
            this.read_ticks = read_ticks;
            this.write_ticks = write_ticks;
        }

        public String get_config_string(){
            return "=" + size + "," + read_ticks + "," + write_ticks + "\n";
        }

        public long getSize(){
            return size;
        }
        public long getReadSpeedCpuCycles(){
            return read_ticks;
        }
        public long getWriteSpeedCpuCycles(){
            return write_ticks;
        }
    }

    private String autopilot;
    private EmulatorType emulator_type;
    private TimeModel time_model= TimeModel.INSTANT;
    private Duration execution_time;
    private OS os = OS.AUTO;
    private boolean test_real = false;
    private boolean test_real_required = false;
    private long cpu_frequency_hertz = -1;
    private long memory_frequency_hertz = -1;
    private CacheOption IL1_cache = null;
    private CacheOption DL1_cache = null;
    private CacheOption L2_cache = null;
    private CacheOption L3_cache = null;
    private DebugFlag[] debug_flags = null;


    public ControllerConfig(EmulatorType emulator_type, String autopilot_name) {
        this.emulator_type = emulator_type;
        this.autopilot = autopilot_name;
    }

    //Time Model config
    public ControllerConfig set_timemodel_instant() {
        this.time_model = TimeModel.INSTANT;
        return this;
    }

    public ControllerConfig set_timemodel_constant(Duration execution_time) {
        this.time_model = TimeModel.CONSTANT;
        this.execution_time = execution_time;
        return this;
    }

    public ControllerConfig set_timemodel_timemodel() {
        if (emulator_type == EmulatorType.DIRECT){
            throw new IllegalArgumentException("The DIRECT mode of running autopilots does not support Time Models.");
        }
        this.time_model = TimeModel.TIMEMODELS;
        return this;
    }

    //Config entries for HARDWARE_EMULATOR type:

    //Sets the OS used for the Emulation (optional: will try to deduce from Autopilot library type)
    public ControllerConfig set_os(OS os) {
        if (emulator_type != EmulatorType.HARDWARE_EMULATOR){
            throw new IllegalArgumentException("Can only choose OS in EMULATOR mode.");
        }
        this.os = os;
        return this;
    }

    public ControllerConfig set_debug_options(DebugFlag... flags) {
        if (emulator_type != EmulatorType.HARDWARE_EMULATOR){
            throw new IllegalArgumentException("Can only have debug options in EMULATOR mode.");
        }
        this.debug_flags = flags;
        return this;
    }

    //test_real: Loads the Autopilot as DIRECT library in parallel and verifies that the outputs of the emulated version match the real one.
    public ControllerConfig set_test_real(boolean required) {
        if (emulator_type != EmulatorType.HARDWARE_EMULATOR){
            throw new IllegalArgumentException("Can only Test Real in EMULATOR mode.");
        }
        this.test_real = true;
        this.test_real_required = required;
        return this;
    }

    // FOR TIMEMODELS:

    //cpu_frequency: Frequency of the virtual CPU (in Hertz).
    public ControllerConfig set_cpu_frequency(long cpu_frequency_hertz) {
        if (time_model != TimeModel.TIMEMODELS){
            throw new IllegalArgumentException("Can only set cpu frequency with TIME MODELS.");
        }
        this.cpu_frequency_hertz = cpu_frequency_hertz;
        return this;
    }

    //memory_frequency: Frequency of the virtual RAM (in Hertz).
    public ControllerConfig set_memory_frequency(long memory_frequency_hertz) {
        if (time_model != TimeModel.TIMEMODELS){
            throw new IllegalArgumentException("Can only set memory frequency with TIME MODELS.");
        }
        this.memory_frequency_hertz = memory_frequency_hertz;
        return this;
    }


    //cache_
    public ControllerConfig set_cache_DL1(CacheOption cache) {
        if (time_model != TimeModel.TIMEMODELS){
            throw new IllegalArgumentException("Can only set cache options with TIME MODELS.");
        }
        this.DL1_cache = cache;
        return this;
    }
    public ControllerConfig set_cache_IL1(CacheOption cache) {
        if (time_model != TimeModel.TIMEMODELS){
            throw new IllegalArgumentException("Can only set cache options with TIME MODELS.");
        }
        this.IL1_cache = cache;
        return this;
    }
    public ControllerConfig set_cache_L2(CacheOption cache) {
        if (time_model != TimeModel.TIMEMODELS){
            throw new IllegalArgumentException("Can only set cache options with TIME MODELS.");
        }
        this.L2_cache = cache;
        return this;
    }
    public ControllerConfig set_cache_L3(CacheOption cache) {
        if (time_model != TimeModel.TIMEMODELS){
            throw new IllegalArgumentException("Can only set cache options with TIME MODELS.");
        }
        this.L3_cache = cache;
        return this;
    }

    public ControllerConfig set_default_timemodel() {
        set_timemodel_timemodel();
        set_cpu_frequency(4000000000L);
        set_memory_frequency(2500000000L);
        set_cache_DL1(new CacheOption(262144,4,4));
        set_cache_IL1(new CacheOption(262144,4,4));
        set_cache_L2(new CacheOption(2097152,6,6));
        set_cache_L3(new CacheOption(12582912,40,40));
        return this;
    }

    //Creates the string that is passed to the emulator when initializing a new autopilot.
    public String get_config_string(){
        String res = "software="+autopilot+"\n";
        res += emulator_type.get_config_entry();
        res += time_model.get_config_entry();
        if (time_model == TimeModel.CONSTANT){
            res += "const_execution_time=" + (execution_time.toNanos()/1000) + "\n";
        }
        if (emulator_type == EmulatorType.HARDWARE_EMULATOR){
            res += os.get_config_entry();
            if (test_real){
                res += "test_real";
                if (test_real_required){
                    res += "=required\n";
                } else{
                    res += "=try\n";
                }
            }
            if (debug_flags != null){
                res += "debug=";
                for (DebugFlag flag : debug_flags){
                    res += flag.get_name() + ",";
                }
                res += "\n";
            }
        }
        if (time_model == TimeModel.TIMEMODELS){
            if (cpu_frequency_hertz < 0){
                throw new IllegalArgumentException("Did not set CPU FREQUENCY with TIME MODELS.");
            }
            if (memory_frequency_hertz < 0){
                throw new IllegalArgumentException("Did not set MEMORY FREQUENCY with TIME MODELS.");
            }
            res += "cpu_frequency="+cpu_frequency_hertz + "\n";
            res += "memory_frequency="+memory_frequency_hertz + "\n";
            if (IL1_cache != null) res += "cache_IL1" + IL1_cache.get_config_string();
            if (DL1_cache != null) res += "cache_DL1" + DL1_cache.get_config_string();
            if (L2_cache != null) res += "cache_L2" + L2_cache.get_config_string();
            if (L3_cache != null) res += "cache_L3" + L3_cache.get_config_string();
        }
        
        return res;
    }

    public String getSoftwareName(){
        return autopilot;
    }
    public EmulatorType getSoftwareSimulatorType(){
        return emulator_type;
    }
    public TimeModel getTimeModel(){
        return time_model;
    }
    public Duration getExecutionTime(){
        return execution_time;
    }
    public OS getOS(){
        return os;
    }
    public boolean getTestReal(){
        return test_real;
    }
    public boolean getTestRealRequired(){
        return test_real_required;
    }
    public long getCpuFrequencyHertz(){
        return cpu_frequency_hertz;
    }
    public long getRAMFrequencyHertz(){
        return memory_frequency_hertz;
    }
    public CacheOption getCacheIL1Options(){
        return IL1_cache;
    }
    public CacheOption getCacheDL1Options(){
        return DL1_cache;
    }
    public CacheOption getCacheL2Options(){
        return L2_cache;
    }
    public CacheOption getCacheL3Options(){
        return L3_cache;
    }
}