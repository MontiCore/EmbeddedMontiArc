/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator;

import java.util.logging.Logger;

import de.rwth.montisim.commons.utils.LibraryService;
import de.rwth.montisim.commons.utils.LibraryService.LibraryException;

import de.rwth.montisim.hardware_emulator.computer.HardwareEmulatorException;

/// Contains the STATIC functions to interact with the native C++ library.
public class CppBridge {
    static String lib_path = "";
    static boolean loaded = false;

    static {
        try {
            loadLibrary();
        } catch (LibraryException e) {
            e.printStackTrace();
            System.exit(-20);
        }
    }

    public static boolean isLoaded(){
        return loaded;
    }

    public static void loadLibrary() throws LibraryService.LibraryException {
        //Use LibraryService to get the library from resources to the working directory (if not already there)
        String system_library_name = LibraryService.getSystemLibraryName("hardware_emulator_lib");
        LibraryService.prepareLibrary(system_library_name, true);
        //Load the library
        lib_path = LibraryService.getWorkingDirectory() + system_library_name;
        System.load(lib_path);
        //System.out.println("LibPath: " + lib_path);
    }

    static public void init(String config) throws Exception{
        String version = getVersion();
        if (!HardwareEmulatorVersion.version.equals(version)) 
            System.out.println("WARNING: Wrong native HardwareEmulator library version: "+version+", expected: "+HardwareEmulatorVersion.version+". Make sure to match versions in 'software_simulator_manager.h' and recompile the C++ project.");
        initManager(config);
        loaded = true;
    }

    // Meant for logging by the C++ code
    static public void log(String msg){
        Logger.getLogger("hardware_emu").info(msg);
    }

    static public native String getVersion();

    static private native void initManager(String config) throws HardwareEmulatorException;
    static public native String queryManager(String msg) throws HardwareEmulatorException;


    static public native int allocSimulator(String config) throws HardwareEmulatorException;
    static public native void freeSimulator(int id) throws HardwareEmulatorException;

    static public native String getInterface(int id);
    static public native void setPortJson(int id, int i, String data) throws HardwareEmulatorException;
    static public native String getPortJson(int id, int i) throws HardwareEmulatorException;
    static public native void setPortBinary(int id, int i, byte[] data) throws HardwareEmulatorException;
    static public native byte[] getPortBinary(int id, int i) throws HardwareEmulatorException;

    static public native void startTimer(int id) throws HardwareEmulatorException;
    static public native long getTimerMicrosec(int id) throws HardwareEmulatorException;

    static public native void execute(int id, double delta_sec) throws HardwareEmulatorException;
}
