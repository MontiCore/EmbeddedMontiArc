/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;

import java.io.Serializable;
import java.util.HashMap;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.LibraryService;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.LibraryService.LibraryException;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.SoftwareSimulatorConfig;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.HardwareEmulatorVersion;

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
        String system_library_name = LibraryService.getSystemLibraryName("HardwareEmulator");
        LibraryService.prepareLibrary(system_library_name, true);
        //Load the library
        lib_path = LibraryService.getWorkingDirectory() + system_library_name;
        System.load(lib_path);
        //System.out.println("LibPath: " + lib_path);
    }

    static public void init(SoftwareSimulatorConfig manager_config) throws Exception{
        String version = getVersion();
        if (!HardwareEmulatorVersion.version.equals(version)) 
            throw new Exception("Wrong native HardwareEmulator library version: "+version+", expected: "+HardwareEmulatorVersion.version);
        initManager(manager_config.get_config_string());
        loaded = true;
    }

    static public native String getVersion();

    static private native void initManager(String config);

    static public native int allocSimulator(String config) throws Exception;
    static public native void freeSimulator(int id);


    static public native String queryManager(String msg);

    static public native long runCycle(int id) throws Exception; //Returns execution duration in nanoseconds
    static public native String querySimulator(int id, String msg);
    static public native void addOneInput(int id, String key, Serializable value) throws Exception;
    static public native void queryOutputs(int id, HashMap<String, Serializable> opaque_hashmap) throws Exception;

    //Called by the simulator library
    static private void addOneOutput(HashMap<String, Serializable> opaque_hashmap, String key, Serializable value) {
        opaque_hashmap.put(key, value);
    }
}
