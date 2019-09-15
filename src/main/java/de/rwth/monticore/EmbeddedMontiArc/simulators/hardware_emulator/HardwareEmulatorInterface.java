/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;

import java.io.File;
import java.util.HashMap;
import java.io.Serializable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.LibraryService;

public class HardwareEmulatorInterface implements ModelServer {
    String lib_path = "";
    public HardwareEmulatorInterface(String emu_config, String default_config) throws LibraryService.LibraryException, Exception {
        super();
        load_library();
        if (!init(emu_config, default_config)){
            String error_msg = query("get_error_msg");
            throw new Exception("Error initializing the HardwareEmulator: " + error_msg);
        }
    }

    public void load_library() throws LibraryService.LibraryException {
        //Use LibraryService to get the library from resources to the working directory (if not already there)
        String system_library_name = LibraryService.getSystemLibraryName("HardwareEmulator");
        LibraryService.prepareLibrary(system_library_name);
        //Load the library
        lib_path = LibraryService.getWorkingDirectory() + system_library_name;
        System.load(lib_path);
        //System.out.println("LibPath: " + lib_path);
    }

    public File get_library(){
        return new File(lib_path);
    }

    private native boolean init(String config, String default_config);
    @Override
    public native int alloc_autopilot(String config);
    @Override
    public native void free_autopilot(int id);
    @Override
    public void update_bus(int id, HashMap<String, Serializable> inputs){
        for (HashMap.Entry<String, Serializable> entry : inputs.entrySet()){
		    add_one_input(id, entry.getKey(), entry.getValue());
        }
    }

    @Override
    public HashMap<String, Serializable> old_execute(int id, long time_delta, HashMap<String, Serializable> inputs){
        for (HashMap.Entry<String, Serializable> entry : inputs.entrySet()){
            add_one_input(id, entry.getKey(), entry.getValue());
        }
        execute_one(id, time_delta);
        HashMap<String, Serializable> outputs = new HashMap<String, Serializable>();
        query_outputs(id, outputs);
        return outputs;
    }
    private native void execute_one(int id, long time_delta);


    @Override
    public native void start_tick(long time_delta);
    @Override
    public native void end_tick();

    @Override
    public HashMap<String, Serializable> get_outputs(int id) {
        HashMap<String, Serializable> outputs = new HashMap<String, Serializable>();
		query_outputs(id, outputs);
		return outputs;	
    }

    @Override
    public native String query(String msg);
    @Override
    public native String query_autopilot(int id, String msg);
    
    
    private native void add_one_input(int id, String key, Serializable value);
    private native void query_outputs(int id, HashMap<String, Serializable> opaque_hashmap);
    private void add_one_output(HashMap<String, Serializable> opaque_hashmap, String key, Serializable value) {
        opaque_hashmap.put(key, value);
    }
}
