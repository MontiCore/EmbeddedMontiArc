/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
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
