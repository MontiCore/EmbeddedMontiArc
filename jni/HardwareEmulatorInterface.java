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
package simulator.integration;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.io.Serializable;

public class HardwareEmulatorInterface implements RMIManager {
    public HardwareEmulatorInterface(String libraryPath) {
        super();
        String emu = System.getProperty("user.dir") + "/HardwareEmulator";
        String OS = System.getProperty("os.name").toLowerCase();
        if (OS.contains("win")){
            System.load(emu+".dll");
        } else if (OS.contains("nix")|| OS.contains("nux")|| OS.contains("aix")){
            System.load(emu+".so");
        } else {
            System.out.println("Could not deduce Operating System for loading the HardwareEmulator.");
        }
        try {// Load AutopilotAdapter.dll from libraryPath
            File b = new File(libraryPath);
            String absolute = b.getCanonicalPath();
            init("autopilots_folder=" + absolute, "");
        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("Could not initiate the Hardware Emulator");
            init("", "");
        }

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
