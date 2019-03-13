package simulator.integration;
import java.util.HashMap;
import java.io.Serializable;

public class HardwareEmulatorInterface {
	
	static {
        // make sure the corresponding DLL or SO is on the classpath
        // when the java app with simulation is started
        System.loadLibrary("HardwareEmulator");
    }
    
    public native boolean init(String config);
    
    public native int alloc_autopilot(String config);
    public native void free_autopilot(int id);
    
    public void update_bus(int id, HashMap<String, Serializable> inputs){
        for (HashMap.Entry<String, Serializable> entry : inputs.entrySet()){
		    add_one_input(id, entry.getKey(), entry.getValue());
        }
    }
    
    public native void start_tick(long time_delta);
    public native void end_tick();
    
    public HashMap<String, Serializable> get_outputs(int id) {
        HashMap<String, Serializable> outputs = new HashMap<String, Serializable>();
		querry_outputs(id, outputs);
		return outputs;	
    }
    
    
    public native String querry(String msg);
    public native String querry_autopilot(int id, String msg);
    
    
    private native void add_one_input(int id, String key, Serializable value);
    private native void querry_outputs(int id, HashMap<String, Serializable> opaque_hashmap);
    private void add_one_output(HashMap<String, Serializable> opaque_hashmap, String key, Serializable value) {
        opaque_hashmap.put(key, value);
    }
}
