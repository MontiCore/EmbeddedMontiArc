package simulator.integration;
import java.util.HashMap;
import java.io.Serializable;

public class HardwareEmulatorInterface {
	
	static {
        // make sure the corresponding DLL is on the classpath
        // when the java app with simulation is started
        System.loadLibrary("HardwareEmulator");
    }

    private native boolean init();
    private native void add_input(String key, Serializable value);
    private native void exec();
    private native void get_outputs(HashMap<String, Serializable> opaque_hashmap);
    private void add_output(HashMap<String, Serializable> opaque_hashmap, String key, Serializable value) {
        opaque_hashmap.put(key, value);
    }
    private native String message(String msg);
    

    public HashMap<String, Serializable> execute(HashMap<String, Serializable> inputs) {
		//set inputs
		for (HashMap.Entry<String, Serializable> entry : inputs.entrySet()){
		    add_input(entry.getKey(), entry.getValue());
        }

		//execute model
		exec();
		
		//fill in outputs
		HashMap<String, Serializable> outputs = new HashMap<String, Serializable>();
		get_outputs(outputs);
		
		return outputs;	
	}
}
