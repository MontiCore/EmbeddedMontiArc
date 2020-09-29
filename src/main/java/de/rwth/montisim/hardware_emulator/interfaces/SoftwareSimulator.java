package de.rwth.montisim.hardware_emulator.interfaces;

import java.io.Serializable;
import java.rmi.Remote;
import java.time.Duration;
import java.util.HashMap;

public interface SoftwareSimulator extends Remote {

    //Send and get the Bus Messages (=port values)
    public void setInputs(HashMap<String, Serializable> inputs) throws Exception;    
    public HashMap<String, Serializable> getOutputs() throws Exception;

    /// Requests information to the SoftwareSimulator. The available queries can be seen in software_simulator.cpp/hardware_emulator.cpp in the query function
    public String query(String msg) throws Exception;
    /// Runs a cycle of the controller: read the inputs from the buffer, run the program, write the outputs to the buffer. returns the execution time for this cycle.
    public Duration runCycle() throws Exception;
    /// IMPORTANT: frees the C resources associated with this Simulator. Won't happen automatically.
    public void free() throws Exception;
}