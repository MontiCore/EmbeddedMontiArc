package simulation.api;

import org.apache.log4j.Logger;
import rwth.rmi.model.server.RMIServer;
import rwth.rmi.model.server.interfaces.RMIManager;

import java.io.Serializable;
import java.rmi.NotBoundException;
import java.rmi.RemoteException;
import java.rmi.registry.LocateRegistry;
import java.rmi.registry.Registry;
import java.util.HashMap;

//import com.smartfoxserver.v2.extensions.ExtensionLogLevel;
//import rwth.server.bo.util.Logger;

public class RMIClient {

	private Logger console;
	/**
	 * RMIServer providing functionalities for specific model.
	 */
	private RMIManager manager;
	private Integer modelId;
	private Long vehicleId;
	
	/**
	 * Opens a connection to the given RMI server and provides 
	 * access to an object, implementing the VehicleModel interface.
	 * 
	 * @param host - RMI server address
	 * @param port - RMI server port
	 * @param configuration - the configuration to setup an Autopilot (autopilot, os, cpu_frequency, ...)
	 */
	public RMIClient(String host, int port, String configuration) {
		this.console = Logger.getLogger("RMIClient");

		try {
			Registry registry = LocateRegistry.getRegistry(host, port);
			this.manager = (RMIManager) registry.lookup(RMIServer.MODEL_MANAGER);
			//register model to be created by the RMIManager
			this.modelId = this.manager.alloc_autopilot(configuration);
			if (this.modelId < 0){
				System.out.println("Could not allocate autopilot and emulator: " + this.manager.query("get_error_msg"));
			}
		} catch (RemoteException e) {
			System.out.println("RMI connection problem: " + e);
		} catch (NotBoundException e) {
			System.out.println("No RMI object associated with [" + RMIServer.MODEL_MANAGER + "]: " + e);
		}
	}
	/*
		This function can eventually be replaced with the distributed start_tick()/end_tick() method for updating all
		HardwareEmulators in parallel.
		This method first calls updateBus() on all the emulators with the BUS updates.
		Then the simulation starts all the emulators with start_tick() on all the RMIModelServers.
		Then the simulation resyncronizes the emulators with end_tick() on all the RMI servers.
		Finally the individual BUS updates can be pulled with getOutputs().
	 */
	public HashMap<String, Serializable> oldExecute(HashMap<String, Serializable> inputs, double timeDelta){
		if (this.modelId >= 0) {
			try {
				return this.manager.old_execute(this.modelId, (long)(timeDelta*1000000), inputs);
			} catch(RemoteException e){
				System.out.println("RMI manager execution error: " + e);
			}
		} else {
			System.out.println("No remote Autopilot to send a bus update to.");
		}
		return new HashMap<String, Serializable>();
	}

	public void updateBus(HashMap<String, Serializable> inputs) {
		if (this.modelId >= 0) {
			try {
				this.manager.update_bus(this.modelId, inputs);
			} catch(RemoteException e){
				System.out.println("RMI manager execution error: " + e);
			}
		} else {
			System.out.println("No remote Autopilot to send a bus update to.");
		}
	}

	public HashMap<String, Serializable> getOutputs() {
		if (this.modelId >= 0) {
			try {
				return this.manager.get_outputs(this.modelId);
			} catch(RemoteException e){
				System.out.println("RMI manager execution error: " + e);
			}
		} else {
			System.out.println("No remote Autopilot to pull a bus update from.");
		}
		return new HashMap<String, Serializable>();
	}


	public String query(String query){
		if (this.modelId >= 0) {
			try {
				return this.manager.query(query);
			} catch(RemoteException e){
				System.out.println("RMI manager execution error: " + e);
			}
		} else {
			System.out.println("Query on unallocated Autopilot");
		}
		return new String();
	}
	
	/**
	 * Sets vehicle id.
	 * @param vId - unique vehicle id
	 */
	public void setVehicleId(long vId) {
		this.vehicleId = vId;
	}
	
	/**
	 * Returns the vehicle id.
	 * @return
	 */
	public Long getVehicleId() {
		return this.vehicleId;
	}
	
	/**
	 * Unregisters remote VehicleModel.
	 */
	public void removeModel() {
		if (this.modelId >= 0)
			try {
				this.manager.free_autopilot(this.modelId);
				this.modelId = -1;
			} catch (RemoteException e) {
				System.out.println("RMI manager execution error: " + e);
			}
	}
}
