/**
 * (c) https://github.com/MontiCore/monticore
 */
// package de.rwth.montisim.hardware_emulator;

// import java.rmi.RemoteException;
// import java.rmi.server.UnicastRemoteObject;

// import de.rwth.montisim.hardware_emulator.config.ControllerConfig;
// import de.rwth.montisim.hardware_emulator.config.SoftwareSimulatorConfig;
// import de.rwth.montisim.hardware_emulator.interfaces.SoftwareSimulator;
// import de.rwth.montisim.hardware_emulator.interfaces.SoftwareSimulatorManager;


// //This Manager can be used to allocate SoftwareSimulators on a remote machine, using RMI. (ex: RMIModelServer)
// public class RemoteSoftwareSimulatorManager implements SoftwareSimulatorManager {
//     int port_id;
//     public RemoteSoftwareSimulatorManager(SoftwareSimulatorConfig manager_config, int base_port) throws Exception {
//         super();
//         this.port_id = base_port + 1;
//         CppBridge.init(manager_config);
//     }
//     public SoftwareSimulator newSimulator(ControllerConfig config) throws Exception {
//         SoftwareSimulatorImpl simulator = new SoftwareSimulatorImpl(config);
//         return (SoftwareSimulator) UnicastRemoteObject.exportObject(simulator, getPortId());
//     }
//     private int getPortId(){
//         return ++port_id;
//     }

//     public String query(String msg) throws RemoteException {
//         return CppBridge.queryManager(msg);
//     }
// }