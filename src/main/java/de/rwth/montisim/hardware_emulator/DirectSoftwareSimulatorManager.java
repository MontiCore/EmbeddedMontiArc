// package de.rwth.montisim.hardware_emulator;

// import java.rmi.RemoteException;

// import de.rwth.montisim.hardware_emulator.config.SoftwareSimulatorConfig;
// import de.rwth.montisim.hardware_emulator.interfaces.SoftwareSimulator;
// import de.rwth.montisim.hardware_emulator.interfaces.SoftwareSimulatorManager;

// //This Manager can be used to allocate SoftwareSimulators on the same machine.
// public class DirectSoftwareSimulatorManager implements SoftwareSimulatorManager {
//     public DirectSoftwareSimulatorManager(SoftwareSimulatorConfig manager_config) throws Exception {
//         super();
//         CppBridge.init(manager_config);
//     }
//     public SoftwareSimulator newSimulator(ControllerConfig config) throws Exception {
//         return new SoftwareSimulatorImpl(config);
//     }

//     public String query(String msg) throws RemoteException {
//         return CppBridge.queryManager(msg);
//     }

//     @Override
//     public SoftwareSimulator newSimulator(ComputerProperties config) throws RemoteException, Exception {
//         // TODO Auto-generated method stub
//         return null;
//     }
// }