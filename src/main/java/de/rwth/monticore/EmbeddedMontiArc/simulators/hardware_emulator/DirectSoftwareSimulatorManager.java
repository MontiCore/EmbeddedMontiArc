/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;

import java.rmi.RemoteException;

import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.ControllerConfig;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.SoftwareSimulatorConfig;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces.SoftwareSimulator;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces.SoftwareSimulatorManager;

//This Manager can be used to allocate SoftwareSimulators on the same machine.
public class DirectSoftwareSimulatorManager implements SoftwareSimulatorManager {
    public DirectSoftwareSimulatorManager(SoftwareSimulatorConfig manager_config) throws Exception {
        super();
        CppBridge.init(manager_config);
    }
    public SoftwareSimulator newSimulator(ControllerConfig config) throws Exception {
        return new SoftwareSimulatorImpl(config);
    }

    public String query(String msg) throws RemoteException {
        return CppBridge.queryManager(msg);
    }
}