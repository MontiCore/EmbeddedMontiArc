/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces;

import java.rmi.Remote;
import java.rmi.RemoteException;

import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.ControllerConfig;

public interface SoftwareSimulatorManager extends Remote {
    public SoftwareSimulator newSimulator(ControllerConfig config) throws RemoteException, Exception;
    ///Queries general information, the available queries can be seen in software_simulator_manager.cpp->query()
    public String query(String msg) throws RemoteException;
}