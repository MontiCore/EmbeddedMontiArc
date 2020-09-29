/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.hardware_emulator.interfaces;

import java.rmi.Remote;
import java.rmi.RemoteException;

import de.rwth.montisim.hardware_emulator.ComputerProperties;

public interface SoftwareSimulatorManager extends Remote {
    public SoftwareSimulator newSimulator(ComputerProperties config) throws RemoteException, Exception;
    ///Queries general information, the available queries can be seen in software_simulator_manager.cpp->query()
    public String query(String msg) throws RemoteException;
}