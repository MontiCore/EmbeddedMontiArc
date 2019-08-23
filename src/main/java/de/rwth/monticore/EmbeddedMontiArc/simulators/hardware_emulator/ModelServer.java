/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;

import java.rmi.Remote;
import java.rmi.RemoteException;

import java.io.Serializable;
import java.util.HashMap;

public interface ModelServer extends Remote {
    public int alloc_autopilot(String config) throws RemoteException;
    public void free_autopilot(int id) throws RemoteException;

    public void update_bus(int id, HashMap<String, Serializable> inputs) throws RemoteException;

    public HashMap<String, Serializable> old_execute(int id, long time_delta, HashMap<String, Serializable> inputs) throws RemoteException;

    public void start_tick(long time_delta) throws RemoteException;
    public void end_tick() throws RemoteException;

    public HashMap<String, Serializable> get_outputs(int id) throws RemoteException;


    public String query(String msg) throws RemoteException;
    public String query_autopilot(int id, String msg) throws RemoteException;
}
