/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulator.integration;

import java.rmi.AlreadyBoundException;
import java.rmi.Remote;
import java.rmi.RemoteException;

import java.io.Serializable;
import java.util.HashMap;

public interface RMIManager {
    public int alloc_autopilot(String config) throws RemoteException;
    public void free_autopilot(int id) throws RemoteException;

    public void update_bus(int id, HashMap<String, Serializable> inputs) throws RemoteException;
    
    public HashMap<String, Serializable> old_execute(int id, long time_delta, HashMap<String, Serializable> inputs) throws RemoteException;

    public void start_tick(long time_delta) throws RemoteException;
    public void end_tick() throws RemoteException;

    public HashMap<String, Serializable> get_outputs(int id) throws RemoteException;


    public String querry(String msg) throws RemoteException;
    public String querry_autopilot(int id, String msg) throws RemoteException;
}
