/**
 *
 * ******************************************************************************
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
package simulation.simulator;

import commons.simulation.SimulationLoopExecutable;
import commons.simulation.SimulationLoopNotifiable;
import simulation.util.Plotter2D;
import simulation.vehicle.PhysicalVehicle;

import java.time.Duration;
import java.time.Instant;
import java.util.List;

/**
 * Debug Plotter that visualizes the movement for a simulation with only one physical vehicle
 */
public class SimulationDebugPlotter implements SimulationLoopNotifiable {

    private long counter = 0;
    private String name;

    public SimulationDebugPlotter(String name){
        this.name = name;
    }

    @Override
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration timeDiffms) {
        if(simulationObject instanceof PhysicalVehicle) {
            PhysicalVehicle physicalVehicle = (PhysicalVehicle) simulationObject;
            Plotter2D.plotOne(physicalVehicle, counter, timeDiffms, name);
        }
        counter++;
    }

    @Override
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, Instant totalTime) {}

    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {}

    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {}

    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration deltaTime) {}

    @Override
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}
}