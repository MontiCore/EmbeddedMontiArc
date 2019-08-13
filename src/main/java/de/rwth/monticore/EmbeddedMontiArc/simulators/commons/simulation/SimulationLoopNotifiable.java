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
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

import java.util.List;

/**
 * Informs objects about state changes in the simulation. To be informed about
 * the simulation loop, you first need to register the object to the shared
 * instance of the simulator using registerLoopObserver().
 * Override the required functions to get notified for a given simulation event.
 */
public abstract class SimulationLoopNotifiable {
    /**
     * Is called just before the simulation objects update their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}

    /**
     * Is called after the simulation objects updated their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}

    /**
     * Is called for each object just before the simulation objects update their states.
     *
     * @param simulationObject Object for which the loop will be executed
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}

    /**
     * Is called for each object after the simulation objects updated their states.
     *
     * @param simulationObject Object for which the loop iteration has been completed
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}

    /**
     * Is called just before the simulation starts
     *
     * @param simulationObjects List of all simulation objects
     */
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}

    /**
     * Is called just after the simulation ends
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     */
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, long totalTime) {}
}
