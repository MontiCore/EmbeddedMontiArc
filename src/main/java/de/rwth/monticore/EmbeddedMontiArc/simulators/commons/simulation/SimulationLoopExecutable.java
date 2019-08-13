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

/**
 * Should be implemented by all objects represented in the simulation that should
 * update their state during the simulation. To be updated, objects need to register
 * themselves to the shared instance of the simulator using registerSimulationObject().
 */
public interface SimulationLoopExecutable {
    /**
     * Function that requests the called object to update its state for given time difference
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    void executeLoopIteration(long timeDiffMs);
}
