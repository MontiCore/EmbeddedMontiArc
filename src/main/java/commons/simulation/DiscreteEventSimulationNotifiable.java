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
package commons.simulation;

import java.time.Duration;

/**
 * Interface for objects that want to get notified by a discrete event simulation about specified actions
 */
public interface DiscreteEventSimulationNotifiable {
    /**
     * Function that is called when a discrete event is scheduled
     *
     * @param discreteEvent Discrete event that is scheduled
     */
    public void onScheduleEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called after a discrete event is scheduled
     *
     * @param discreteEvent Discrete event that is scheduled
     */
    public void afterScheduleEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called when the processing function for a discrete event is called
     *
     * @param discreteEvent Discrete event that is processed
     */
    public void onProcessEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called after the processing function for a discrete event is called
     *
     * @param discreteEvent Discrete event that is processed
     */
    public void afterProcessEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called after the simulation time is advanced
     *
     * @param deltaTime Delta simulation time
     */
    public void onAdvanceSimulationTime(Duration deltaTime);

    /**
     * Function that is called after the simulation time is advanced
     *
     * @param deltaTime Delta simulation time
     */
    public void afterAdvanceSimulationTime(Duration deltaTime);

}
