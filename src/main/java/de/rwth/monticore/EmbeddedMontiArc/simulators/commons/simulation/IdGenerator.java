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

import java.util.concurrent.atomic.AtomicLong;

/**
 * Generates unique Ids for, e.g., PhysicalObject
 */
public class IdGenerator {
    /** Singleton instance */
    private static IdGenerator sharedInstance = new IdGenerator();

    /**
     * The maximum ID handed out. Not reset between simulations to keep
     * IDs even between different simulations unique.
     */
    private final AtomicLong maximumID = new AtomicLong(1);

    /** Prevent others from instantiating this class*/
    private IdGenerator() {}

    /**
     * Provides access to the shared instance of the IdGenerator.
     *
     * @return The shared instance of the IdGenerator.
     */
    public static IdGenerator getSharedInstance() {
        return sharedInstance;
    }

    /**
     * Reset instance of the IdGenerator.
     */
    public static void resetInstance() {
        sharedInstance = new IdGenerator();
    }

    /**
     * Generates a positive ID that is unique in the simulation. If the simulator is reset,
     * the IDs are not given out a second time. 0 is invalid.
     *
     * @return Unique ID that can be used, e.g., to identify objects
     */
    public long generateUniqueId() {
        synchronized (maximumID) {
            //If all positive IDs are handed out, stop handing out IDs.
            //As Long.MAX_VALUE is 2^63-1, this will most likely never happen
            if (maximumID.get() == Long.MAX_VALUE) {
                return 0L;
            }

            //Default case: return current maximumID and increase by 1
            return maximumID.getAndIncrement();
        }
    }
}
