/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
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
