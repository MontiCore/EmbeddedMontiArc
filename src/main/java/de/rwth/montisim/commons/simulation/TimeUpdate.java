/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import java.time.Duration;
import java.time.Instant;

public class TimeUpdate {
    public final Instant newTime; // New time after this update (old time = newTime - deltaTime)
    public final Duration deltaTime; // Time difference between last and current state.

    public TimeUpdate(Instant newTime, Duration deltaTime){
        this.newTime = newTime;
        this.deltaTime = deltaTime;
    }

    public static TimeUpdate fromOldTime(Instant oldTime, Duration deltaTime){
        return new TimeUpdate(oldTime.plus(deltaTime), deltaTime);
    }
}