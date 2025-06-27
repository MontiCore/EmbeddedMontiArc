/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.utils.Time;

/**
 * Contains the information on a new tick for the simulation:
 * the new and old time and the delta between them.
 */
public class TimeUpdate {
    /** Time before this update */
    public final Instant oldTime;
    /** New time after this update (old time = newTime - deltaTime) */
    public final Instant newTime;
    /** Time difference between last and current state. */
    public final Duration deltaTime;
    public final double deltaSeconds;

    public TimeUpdate(Instant oldTime, Duration deltaTime){
        this.oldTime = oldTime;
        this.deltaTime = deltaTime;
        this.newTime = oldTime.plus(deltaTime);
        this.deltaSeconds = Time.secondsFromDuration(deltaTime);
    }
}