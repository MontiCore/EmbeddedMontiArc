/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.utils.Time;

public class TimeUpdate {
    public final Instant oldTime; // Time before this update
    public final Instant newTime; // New time after this update (old time = newTime - deltaTime)
    public final Duration deltaTime; // Time difference between last and current state.
    public final double deltaSeconds;

    public TimeUpdate(Instant oldTime, Duration deltaTime){
        this.oldTime = oldTime;
        this.deltaTime = deltaTime;
        this.newTime = oldTime.plus(deltaTime);
        this.deltaSeconds = Time.secondsFromDuration(deltaTime);
    }
}