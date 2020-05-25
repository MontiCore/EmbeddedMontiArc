/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.simulator;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.utils.Time;

public class SimulationConfig {
    Duration maxSimulationDuration = Duration.ofSeconds(60);
    Duration tickDuration = Duration.ofNanos(Time.SECOND_TO_NANOSEC/100);
    Instant simulationStart = Instant.now();
}