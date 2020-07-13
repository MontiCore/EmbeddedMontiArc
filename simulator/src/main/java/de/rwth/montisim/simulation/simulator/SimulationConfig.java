/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.*;

import de.rwth.montisim.commons.utils.Time;

public class SimulationConfig {
    Duration max_duration = Duration.ofSeconds(60);
    Duration tick_duration = Duration.ofNanos(Time.SECOND_TO_NANOSEC/100);
    Instant start_time = Instant.now();

    
}