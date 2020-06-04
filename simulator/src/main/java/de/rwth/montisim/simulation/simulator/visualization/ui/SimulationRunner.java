/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import java.time.Instant;

public interface SimulationRunner {
    /**
     * @param timePoint For a Simulation: the (simulation) time at which an update step should be computed.
     * For a replay: the moment that should be shown.
     * @return The simulated duration in nanoseconds for a Simulation. 
     * Return 0 to stop the simulation/replay.
     * Must be greater than 0 for replays to keep on playing.
     */
    long run(Instant timePoint);

    void redraw();

    void reset();
}