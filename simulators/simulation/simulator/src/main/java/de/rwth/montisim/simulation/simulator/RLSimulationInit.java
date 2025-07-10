/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.Instant;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.world.World;

import java.lang.Thread;

public class RLSimulationInit {
    final SimulationConfig config;
    Instant simulationTime;
    World world;
    Pathfinding pathfinding;
    OsmMap map;
    boolean distributed = false;
    boolean randomize = false;
    boolean play = false;
    boolean miniStep = false;
    String selfPlay_mode = ".";

    public RLSimulationInit(SimulationConfig config, OsmMap map) {
        this.config = config;
        this.simulationTime = config.start_time;
        this.map = map;
    }

    //initialize simulation handler
    public void init() {
        RLSimulationHandler rlSimulationHandler = new RLSimulationHandler(config, simulationTime, map, null);
        rlSimulationHandler.setSettings(distributed, randomize, play, miniStep, selfPlay_mode);
        rlSimulationHandler.start();
    }

    //set RL specific settings
    public void setRLSettings(boolean distributed, boolean randomize, boolean play, boolean miniStep, String selfPlay_mode) {
        this.distributed = distributed;
        this.randomize = randomize;
        this.play = play;
        this.miniStep = miniStep;
        this.selfPlay_mode = selfPlay_mode;
    }
}