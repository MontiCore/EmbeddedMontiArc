/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.io.File;
import java.time.Instant;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.SimulationConfig;
import de.rwth.montisim.simulation.simulator.Simulator;

import org.ros.exception.RosRuntimeException;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

public class RLSimulationInit {
    final SimulationConfig config;
    Instant simulationTime;
    World world;
    Pathfinding pathfinding;
    OsmMap map;
    boolean distributed = false;
    boolean randomize = false;
    boolean play = false;

    public RLSimulationInit(SimulationConfig config, World world, Pathfinding pathfinding, OsmMap map){
        this.config = config;
        this.simulationTime = config.start_time;
        this.world = world;
        this.pathfinding = pathfinding;
        this.map = map;
    }

    //initialize simulation handler
    public void init() {
        NodeConfiguration rosNodeConfiguration = NodeConfiguration.newPrivate();
        NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        NodeMain rlSimulationHandler = new RLSimulationHandler(config, simulationTime, world, pathfinding, map, null, nodeMainExecutor);
        ((RLSimulationHandler) rlSimulationHandler).setSettings(distributed, randomize, play);
        nodeMainExecutor.execute(rlSimulationHandler, rosNodeConfiguration);
    }

    //set RL specific settings
    public void setRLSettings(boolean distributed, boolean randomize, boolean play){
        this.distributed = distributed;
        this.randomize = randomize;
        this.play = play;
    }
}