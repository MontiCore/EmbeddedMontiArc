package de.rwth.montisim.simulation.simulator;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.Triplet;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.simulation.commons.util.CollisionLogWriter;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToWorldLoader;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.world.World;

public class SimulationCLI {
    
    /**
     * Runs a Scenario file until completion or failure.
     * Make sure the necessary Types for the JSON system are registered (TypedSimulation/TypedHardwareEmu)
     * and that the CppBridge is initialized if the hardware-emulator is used locally
     */
    public static TaskStatus runSimulationFromFile(String path, String mapsFolder) throws Exception {
        // Create simulator from scenario file
        File scenarioFile = new File(path);
        SimulationConfig config = SimulationConfig.fromFile(scenarioFile);
        File mapPath = Paths.get(mapsFolder, config.map_name + ".osm").toFile();
        OsmMap map = new OsmMap(config.map_name, mapPath);
        World world = new OsmToWorldLoader(map).getWorld();
        Pathfinding pathfinding = new PathfindingImpl(world);
        Simulator simulator = config.build(world, pathfinding, map);

        // Run simulation
        SimulationLoop simLoop = new SimulationLoop(simulator, config);
        TaskStatus res = simLoop.run();
        if (res == TaskStatus.SUCCEEDED) {
            // TODO: output formatted JSON for the simulation results ?
            System.out.println("Simulation SUCCEEDED.");
        } else if(res == TaskStatus.FAILED_TIMEOUT) {
            System.out.println("Simulation FAILED. REASON: Not all vehicles completed their tasks in time.");
        } else if(res == TaskStatus.FAILED_COLLISION){
            System.out.println("SIMULATION FAILED. REASON: Following collision(s) occured:");
            for(Triplet<String,String,Duration> collision : simulator.currentCollisions.keySet()){
                System.out.println("Collision between " + collision.getAt(0) + " and " + collision.getAt(1) + " after " + ((Duration) collision.getAt(2)).getSeconds() + " seconds.");
            }
        } else {
            System.out.println("Simulation FAILED.");
        }

        //write collision history to file
        if(simulator.config.collision_mode.equals("LOG_COLLISIONS")){
            CollisionLogWriter.addCollisions(simulator.collisionHistory, simulator.config.start_time);
        }
        return res;
    }
}
