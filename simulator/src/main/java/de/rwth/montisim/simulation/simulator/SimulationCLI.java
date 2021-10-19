package de.rwth.montisim.simulation.simulator;

import java.io.File;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.simulation.commons.TaskStatus;
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
    public static void runSimulationFromFile(String path){
        try {
            // Create simulator from scenario file
            File scenarioFile = new File(path);
            SimulationConfig config = SimulationConfig.fromFile(scenarioFile);
            File mapPath = new File(config.map_name + ".osm");
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
            } else {
                System.out.println("Simulation FAILED.");
                System.exit(-1);
            }
        } catch (Exception e1) {
            e1.printStackTrace();
            return;
        }
    }
}
