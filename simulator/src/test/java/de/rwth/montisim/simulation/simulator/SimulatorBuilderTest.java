package de.rwth.montisim.simulation.simulator;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToWorldLoader;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.world.World;
import junit.framework.TestCase;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class SimulatorBuilderTest extends TestCase {
    public void testJson() throws Exception {
        World world = new OsmToWorldLoader(new OsmMap("aachen", new File("src/test/resources/aachen.osm"))).getWorld();
        Pathfinding pathfinding = new PathfindingImpl(world);
        MessageTypeManager mtManager = new MessageTypeManager();
        String configStr = new String(Files.readAllBytes(Paths.get("src/test/resources/simmulation_config.json")));
        SimulationConfig config = Json.instantiateFromJson(configStr, SimulationConfig.class);

        Simulator simulator = SimulatorBuilder
                .fromJsonConfig(configStr)
                .setWorld(world)
                .setPathfinding(pathfinding)
                .setMtManager(mtManager)
                .build();

        simulator.config.equals(config);
    }

    public void testWithDefaultConfig() throws Exception {
        World world = new OsmToWorldLoader(new OsmMap("aachen", new File("src/test/resources/aachen.osm"))).getWorld();
        Pathfinding pathfinding = new PathfindingImpl(world);
        MessageTypeManager mtManager = new MessageTypeManager();
        Simulator simulator = SimulatorBuilder
                .withDefaultConfig()
                .setWorld(world)
                .setPathfinding(pathfinding)
                .setMtManager(mtManager)
                .build();

        simulator.world.equals(world);
        simulator.pathfinding.equals(pathfinding);
        simulator.mtManager.equals(mtManager);
        simulator.config.equals(new SimulationConfig());
    }
}