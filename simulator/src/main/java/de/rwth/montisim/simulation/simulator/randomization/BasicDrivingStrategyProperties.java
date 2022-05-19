package de.rwth.montisim.simulation.simulator.randomization;

import java.io.File;
import java.nio.file.Paths;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToWorldLoader;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.SimulationConfig;

@Typed(BasicDrivingStrategyProperties.TYPE_LABEL)
public class BasicDrivingStrategyProperties extends RandomizationProperties {

    public static final String TYPE_LABEL = "basic";

    public double minGoalDistance = 500;
    public double maxGoalDistance = 1000;

    @Override
    public RandomizationStrategy build(SimulationConfig config, String mapsFolder) throws Exception {
        File mapPath = Paths.get(mapsFolder, config.map_name + ".osm").toFile();
        OsmMap map = new OsmMap(config.map_name, mapPath);
        World world = new OsmToWorldLoader(map).getWorld();
        return new BasicDrivingStrategy(seed, world, minGoalDistance, maxGoalDistance);
    }

}
