package de.rwth.montisim.simulation.simulator.randomization;

import java.io.File;
import java.nio.file.Paths;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToWorldLoader;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.SimulationConfig;

@Typed(PlatooningStrategyProperties.TYPE_LABEL)
public class PlatooningStrategyProperties extends RandomizationProperties {

    public static final String TYPE_LABEL = "platooning";

    public int minNumberOfVehicles = 2;
    public int maxNumberOfVehicles = 20;
    public double minDistanceBtwVehicles = 10;
    public double maxDistanceBtwVehicles = 20;
    public double minGoalDistance = 500;
    public double maxGoalDistance = 1000;

    @Override
    public RandomizationStrategy build(SimulationConfig config, String mapsFolder) throws Exception {
        File mapPath = Paths.get(mapsFolder, config.map_name + ".osm").toFile();
        OsmMap map = new OsmMap(config.map_name, mapPath);
        World world = new OsmToWorldLoader(map).getWorld();
        return new PlatooningStrategy(seed, world, minNumberOfVehicles, maxNumberOfVehicles, minDistanceBtwVehicles, maxDistanceBtwVehicles, minGoalDistance, maxGoalDistance);
    }

}
