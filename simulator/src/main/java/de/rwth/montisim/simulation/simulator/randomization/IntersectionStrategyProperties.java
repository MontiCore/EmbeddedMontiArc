package de.rwth.montisim.simulation.simulator.randomization;

import java.io.File;
import java.nio.file.Paths;
import java.util.Optional;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToWorldLoader;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.SimulationConfig;

@Typed("intersection")
public class IntersectionStrategyProperties extends RandomizationProperties {
  
  public int min_number_of_vehicles = 1;
  public int max_number_of_vehicles = 1;
  public double min_distance_btw_vehicles = 8;
  public double max_distance_btw_vehicles = 8;
  public double min_distance_from_intersection = 5;
  public double max_distance_from_intersection = 5;
  
  @Override
  public RandomizationStrategy build(SimulationConfig config, String mapsFolder) throws Exception {
    File mapPath = Paths.get(mapsFolder, config.map_name + ".osm").toFile();
    OsmMap map = new OsmMap(config.map_name, mapPath);
    World world = new OsmToWorldLoader(map).getWorld();
    Optional<Long> usedSeed = (seed.isPresent())? seed : Optional.of(System.nanoTime());
    System.out.println("Seed: " + usedSeed.get());
    return new IntersectionStrategy(usedSeed, world, max_number_of_vehicles, min_number_of_vehicles, min_distance_btw_vehicles, max_distance_btw_vehicles, min_distance_from_intersection, max_distance_from_intersection);
  }
  
}
