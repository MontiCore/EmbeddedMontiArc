/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.map;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.environment.map.elements.*;

/**
 * An intermediary representation of the contents of the map.
 * Does not contain precise geometry, only deterministic information 
 * to build it.
 * 
 * This representation is serializable into json and can be created
 * from an [OsmMap].
 */
public class Map {
    public final String name;
    public Optional<SimpleCoordinateConverter> converter = Optional.empty();
    public Vec2 minCorner = new Vec2();
    public Vec2 maxCorner = new Vec2();

    public final List<Road> roads = new ArrayList<>();
    public final List<Intersection> intersections = new ArrayList<>();
    public final List<Building> buildings = new ArrayList<>();
    public final List<ChargingStation> charingStations = new ArrayList<>();
    public final List<Lights> lights = new ArrayList<>();
    public final List<Sign> signs = new ArrayList<>();
    public final List<Waterway> waterways = new ArrayList<>();

    public Map(String name){
        this.name = name;
    }
    
}