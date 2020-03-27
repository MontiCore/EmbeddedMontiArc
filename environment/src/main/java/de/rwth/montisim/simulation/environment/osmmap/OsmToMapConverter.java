/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osmmap;

import java.util.HashMap;
import java.util.Optional;

import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.map.Map;
import de.rwth.montisim.simulation.environment.map.elements.Building;
import de.rwth.montisim.simulation.environment.map.elements.Road;

public class OsmToMapConverter {
    private static final double INCH_LENGTH = 0.0254;
    private static final double FOOT_LENGTH = 0.3048;

    private OsmMap osmMap;
    private Map map;
    private SimpleCoordinateConverter conv;

    public OsmToMapConverter(OsmMap osmMap){
        this.osmMap = osmMap;
        this.map = new Map(osmMap.name);
        this.conv = new SimpleCoordinateConverter(osmMap.mid_point);
    }

    public Map getMap(){
        long start = System.nanoTime();
        
        map.converter = Optional.of(conv);
        map.minCorner = conv.coordsToMeters(osmMap.min_corner);
        map.maxCorner = conv.coordsToMeters(osmMap.max_corner);

        for (HashMap.Entry<Long, OsmMap.Way> c : osmMap.ways.entrySet()){
            OsmMap.Way way = c.getValue();
            if (osmMap.isRoad(way))
                parseRoad(way);
            else if (osmMap.isBuilding(way))
                parseBuilding(way);
        }

        long end = System.nanoTime();
        System.out.println("Loaded Map from OsmMap " + map.name + " in " + ((end-start)/1000000.0) + " ms.");
        return map;
    }

    

    private void parseRoad(OsmMap.Way way) {
        // Parse a new road
        String roadNameTag = way.getTag(osmMap.TAG.NAME);
        String oneWayTag = way.getTag(osmMap.TAG.ONEWAY);
        String lanesTag = way.getTag(osmMap.TAG.LANES);
        String areaTag = way.getTag(osmMap.TAG.AREA);
        Road r = new Road(
            roadNameTag !=null ? roadNameTag : "", 
            oneWayTag != null && oneWayTag.equals(osmMap.VALUE_YES.str),
            lanesTag != null ? Integer.valueOf(lanesTag) : 1,
            areaTag != null && areaTag.equals(osmMap.VALUE_YES.str)
        );

        // Add points of the road
        for (Long nid : way.nodes){
            OsmMap.Node node = osmMap.nodes.get(nid);
            if (node != null){
                Vec2 pos = conv.coordsToMeters(node.coords);
                r.points.add(new Vec3(pos.x, pos.y, 0));
            }
        }
        map.roads.add(r);
    }

    private void parseBuilding(OsmMap.Way way) {
        // Parse a new building
        String nameTag = way.getTag(osmMap.TAG.NAME);
        String type = way.getTag(osmMap.TAG.BUILDING);
        String heightTag = way.getTag(osmMap.TAG.HEIGHT);
        String levelsTag = way.getTag(osmMap.TAG.BUILDING_LEVELS);

        Building b = new Building(
            nameTag !=null ? nameTag : "", 
            type.equals(osmMap.VALUE_YES.str) ? "" : type, 
            parseHeight(heightTag), 
            parseLevels(levelsTag)
        );

        // Add boundary
        for (Long nid : way.nodes){
            OsmMap.Node node = osmMap.nodes.get(nid);
            if (node != null){
                b.boundary.add(conv.coordsToMeters(node.coords));
            }
        }
        map.buildings.add(b);
    }

    private double parseHeight(String value){
        if (value == null || value.length() == 0) return 0;
        value = value.replace(',', '.');
        if (value.charAt(value.length()-1) == 'm')
            return Double.valueOf(value.substring(0, value.length()-1));
        if (value.contains("'") || value.contains("\"") ){
            throw new IllegalArgumentException("TODO: parse Foot/Inch height.");
            //return INCH_LENGTH*inches+FOOT_LENGTH*feet;
        }
        return Double.valueOf(value);
    }

    private int parseLevels(String value){
        if (value == null || value.length() == 0) return 0;
        if (value.contains(".")){
            return (int) Math.floor(Double.valueOf(value));
        }
        return Integer.valueOf(value);
    }
}