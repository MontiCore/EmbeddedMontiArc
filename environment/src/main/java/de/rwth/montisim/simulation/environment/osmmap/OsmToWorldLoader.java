/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.osmmap;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Building;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;

public class OsmToWorldLoader {
    public static final double DEFAULT_MAX_SPEED = 50; // km/h

    private final OsmMap osmMap;
    private final World world;
    private final SimpleCoordinateConverter converter;

    final int nodeRoadCounter[];
    final int intersectionID[];
    final Vec3 nodePoint[];
    final List<OsmMap.Way> roads = new ArrayList<>();

    public OsmToWorldLoader(OsmMap osmMap) {
        this.osmMap = osmMap;
        this.world = new World(osmMap.name);
        this.converter = new SimpleCoordinateConverter(osmMap.mid_point);

        int nodeCount = osmMap.nodeTable.size();
        this.nodeRoadCounter = new int[nodeCount];
        this.intersectionID = new int[nodeCount];
        this.nodePoint = new Vec3[nodeCount];
        for (int i = 0; i < nodeCount; ++i) {
            nodeRoadCounter[i] = 0;
            intersectionID[i] = -1;
        }
    }

    public World getWorld() {
        long start = System.nanoTime();

        // Convert coordinates
        world.converter = Optional.of(converter);
        converter.coordsToMeters(osmMap.min_corner, world.minCorner);
        converter.coordsToMeters(osmMap.max_corner, world.maxCorner);

        int nodeCount = osmMap.nodeTable.size();
        for (int i = 0; i < nodeCount; ++i) {
            OsmMap.Node node = osmMap.nodeTable.elementAt(i);
            if (node != null) {
                Vec3 pos = new Vec3(0, 0, 0);
                converter.coordsToMeters(node.coords, pos);
                nodePoint[i] = pos;
            }
        }

        // Count roads & add buildings
        for (OsmMap.Way way : osmMap.wayTable) {
            if (osmMap.isRoad(way)) {
                roads.add(way);
                countIntersections(way);
            } else if (osmMap.isBuilding(way))
                parseBuilding(way);
        }

        createIntersections();

        // Load Roads
        for (OsmMap.Way way : roads) {
            parseWays(way);
        }

        world.finalizeGraph();

        long end = System.nanoTime();
        System.out.println("Loaded World from OsmMap " + world.name + " in " + ((end - start) / 1000000.0) + " ms.");
        return world;
    }

    private void countIntersections(OsmMap.Way way) {
        for (int nid : way.nodesLocalID) {
            nodeRoadCounter[nid]++;
        }
    }

    private void createIntersections() {
        for (int i = 0; i < nodeRoadCounter.length; ++i) {
            int count = nodeRoadCounter[i];
            if (count > 1) {
                OsmMap.Node node = osmMap.nodeTable.elementAt(i);
                if (node != null) {
                    Node is = new Node(nodePoint[i]);
                    intersectionID[i] = world.addNode(is);
                }
            }
        }
    }

    private void parseWays(OsmMap.Way way) {
        if (way.nodesLocalID.size() <= 1) return;
        // Parse a new road
        String roadNameTag = way.getTag(osmMap.TAG.NAME);
        String oneWayTag = way.getTag(osmMap.TAG.ONEWAY);
        String lanesTag = way.getTag(osmMap.TAG.LANES);
        String areaTag = way.getTag(osmMap.TAG.AREA);
        String maxspeedTag = way.getTag(osmMap.TAG.MAX_SPEED);
        Way w = new Way(
                roadNameTag != null ? roadNameTag : "",
                oneWayTag != null && oneWayTag.equals(osmMap.VALUE_YES.str),
                lanesTag != null ? Integer.valueOf(lanesTag) : 1,
                areaTag != null && areaTag.equals(osmMap.VALUE_YES.str),
                maxspeedTag != null ? Double.parseDouble(maxspeedTag) : DEFAULT_MAX_SPEED
        );

        int count = way.nodesLocalID.size();
        for (int i = 0; i < count; ++i) {
            int nid = way.nodesLocalID.elementAt(i);
            Vec3 p = nodePoint[nid];
            if (p != null) {
                w.addPoint(p, intersectionID[nid]);
            }
        }

        world.addWay(w);
    }

    private void parseBuilding(OsmMap.Way way) {
        // Parse a new building
        String nameTag = way.getTag(osmMap.TAG.NAME);
        String type = way.getTag(osmMap.TAG.BUILDING);
        String heightTag = way.getTag(osmMap.TAG.HEIGHT);
        String levelsTag = way.getTag(osmMap.TAG.BUILDING_LEVELS);

        Building b = new Building(
                nameTag != null ? nameTag : "",
                way.osmID,
                type.equals(osmMap.VALUE_YES.str) ? "" : type,
                parseHeight(heightTag),
                parseLevels(levelsTag)
        );

        // Add boundary
        Vec3 lastP = null;
        for (int nid : way.nodesLocalID) {
            Vec3 p = nodePoint[nid];
            if (p == null) continue;
            if (lastP != null && p.distance(lastP) < 0.001) continue; // Ensure no points are too close to each other
            b.boundary.add(p);
            lastP = p;
        }
        // Remove last point if it is the same as the first
        if (b.boundary.size() >= 2 && b.boundary.get(0).distance(b.boundary.get(b.boundary.size() - 1)) < 0.001)
            b.boundary.remove(b.boundary.size() - 1);

        world.buildings.add(b);
    }

    private double parseHeight(String value) {
        if (value == null || value.length() == 0) return 0;
        value = value.replace(',', '.');
        if (value.charAt(value.length() - 1) == 'm')
            return Double.valueOf(value.substring(0, value.length() - 1));
        if (value.contains("'") || value.contains("\"")) {
            throw new IllegalArgumentException("TODO: parse Foot/Inch height.");
            //return INCH_LENGTH*inches+FOOT_LENGTH*feet;
        }
        return Double.valueOf(value);
    }

    private int parseLevels(String value) {
        if (value == null || value.length() == 0) return 0;
        if (value.contains(".")) {
            return (int) Math.floor(Double.valueOf(value));
        }
        return Integer.valueOf(value);
    }
}