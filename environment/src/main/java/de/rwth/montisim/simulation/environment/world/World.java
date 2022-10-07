/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.world;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.commons.utils.BuildObject;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.elements.*;

/**
 * An intermediary representation of the contents of the map.
 * Does not contain precise geometry, only deterministic information
 * to build it.
 * <p>
 * This representation is serializable into json and can be created
 * from an [OsmMap].
 */
public class World implements BuildObject {
    public static final String CONTEXT_KEY = "world";
    public final String name;

    public Optional<SimpleCoordinateConverter> converter = Optional.empty();
    public Vec2 minCorner = new Vec2();
    public Vec2 maxCorner = new Vec2();

    // Physical Objects With Geometry
    // public final Vector<Road> roads = new Vector<>();
    // public final Vector<Intersection> intersections = new Vector<>();
    public final List<Building> buildings = new ArrayList<>();
    public final List<ChargingStation> charingStations = new ArrayList<>();
    public final List<Lights> lights = new ArrayList<>();
    public final List<Sign> signs = new ArrayList<>();
    public final List<Waterway> waterways = new ArrayList<>();

    // Navigation graph. The Nodes are the vertices & the WaySegments are the DIRECTED edges.
    public final Vector<Node> nodes = new Vector<>();
    public final Vector<Way> ways = new Vector<>();
    public final Vector<WaySegment> waySegments = new Vector<>();


    public int addWay(Way way) {
        int id = ways.size();
        if (way.localID < 0) { // local ID not set yet
            way.setLocalID(id);
        }
        ways.add(way);
        return id;
    }

    /**
     * @param s New Road Segment to add to the world.
     * @return Its local ID.
     */
    public int addWaySegment(WaySegment s) {
        int id = waySegments.size();
        if (s.localID < 0) { // local ID not set yet
            s.setLocalID(id);
        }
        waySegments.add(s);
        return id;
    }

    /**
     * @param is New Intersection to add to the world.
     * @return Its local ID.
     */
    public int addNode(Node is) {
        int id = nodes.size();
        if (is.localID < 0) { // local ID not set yet
            is.setLocalID(id);
        }
        nodes.add(is);
        return id;
    }

    public WaySegment getWaySegment(int localID) {
        return waySegments.elementAt(localID);
    }

    public Node getNode(int localID) {
        return nodes.elementAt(localID);
    }

    public World(String name) {
        this.name = name;
    }

    /**
     * Takes the current Way & Node information and computes the WaySegments.
     */
    public void finalizeGraph() {
        // Get AREA center points
        for (Way way : ways) {
            if (!way.isArea) continue;
            if (way.points.size() == 0) continue;
            way.centerPoint = new Vec3(0, 0, 0);
            for (Vec3 p : way.points) IPM.add(way.centerPoint, p);
            IPM.multiply(way.centerPoint, 1.0 / way.points.size());
        }

        for (Way way : ways) {
            double time_factor = 3.6 / way.maxSpeed;

            boolean hasCurrentSegment = false;
            Vec3 lastPos = null;
            double length = 0;
            int lastNodeID = -1;
            Node lastNode = null;
            int lastPointIndex = 0;

            int count = way.points.size();
            for (int i = 0; i < count; ++i) {
                // Track length
                if (lastPos == null) {
                    lastPos = way.points.elementAt(i);
                } else {
                    Vec3 p = way.points.elementAt(i);
                    length += p.distance(lastPos);
                    lastPos = p;
                }
                // Check if intersection
                int nid = way.nodeID.elementAt(i);
                if (nid >= 0) {
                    Node node = getNode(nid);
                    node.ways.add(way);
                    if (hasCurrentSegment) {
                        WaySegment segment = new WaySegment(way, length, time_factor, lastPointIndex, i, lastNodeID, nid);
                        int segmentID = addWaySegment(segment);
                        if (lastNode != null) lastNode.outRoadSegmentIDs.add(segmentID);
                        node.inRoadSegmentIDs.add(segmentID);
                        if (!way.oneWay) {
                            WaySegment reverseSegment = new WaySegment(way, length, time_factor, i, lastPointIndex, nid, lastNodeID);
                            int reverseSegmentID = addWaySegment(reverseSegment);
                            node.outRoadSegmentIDs.add(reverseSegmentID);
                            if (lastNode != null) lastNode.inRoadSegmentIDs.add(reverseSegmentID);
                            reverseSegment.reverseId = segmentID;
                            segment.reverseId = reverseSegmentID;
                        }
                    }
                    length = 0;
                    lastNodeID = nid;
                    lastNode = node;
                    lastPointIndex = i;
                }
                if (!hasCurrentSegment) hasCurrentSegment = true;
            }

            // If the last piece of road doesn't arrive at an intersection
            if (lastNodeID != way.nodeID.elementAt(count - 1)) {
                WaySegment segment = new WaySegment(way, length, time_factor, lastPointIndex, count - 1, lastNodeID, -1);
                int segmentID = addWaySegment(segment);
                if (lastNode != null) lastNode.outRoadSegmentIDs.add(segmentID);

                if (!way.oneWay) {
                    WaySegment reverseSegment = new WaySegment(way, length, time_factor, count - 1, lastPointIndex, -1, lastNodeID);
                    int reverseSegmentID = addWaySegment(reverseSegment);
                    if (lastNode != null) lastNode.inRoadSegmentIDs.add(reverseSegmentID);
                    reverseSegment.reverseId = segmentID;
                    segment.reverseId = reverseSegmentID;
                }
            }
        }

    }

    @Override
    public String getKey() {
        return CONTEXT_KEY;
    }
}