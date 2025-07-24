/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.world.elements;

/**
 * Represents a DIRECTED edge in the 'drivable' graph.
 * Corresponds to a piece of road between 2 intersections.
 * It is a virtual construct used for routing.
 */
public class WaySegment {
    public final Way way;
    public int reverseId;
    public final double length;
    public final double travelCost; // Cost per traveled dist for this road
    /**
     * Start index into the road.points array. INCLUSIVE.
     */
    public final int pointsStart;
    /**
     * End index into the road.points array. INCLUSIVE.
     */
    public final int pointsEnd;

    public final int startNodeID;
    public final int endNodeID;

    public int localID; // -1 if not set

    public WaySegment(int localID,
                      Way road, double length, double travelCost,
                      int pointsStart, int pointsEnd,
                      int startNodeID, int endNodeID
    ) {
        this.localID = localID;
        this.way = road;
        this.length = length;
        this.travelCost = travelCost;
        this.pointsStart = pointsStart;
        this.pointsEnd = pointsEnd;
        this.startNodeID = startNodeID;
        this.endNodeID = endNodeID;
        this.reverseId = -1;
    }

    public WaySegment(Way road, double length, double travelCost,
                      int pointsStart, int pointsEnd,
                      int startNodeID, int endNodeID
    ) {
        this(-1, road, length, travelCost, pointsStart, pointsEnd, startNodeID, endNodeID);
    }


    public boolean setLocalID(int localID) {
        if (this.localID < 0 && localID >= 0) {
            this.localID = localID;
            return true;
        } else {
            return false;
        }
    }
}