/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.world.elements;

import java.util.Vector;

import de.rwth.montisim.commons.utils.Vec3;

public class Way {
    public final String name;
    public final boolean oneWay;
    public final int lanes;
    public final boolean isArea;
    public Vec3 centerPoint = null; // For AREAS
    public final double maxSpeed;
    public final Vector<Vec3> points = new Vector<>();
    /**
     * For a given point, the nodeID is >= 0 if there is an associated intersection (=> Node).
     */
    public final Vector<Integer> nodeID = new Vector<>();

    public int localID; // -1 if not set

    public Way(int localID, String name, boolean oneWay, int lanes, boolean isArea, double maxSpeed) {
        this.isArea = isArea;
        this.localID = localID;
        this.name = name;
        this.oneWay = oneWay;
        this.lanes = lanes;
        this.maxSpeed = maxSpeed;
    }

    public Way(String name, boolean oneWay, int lanes, boolean isArea, double maxSpeed) {
        this(-1, name, oneWay, lanes, isArea, maxSpeed);
    }

    public void addPoint(Vec3 point, int nodeID) {
        this.points.add(point);
        this.nodeID.add(nodeID);
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