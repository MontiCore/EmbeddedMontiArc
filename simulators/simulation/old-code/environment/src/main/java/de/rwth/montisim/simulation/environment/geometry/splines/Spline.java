/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.splines;

import de.rwth.montisim.commons.map.ControllerNode;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.pedestrians.PedestrianStreetParameters;

import java.util.ArrayList;

/**
 * an interface for Splines
 */
public interface Spline {
    /**
     *
     * @param t between 0 and 1
     * @return the Point on that spline for given t
     */
    public abstract Vec3 computePoint(double t);

    /**
     * computes the distance of a given point to a spline using this paper
     * http://www.tinaja.com/glib/cmindist.pdf
     * Note: Things get a lot easier if Linear Interpolation is used
     * @param p
     * @return distance from p to the middle spline
     */
    public abstract double computeDistanceToMiddle(Vec3 p);

    /**
     *
     * @param p
     * @return analogously to computeDistanceToMiddle the distance from p to the left spline
     */
    public abstract double computeDistanceToLeft(Vec3 p);

    double computeDistanceToFrontLeft(Vec3 p);

    double computeDistanceToFrontRight(Vec3 p);

    /**
     *
     * @param p
     * @return analogously to computeDistanceToMiddle the distance from p to the right spline
     */
    public abstract double computeDistanceToRight(Vec3 p);

    /**
     *
     * @param p
     * @return true iff p is a point on the spline
     */
    public abstract boolean isOnStreet(Vec3 p);

    /**
     * @param p
     * @return true iff p is a point on a pavement
     */
    public abstract boolean isOnPavement(Vec3 p);

    /**
     * @param p
     * @return returns the real z-Coordinate for p; Note p contains not the real z-Coordinate but the last known z-Coordinate
     */
    public double getFloorForPoint(Vec3 p);


    /**
     * Converts this spline to a List of IControllerNode by interpolating points with fixed distances to each other
     * @return
     */
    public abstract ArrayList<ControllerNode> convertToControllerList();

    public abstract Vec3 getDifference();

    public abstract Vec3 getP1();

    public abstract Vec3 getP2();

    public abstract Vec3 getBorder(boolean left, boolean isP1);

    public abstract ArrayList<Vec3> getAllBorders();

    /**
     *
     * @param lastResult
     * @param distance
     * @return a container which encapsulates the new movement of the pedestrian
     */
    public abstract PedestrianStreetParameters computePointForPedestrian(PedestrianStreetParameters lastResult, double distance);

    /**
     * @param isLeft
     * @return left or right pavement corresponding to this spline
     */
    public abstract LinearInterpolator getPavement(boolean isLeft);

    /**
     *
     * @param rightLane true iff car should use the right lane
     * @param p the nearest point to the spawning position
     * @return the position of the car spawned
     */

    public abstract Vec3 spawnCar(boolean rightLane, Vec3 p);


}
