/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.environment.geometry.splines;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.IControllerNode;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObject;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;
import simulation.environment.pedestrians.PedestrianStreetParameters;
import java.util.ArrayList;

/**
 * Created by lukas on 21.01.17.
 *
 * an interface for Splines
 */
public interface Spline {
    /**
     *
     * @param t between 0 and 1
     * @return the Point on that spline for given t
     */
    public abstract Point3D computePoint(double t);

    /**
     * computes the distance of a given point to a spline using this paper
     * http://www.tinaja.com/glib/cmindist.pdf
     * Note: Things get a lot easier if Linear Interpolation is used
     * @param p
     * @return distance from p to the middle spline
     */
    public abstract double computeDistanceToMiddle(Point3D p);

    /**
     *
     * @param p
     * @return analogously to computeDistanceToMiddle the distance from p to the left spline
     */
    public abstract double computeDistanceToLeft(Point3D p);

    /**
     *
     * @param o
     * @return analogously to computeDistanceToMiddle the distance from o to the left spline
     */
    public abstract double computeDistanceToLeft(PhysicalObject o);

    double computeDistanceToFrontLeft(PhysicalObject o);

    double computeDistanceToFrontRight(PhysicalObject o);

    /**
     *
     * @param p
     * @return analogously to computeDistanceToMiddle the distance from p to the right spline
     */
    public abstract double computeDistanceToRight(Point3D p);

    /**
     *
     * @param o
     * @return analogously to computeDistanceToMiddle the distance from o to the right spline
     */
    public abstract double computeDistanceToRight(PhysicalObject o);

    /**
     *
     * @param p
     * @return true iff p is a point on the spline
     */
    public abstract boolean isOnStreet(Point3D p);

    /**
     * @param p
     * @return true iff p is a point on a pavement
     */
    public abstract boolean isOnPavement(Point3D p);

    /**
     * @param p
     * @return returns the real z-Coordinate for p; Note p contains not the real z-Coordinate but the last known z-Coordinate
     */
    public double getFloorForPoint(Point3D p);


    /**
     * Converts this spline to a List of IControllerNode by interpolating points with fixed distances to each other
     * @return
     */
    public abstract ArrayList<IControllerNode> convertToControllerList();

    public abstract Point3D getDifference();

    public abstract Point3D getP1();

    public abstract Point3D getP2();

    public abstract Point3D getBorder(boolean left, boolean isP1);

    public abstract ArrayList<Point3D> getAllBorders();

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

    public abstract Point3D spawnCar(boolean rightLane, Point3D p);


}