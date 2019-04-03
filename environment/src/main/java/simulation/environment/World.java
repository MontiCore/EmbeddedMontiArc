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
package simulation.environment;

import commons.map.ControllerContainer;
import commons.map.IControllerNode;
import commons.simulation.IPhysicalVehicle;
import commons.simulation.PhysicalObject;
import javafx.geometry.Point3D;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.geometry.osmadapter.GeomStreet;
import simulation.environment.osm.Parser2D;
import simulation.environment.pedestrians.PedestrianContainer;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.environment.visualisationadapter.interfaces.VisualisationEnvironmentContainer;
import java.util.List;

/**
 * Created by Lukas Walbröl on 25.11.16.
 *
 * An Interface to access the World
 *
 * all distances are in meters!
 */
public interface World {

    /**
     * @param x x-Coordinate
     * @param y y-Coordinate
     * @param z the last known z-Coordinate
     * @return the z-Coordinate corresponding to x and y and that is next to the last known z-Coordinate
     */
    public abstract Number getGround(Number x, Number y, Number z);

    /**
     * @param x
     * @param y
     * @return the z-Coordinate using the z-Generator of the Environment. The result might differ from getGround()
     * if the node lies on a street!
     */
    public abstract Number getGroundForNonStreet(Number x, Number y);

    /**
     * @param o
     * @return the Distance to the middle of the street for the Point specified by o
     */
    public abstract Number getDistanceToMiddleOfStreet(PhysicalObject o);

    /**
     * @param v
     * @return the Distance to the left border of the street for the Point specified by v
     */
    public abstract Number getDistanceToLeftStreetBorder(IPhysicalVehicle v);

    Number getDistanceLeftFrontToStreetBorder(IPhysicalVehicle v);

    Number getDistanceRightFrontToStreetBorder(IPhysicalVehicle v);

    /**
     * @param v
     * @return the Distance to the right border of the street for the Point specified by v
     */
    public abstract Number getDistanceToRightStreetBorder(IPhysicalVehicle v);

    /**
     * @param v
     * @return the Distance to the left border of the street for the left back wheel
     */
    public abstract Number getDistanceFrontLeftWheelToLeftStreetBorder(IPhysicalVehicle v);

    /**
     * @param v
     * @return the Distance to the right border of the street for the right front wheel
     */
    public abstract Number getDistanceFrontRightWheelToRightStreetBorder(IPhysicalVehicle v);

    /**
     * @param v
     * @return the Distance to the left border of the street for the left back wheel
     */
    public abstract Number getDistanceBackLeftWheelToLeftStreetBorder(IPhysicalVehicle v);

    /**
     * @param v
     * @return the Distance to the right border of the street for the right back wheel
     */
    public abstract Number getDistanceBackRightWheelToRightStreetBorder(IPhysicalVehicle v);

    /**
     * @param x x-Coordinate
     * @param y y-Coordinate
     * @param z z-Coordinate
     * @return wether given cordinates correspond to a point on a street
     */
    boolean isPointOnStreet(double x, double y, double z);

    /**
     * @param v
     * @return returns surface of the point corresponding to the given coordinate vector
     */
    EnvStreet.StreetPavements getSurfaceType(RealVector v);

    /**
     * @param o
     * @return the Street the Vehicle o is on
     */
    public abstract GeomStreet getStreet(PhysicalObject o);

    /**
     * @return the Container to be used by the Visualisation-Group
     * @throws Exception
     */
    public abstract VisualisationEnvironmentContainer getContainer() throws Exception;

    /**
     * @return the Parser to be used by the Visualisation-Group
     * @throws Exception
     */
    public abstract Parser2D getParser() throws Exception;

    /**
     * @return true iff it is raining
     */
    public abstract boolean isItRaining();

    /**
     * @return a double representation of the weather
     */
    public abstract double getWeather();

    /**
     * @return the Container used to be by the Controller-Group
     */
    public abstract ControllerContainer getControllerMap();


    public abstract PedestrianContainer getPedestrianContainer();


    /**
     *
     * @param x
     * @param y
     * @param z
     * @param rightLane specifies if the car should drive on the right or the left lane
     * @return a Point on a street which is on the correct lane
     */
    public abstract Point3D spawnOnStreet(Number x, Number y, Number z, boolean rightLane);

    /**
     *
     * @param x
     * @param y
     * @param z
     * @return null if x and y lie on a street otherwise a point with Coordinates x,y and the z-Coordinate corresponding to x and y
     */
    public abstract Point3D spawnNotOnStreet(Number x, Number y, Number z);


    public abstract List<Long> getChangedTrafficSignals();

    public abstract IControllerNode getRandomNode();
}