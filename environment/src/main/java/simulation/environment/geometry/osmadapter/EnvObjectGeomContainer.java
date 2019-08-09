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
package simulation.environment.geometry.osmadapter;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObject;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvObject;

/**
 * Created by lukas on 22.01.17.
 *
 * this is a Container for Geometric operations on Environment Objects
 */
public interface EnvObjectGeomContainer {

    /**
     *
     * @param o
     * @return Distance to the midpoint of this object from node
     */
    public abstract double getDistanceToMiddle(PhysicalObject o);

    /**
     *
     * @param n
     * @return Distance to the midpoint of this object from node
     */
    public abstract double getDistanceToMiddle(EnvNode n);


    /**
     *
     * @param o
     * @return Distance to the right border of this object from node
     */
    public abstract double getDistanceToRight(PhysicalObject o);

    /**
     *
     * @param o
     * @return Distance to the left border of this object from node
     */
    public abstract double getDistanceToLeft(PhysicalObject o);

    double getDistancetoFrontLeft(PhysicalObject o);

    double getDistancetoFrontRight(PhysicalObject o);

    /**
     *
     * @param x
     * @param y
     * @param lastKnownZ
     * @return returns the according z-Coordinate for given x, y and lastKnownZ
     */
    public abstract double getGround(double x, double y, double lastKnownZ);

    /**
     *
     * @param node
     * @return true iff this object contains this node
     */
    public abstract boolean contains(EnvNode node);

    /**
     *
     * @return returns the corresponding EnvObject
     */
    public abstract EnvObject getObject();
}