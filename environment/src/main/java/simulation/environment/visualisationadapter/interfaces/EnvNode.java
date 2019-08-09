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
package simulation.environment.visualisationadapter.interfaces;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.IControllerNode;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;

/**
 * Created by lukas on 15.12.16.
 *
 * An Interface for EnvironmentNodes
 */
public interface EnvNode extends IControllerNode {

    /**
     *
     * @return x-Coordinate of this Node
     */
    public abstract Number getX();

    /**
     *
     * @return y-Coordinate of this Node
     */
    public abstract Number getY();

    /**
     *
     * @return z-Coordinate of this Node
     */
    public abstract Number getZ();

    /**
     *
     * @return OpenStreetMap-Id of this Node
     */
    public abstract long getOsmId();

    /**
     * @return a representation of this Node as 3D-Point
     */
    public abstract Point3D getPoint();

    /**
     *
     * @return the street sign for this node
     */
    public abstract StreetSign getStreetSign();
}