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

import javafx.geometry.Point2D;
import javafx.geometry.Point3D;
import java.util.Collection;

/**
 * Created by lukas on 12.01.17.
 *
 * An interface for an EnvironmentContainer to be used by the Visualisation-Group only!
 */
public interface VisualisationEnvironmentContainer {
    /**
     * @return a Collection of all streets in the container
     */
    public abstract Collection<EnvStreet> getStreets();

    /**
     * @return a Collection of all streets in the container
     */
    public abstract Collection<Waterway> getWaterway();

    /**
     * @return a Collection of all buildings in the container
     */
    public abstract Collection<Building> getBuildings();

    /**
     * @return a Collection of all trees in the container
     */
    public abstract Collection<EnvNode> getTrees();

    /**
     * @return the boundaries of the container
     */
    public abstract EnvBounds getBounds();

    /**
     * @return the midpoint of this environment use for the height calculation
     */
    public abstract Point3D getMidpoint();

    /**
     * @return a two dimensional array with three rows, first row contains the circle height, second row the slope to the next circle, third row the distance to the next circle
     */
    public abstract double[][] getHeightMap();

    public abstract Point2D getHeightMapMinPoint();

    public abstract Point2D getHeightMapMaxPoint();

    public abstract double getHeightMapDeltaX();

    public abstract double getHeightMapDeltaY();
}