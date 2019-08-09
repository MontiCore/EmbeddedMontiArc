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
package simulation.environment.geometry.height;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;
import simulation.environment.osm.ApproximateConverter;

/**
 * Created by lukas on 16.02.17.
 *
 * An interface specifying a height generator
 */
public interface HeightGenerator {
    /**
     * @param x
     * @param y
     * @return the z-Coordinate corresponding to x and y
     */
    public abstract double getGround(double x, double y);

    public abstract double[][] toHeightMap();

    public abstract Point2D getHeightMapMinPoint();

    public abstract Point2D getHeightMapMaxPoint();

    public abstract double getHeightMapDeltaX();

    public abstract double getHeightMapDeltaY();

    public abstract void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter);
}