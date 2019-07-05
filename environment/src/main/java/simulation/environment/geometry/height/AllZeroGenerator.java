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

import commons.utils.Point2D;
import simulation.environment.osm.ApproximateConverter;
import simulation.environment.visualisationadapter.interfaces.EnvBounds;

/**
 * Created by lukas on 23.02.17.
 *
 * A height Generator that returns always zero
 */
public class AllZeroGenerator implements HeightGenerator{
    private EnvBounds envBounds;

    public AllZeroGenerator(EnvBounds envBounds) {
        this.envBounds = envBounds;
    }

    @Override
    public double getGround(double x, double y) {
        return 0;
    }

    @Override
    public double[][] toHeightMap() {
        return new double[0][0];
    }

    @Override
    public Point2D getHeightMapMinPoint() {
        return new Point2D(envBounds.getMinX(), envBounds.getMinY());
    }

    @Override
    public Point2D getHeightMapMaxPoint() {
        return new Point2D(envBounds.getMaxX(), envBounds.getMaxY());
    }

    @Override
    public double getHeightMapDeltaX() {
        return envBounds.getMaxX() - envBounds.getMinX();
    }

    @Override
    public double getHeightMapDeltaY() {
        return envBounds.getMaxY() - envBounds.getMinY();
    }

    @Override
    public void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter) {
        // Empty
    }
}