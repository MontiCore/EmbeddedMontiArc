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
import simulation.environment.visualisationadapter.interfaces.EnvBounds;

/**
 * Created by lukas on 23.02.17.
 *
 * uses fixed slopes. This functionality has to be removed from the ConcentricCircleGenerator and put in here
 */
public class StaticHeightGenerator implements HeightGenerator {

    private ConcentricCircleGenerator generator;

    public StaticHeightGenerator(EnvBounds bounds) {
        ConcentricCircleGenerator.init(bounds,true);
        this.generator = ConcentricCircleGenerator.getInstance();
    }

    @Override
    public double getGround(double x, double y) {
        return generator.getGround(x,y);
    }

    @Override
    public double[][] toHeightMap() {
        return this.generator.toHeightMap();
    }

    @Override
    public Point2D getHeightMapMinPoint() {
        return new Point2D(0,0);
    }

    @Override
    public Point2D getHeightMapMaxPoint() {
        return new Point2D(2,0);
    }

    @Override
    public double getHeightMapDeltaX() {
        return 1.0;
    }

    @Override
    public double getHeightMapDeltaY() {
        return 1.0;
    }

    @Override
    public void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter) {
        // Empty
    }
}