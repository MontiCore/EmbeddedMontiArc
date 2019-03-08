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
package simulation.environment.osm;

import javafx.geometry.Point2D;
import simulation.environment.geometry.height.*;
import simulation.environment.visualisationadapter.implementation.Bounds2D;
import simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D;
import simulation.environment.visualisationadapter.implementation.Node2D;
import simulation.environment.visualisationadapter.interfaces.Building;
import simulation.environment.visualisationadapter.interfaces.EnvBounds;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;

/**
 * Created by lukas on 16.02.17.
 *
 * Generate z-Coordinates for a given point or container
 */
public class ZCoordinateGenerator {
    /**
     * Generate z-Coordinates for container using the Strategy specified in strategy
     * @param container
     * @param strategy
     */

    private static HeightGenerator heightGenerator;

    public static void generateZCoordinates(EnvironmentContainer2D container, ParserSettings.ZCoordinates strategy) {
        if(strategy == ParserSettings.ZCoordinates.ALLZERO) {
            heightGenerator = new AllZeroGenerator(container.getBounds());
        } else if(strategy == ParserSettings.ZCoordinates.STATIC) {
            heightGenerator = new StaticHeightGenerator(container.getBounds());
        } else if (strategy == ParserSettings.ZCoordinates.FROM_FILE) {
            heightGenerator = new SRTMHeightGenerator();
        } else {
            ConcentricCircleGenerator.init(container.getBounds());
            heightGenerator = ConcentricCircleGenerator.getInstance();
        }

        double maxZ = Double.MIN_VALUE;
        double minZ = Double.MAX_VALUE;

        for(EnvStreet s : container.getStreets()) {
            for(EnvNode n : s.getNodes()) {
                Node2D n1 = (Node2D) n;
                n1.setZ(heightGenerator.getGround(n1.getX().doubleValue(), n1.getY().doubleValue()));

                if(n1.getZ().doubleValue() < 0) {
                    System.out.println(n1);
                }

                if(n1.getZ().doubleValue() > maxZ) {
                    maxZ = n1.getZ().doubleValue();
                }

                if(n1.getZ().doubleValue() < minZ) {
                    minZ = n1.getZ().doubleValue();
                }

            }

            for(EnvNode n : s.getIntersections()) {
                Node2D n1 = (Node2D) n;
                n1.setZ(heightGenerator.getGround(n1.getX().doubleValue(), n1.getY().doubleValue()));
                if(n1.getZ().doubleValue() > maxZ) {
                    maxZ = n1.getZ().doubleValue();
                }

                if(n1.getZ().doubleValue() < minZ) {
                    minZ = n1.getZ().doubleValue();
                }
            }
        }

        for(Building b : container.getBuildings()) {
            for(EnvNode n : b.getNodes()) {
                Node2D n1 = (Node2D) n;
                n1.setZ(heightGenerator.getGround(n1.getX().doubleValue(), n1.getY().doubleValue()));

                if(n1.getZ().doubleValue() > maxZ) {
                    maxZ = n1.getZ().doubleValue();
                }

                if(n1.getZ().doubleValue() < minZ) {
                    minZ = n1.getZ().doubleValue();
                }
            }
        }
        EnvBounds oldBounds = container.getBounds();
        container.setBounds(new Bounds2D(oldBounds.getMinX(), oldBounds.getMaxX(), oldBounds.getMinY(), oldBounds.getMaxY(), minZ, maxZ));
    }

    /**
     * @param x
     * @param y
     * @return Ground in the environment for given x and y
     */
    public static double getGround(double x, double y) {
        if(heightGenerator != null) {
            return heightGenerator.getGround(x, y);
        } else {
            //this case should never occur!
            return 0.d;
        }

    }

    public static double[][] getHeightMap() {
        return heightGenerator.toHeightMap();
    }

    public static void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter) {
        if (heightGenerator != null) {
            heightGenerator.setLongLatToMetersConverter(longLatToMeterConverter);
        }
    }

    public static double getHeightMapDeltaX() {
        if (heightGenerator != null) {
            return heightGenerator.getHeightMapDeltaX();
        }

        return 1.0;
    }

    public static double getHeightMapDeltaY() {
        if (heightGenerator != null) {
            return heightGenerator.getHeightMapDeltaY();
        }

        return 1.0;
    }

    public static Point2D getHeightMapMinPoint() {
        if (heightGenerator != null) {
            return heightGenerator.getHeightMapMinPoint();
        }

        return new Point2D(0,0);
    }

    public static Point2D getHeightMapMaxPoint() {
        if (heightGenerator != null) {
            return heightGenerator.getHeightMapMaxPoint();
        }

        return new Point2D(0,0);
    }
}