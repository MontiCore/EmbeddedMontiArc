/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.environment.geometry.height.*;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.Bounds2D;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.EnvironmentContainer2D;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.Node2D;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.Building;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvBounds;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvStreet;
import de.rwth.montisim.simulation.util.Log;

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
        if (strategy == ParserSettings.ZCoordinates.ALLZERO) {
            heightGenerator = new AllZeroGenerator(container.getBounds());
        } else if (strategy == ParserSettings.ZCoordinates.STATIC) {
            heightGenerator = new StaticHeightGenerator(container.getBounds());
        } else if (strategy == ParserSettings.ZCoordinates.FROM_FILE) {
            heightGenerator = new SRTMHeightGenerator();
        } else {
            ConcentricCircleGenerator.init(container.getBounds());
            heightGenerator = ConcentricCircleGenerator.getInstance();
        }

        double maxZ = Double.MIN_VALUE;
        double minZ = Double.MAX_VALUE;

        for (EnvStreet s : container.getStreets()) {
            for (EnvNode n : s.getNodes()) {
                Node2D n1 = (Node2D) n;
                n1.setZ(heightGenerator.getGround(n1.getX().doubleValue(), n1.getY().doubleValue()));

                if (n1.getZ().doubleValue() < 0) {
                    System.out.println(n1);
                }

                if (n1.getZ().doubleValue() > maxZ) {
                    maxZ = n1.getZ().doubleValue();
                }

                if (n1.getZ().doubleValue() < minZ) {
                    minZ = n1.getZ().doubleValue();
                }

            }

            for (EnvNode n : s.getIntersections()) {
                Node2D n1 = (Node2D) n;
                n1.setZ(heightGenerator.getGround(n1.getX().doubleValue(), n1.getY().doubleValue()));
                if (n1.getZ().doubleValue() > maxZ) {
                    maxZ = n1.getZ().doubleValue();
                }

                if (n1.getZ().doubleValue() < minZ) {
                    minZ = n1.getZ().doubleValue();
                }
            }
        }

        for (Building b : container.getBuildings()) {
            for (EnvNode n : b.getNodes()) {
                Node2D n1 = (Node2D) n;
                n1.setZ(heightGenerator.getGround(n1.getX().doubleValue(), n1.getY().doubleValue()));

                if (n1.getZ().doubleValue() > maxZ) {
                    maxZ = n1.getZ().doubleValue();
                }

                if (n1.getZ().doubleValue() < minZ) {
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
        // This code generates a SRTMHeightGenerator, when no height generator was found.
        // Given x,y coordinates in meters are temporarily converted to lat/long values
        // and getGround() is called. This happens, because the object, created from
        // generateZCoordinates() (according to strategy) gets lost while executing SmartFoxServer.
        if (heightGenerator == null) {
            heightGenerator = new SRTMHeightGenerator();
            Log.info("Created new height generator as no reference existed!");
        }

        double minLat = 50.7767081, minLong = 6.052651;
        double LAT_CONSTANT = 110.574;
        double LONG_CONSTANT = 111.320;
        double lat = y / (1000 * LAT_CONSTANT) + minLat;
        double longi = x / (1000 * LONG_CONSTANT * Math.cos(Math.toRadians(lat))) + minLong;

        if (heightGenerator != null) {
            //Log.info("Height at " + heightGenerator.getGround(x, y));
            double z = 0;
            try {
                z = heightGenerator.getGround(longi, lat);
            } catch (ArrayIndexOutOfBoundsException ex) {
                Log.info("IndexOutOfBounds: x=" + x + "->long=" + longi + ",y=" + y + "->lat=" + lat);
            }
            return z;
        } else {
            //this case should never occur!
            //Log.warning("ZCoordinateGenerator has no height generator reference!");
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

    public static Vec2 getHeightMapMinPoint() {
        if (heightGenerator != null) {
            return heightGenerator.getHeightMapMinPoint();
        }

        return new Vec2(0, 0);
    }

    public static Vec2 getHeightMapMaxPoint() {
        if (heightGenerator != null) {
            return heightGenerator.getHeightMapMaxPoint();
        }

        return new Vec2(0, 0);
    }
}
