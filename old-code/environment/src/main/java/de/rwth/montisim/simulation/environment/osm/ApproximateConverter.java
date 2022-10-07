/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import de.rwth.montisim.commons.utils.Vec2;

/**
 * An ApproxiamteConverter which converts
 * Longitude and Latitude to Kilometric units (basically manhatten distance) For
 * details on the math see: http://stackoverflow.com/a/1253545/2451431
 */
public class ApproximateConverter implements MetricConverter {
    public Coordinates minCorner;

    private final double LAT_CONSTANT = 110.574;
    private final double LONG_CONSTANT = 111.320;

    public ApproximateConverter(Coordinates minCorner) {
        this.minCorner = minCorner;
    }

    @Override
    public Vec2 coordinatesToMeters(Coordinates coordinates) {
        Vec2 relativeCoords = coordinates.subtract(minCorner);
        return new Vec2(
                1000 * relativeCoords.x * (LONG_CONSTANT * Math.cos(Math.toRadians(coordinates.getLatitude()))),
                1000 * relativeCoords.y * LAT_CONSTANT
        );
    }

    @Override
    public Coordinates metersToCoordinates(Vec2 position) {
        double lat = position.y / (1000 * LAT_CONSTANT);
        return new Coordinates(
                new Vec2(position.x / (1000 * LONG_CONSTANT * Math.cos(Math.toRadians(lat))), lat)
                        .add(minCorner)
        );
    }

}
