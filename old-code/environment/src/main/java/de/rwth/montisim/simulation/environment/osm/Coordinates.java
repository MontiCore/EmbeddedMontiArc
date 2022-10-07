/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */

package de.rwth.montisim.simulation.environment.osm;

import de.rwth.montisim.commons.utils.Vec2;

/**
 * Represents Geographic Coordinates on the earth
 * Expressed as (Longitude, Latitude)
 * Longitude -> Angle along the equator, "Horizontal"
 * Latitude -> Angle away from the equator, "Vertical"
 * The (0,0) Coordinates is the intersection between the Greenwitch meridian and the Equator.
 */
public class Coordinates extends Vec2 {

    public Coordinates(double longitude, double latitude) {
        super(longitude, latitude);
    }

    /// Creates Geographic coordinates from a Vec2 (no conversion), X->Longitude, Y->Latitude
    public Coordinates(Vec2 vec) {
        super(vec.x, vec.y);
    }

    public double getLongitude() {
        return x;
    }

    public double getLatitude() {
        return y;
    }
}