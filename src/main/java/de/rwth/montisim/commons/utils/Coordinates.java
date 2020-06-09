/* (c) https://github.com/MontiCore/monticore */

package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.Coordinates;

/**
 * Represents Geographic Coordinates on the earth.
 * Expressed as (Longitude, Latitude).
 * Longitude -> Angle along the equator, "Horizontal".
 * Latitude -> Angle away from the equator, "Vertical".
 * The (0,0) Coordinates is the intersection between the Greenwitch meridian and the Equator.
 */
public class Coordinates {
    public static final double EARTH_RADIUS = 12742000*0.5; //In meters
    /** Longitude */
    public double lon;
    /** Latitude */
    public double lat;

    public Coordinates(double lon, double lat) {
        this.lon = lon;
        this.lat = lat;
    }

    /** Creates the (0;0) Coordinates */
    public Coordinates() {
        this(0,0);
    }

    /** Returns NEW Coordinates with the result */
    public Coordinates add(double lon, double lat) {
        return new Coordinates(lon + this.lon, lat + this.lat);
    }

    /** Returns NEW Coordinates with the result */
    public Coordinates add(Coordinates p) {
        return new Coordinates(this.lon + p.lon, this.lat + p.lat);
    }
    /**
     * Use the https://en.wikipedia.org/wiki/Haversine_formula
     */
    public double distance(double lon, double lat) {
        double sinlat = Math.sin((lat-this.lat)*0.5);
        double sinlon = Math.sin((lon-this.lon)*0.5);
        double h = sinlat*sinlat+Math.cos(this.lat)*Math.cos(lat)*sinlon*sinlon;
        return 2*EARTH_RADIUS*Math.asin(Math.sqrt(h));
    }

    /**
     * Returns the distance between these coordinates and the parameter.
     * Uses the https://en.wikipedia.org/wiki/Haversine_formula
     */
    public double distance(Coordinates p) {
        return distance(p.lon, p.lat);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        // One Meter ~= 0.00001
        return equals((Coordinates) obj, 0.000001);
    }

    public boolean equals(Coordinates p, double threshold) {
        return UMath.equalsThreshold(p.lon, this.lon, threshold) && UMath.equalsThreshold(p.lat, this.lat, threshold);
    }

    @Override
    public int hashCode() {
        return Double.valueOf(this.lon).hashCode() ^ Double.valueOf(this.lat).hashCode();
    }

    public Coordinates midpoint(double lon, double lat) {
        return new Coordinates((this.lon + this.lon) * 0.5, (this.lat + this.lat) * 0.5);
    }

    public Coordinates midpoint(Coordinates p) {
        return new Coordinates((p.lon + this.lon) * 0.5, (p.lat + this.lat) * 0.5);
    }

    /** Returns NEW Coordinates with the result */
    public Coordinates subtract(Coordinates p) {
        return new Coordinates(this.lon - p.lon, this.lat - p.lat);
    }

    /** Returns NEW Coordinates with the result */
    public Coordinates subtract(double lon, double lat) {
        return new Coordinates(this.lon - lon, this.lat - lat);
    }

    public Coordinates clone() {
        return new Coordinates(this.lon, this.lat);
    }

    @Override
    public String toString() {
        return "[lon: " + this.lon + ", lat: " + this.lat + "]";
    }
}