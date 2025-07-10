/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

/**
 * Performs approximated conversions between Geographic coordinates and a local coordinate system
 * (in meters). It bases the transformation on a reference coordinate that will become the (0,0) point
 * in the local coordinate system.
 * 
 * https://en.wikipedia.org/wiki/Equirectangular_projection
 * https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
 */
public class SimpleCoordinateConverter {
    private static final double DEG_TO_METERS = Coordinates.EARTH_RADIUS * Geometry.DEG_TO_RAD;
    private static final double METERS_TO_DEG = 1 / DEG_TO_METERS;

    private Coordinates ref;
    private double lon_factor;
    private double inv_lon_factor;

    /**
     * @param ref reference for the Equirectangular projection. Corresponds to the (0,0) point in the local coordinate system.
     */
    public SimpleCoordinateConverter(Coordinates ref){
        this.ref = ref;
        lon_factor = Math.cos(ref.lat*Geometry.DEG_TO_RAD) * DEG_TO_METERS;
        inv_lon_factor = 1 / lon_factor;
    }

    public void coordsToMeters(Coordinates coords, Vec2 target){
        target.x = (coords.lon-ref.lon) *lon_factor;
        target.y = (coords.lat-ref.lat) * DEG_TO_METERS;
    }

    public void coordsToMeters(Coordinates coords, Vec3 target){
        target.x = (coords.lon-ref.lon) *lon_factor;
        target.y = (coords.lat-ref.lat) * DEG_TO_METERS;
    }

    public void metersToCoords(Vec2 meters, Coordinates target){
        target.lon = meters.x*inv_lon_factor + ref.lon;
        target.lat = meters.y*METERS_TO_DEG + ref.lat;
    }

    public void metersToCoords(Vec3 meters, Coordinates target){
        target.lon = meters.x*inv_lon_factor + ref.lon;
        target.lat = meters.y*METERS_TO_DEG + ref.lat;
    }
}