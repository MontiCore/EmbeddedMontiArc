/* (c) https://github.com/MontiCore/monticore */
/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.commons.utils;

/*
    https://en.wikipedia.org/wiki/Equirectangular_projection
    https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
*/
public class SimpleCoordinateConverter {
    private static final double DEG_TO_METERS = Coordinates.EARTH_RADIUS * Geometry.DEG_TO_RAD;
    private static final double METERS_TO_DEG = 1 / DEG_TO_METERS;

    private Coordinates ref;
    private double lon_factor;
    private double inv_lon_factor;

    /**
     * Performs Geographic Coordinates <-> Meters conversions using an Equirectangular projection.
     * @param ref reference for the Equirectangular projection.
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