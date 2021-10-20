/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

//import org.apache.commons.math3.linear.RealVector;

/**
 * Geometry related utils.
 */
public final class Geometry {
    public static final double DEG_TO_RAD = 2.0 * Math.PI / 360.0;
    public static final double RAD_TO_DEG = 1 / DEG_TO_RAD;
    public static final double INCH_TO_METERS = 0.0254;
    public static final double FOOT_TO_METERS = 0.3048;

    public static Mat3 rotationAroundX(double angle) {
        return new Mat3(new Vec3(1, 0, 0), new Vec3(0, Math.cos(angle), Math.sin(angle)),
                new Vec3(0, -Math.sin(angle), Math.cos(angle)));
    }

    public static Mat3 rotationAroundY(double angle) {
        return new Mat3(new Vec3(Math.cos(angle), 0, -Math.sin(angle)), new Vec3(0, 1, 0),
                new Vec3(Math.sin(angle), 0, Math.cos(angle)));
    }

    public static Mat3 rotationAroundZ(double angle) {
        return new Mat3(new Vec3(Math.cos(angle), Math.sin(angle), 0), new Vec3(-Math.sin(angle), Math.cos(angle), 0),
                new Vec3(0, 0, 1));
    }

    // public static Vec3 vec3from(RealVector v) {
    //     return new Vec3(v.getEntry(0), v.getEntry(1), v.getEntry(2));
    // }

    // public static Vec2 vec2from(RealVector v) {
    //     return new Vec2(v.getEntry(0), v.getEntry(1));
    // }

}
