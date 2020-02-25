/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.utils;

import java.util.Random;
import org.apache.commons.math3.linear.RealVector;

/**
 * Geometry related utils.
 */
public final class Geometry {

    private static final Random random = new Random();

    private Geometry() {
    }

    /// Random number between 0 and |upper| (both included)
    public static double random(final double upper) {
        return randomBetween(0, Math.abs(upper));
    }

    /// Random number between 'lower' and 'upper' (both included)
    public static double randomBetween(final double lower, final double upper) {
        final double l = Math.min(lower, upper);
        final double u = Math.max(lower, upper);
        return l + random.nextDouble() * (u - l);
    }

    public static Vec3 randomVec3(final double upper) {
        return new Vec3(random(upper), random(upper), random(upper));
    }

    public static Vec2 randomVec2(final double upper) {
        return new Vec2(random(upper), random(upper));
    }

    public static Vec3 vec3from(RealVector v) {
        return new Vec3(v.getEntry(0), v.getEntry(1), v.getEntry(2));
    }

    public static Vec2 vec2from(RealVector v) {
        return new Vec2(v.getEntry(0), v.getEntry(1));
    }

    /// Maximizes every component of target depending on value: If a component of
    /// value is bigger than that of target, then target is set to it.
    public static void maximize(Vec3 target, Vec3 value) {
        if (value.x > target.x)
            target.x = value.x;
        if (value.y > target.y)
            target.y = value.y;
        if (value.z > target.z)
            target.z = value.z;
    }

    /// Minimizes every component of target depending on value: If a component of
    /// value is smaller than that of target, then target is set to it.
    public static void minimize(Vec3 target, Vec3 value) {
        if (value.x < target.x)
            target.x = value.x;
        if (value.y < target.y)
            target.y = value.y;
        if (value.z < target.z)
            target.z = value.z;
    }

    /// Maximizes every component of target depending on value: If a component of
    /// value is bigger than that of target, then target is set to it.
    public static void maximize(Vec2 target, Vec2 value) {
        if (value.x > target.x)
            target.x = value.x;
        if (value.y > target.y)
            target.y = value.y;
    }

    /// Minimizes every component of target depending on value: If a component of
    /// value is smaller than that of target, then target is set to it.
    public static void minimize(Vec2 target, Vec2 value) {
        if (value.x < target.x)
            target.x = value.x;
        if (value.y < target.y)
            target.y = value.y;
    }

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

    public static boolean equalsThreshold(double a, double b, double threshold){
        return a < b + threshold && a > b - threshold;
    }

}
