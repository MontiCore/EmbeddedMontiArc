/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;


//import org.apache.commons.math3.random.RandomDataGenerator;

/**
 * Math related utility functions: Random numbers, minimization, maximization, ...
 */
public class UMath {
    
    public static final double METERS_PER_SEC_TO_KMH = 3.6;
    public static final double KMH_TO_METERS_PER_SEC = 1/3.6;

    /**
     * Function that generates uniformly distributed random int values in a specified interval
     *
     * @param lower Lower end of the interval, included
     * @param upper Upper end of the interval, included
     * @return Uniformly random int value within the interval
     */
    public static int randomInt(int lower, int upper) {
        if (lower == upper) {
            return lower;
        }
        final int l = Math.min(lower, upper);
        final int u = Math.max(lower, upper)+1;
        return ((int) (Math.random() * (u - l))) + l;
    }

    /// Returns a random int between 0 and upper (included)
    public static int randomInt(int upper){
        return randomInt(0, upper);
    }

    /**
     * Function that generates uniformly distributed random long values in a specified interval
     *
     * @param lower Lower end of the interval, included
     * @param upper Upper end of the interval, included
     * @return Uniformly random long value within the interval
     */
    public static long randomLong(long lower, long upper) {
        if (lower == upper) {
            return lower;
        }
        final long l = Math.min(lower, upper);
        final long u = Math.max(lower, upper)+1;
        return ((long) (Math.random() * (u - l))) + l;
    }

    /// Returns a random long between 0 and upper (included)
    public static long randomLong(long upper){
        return randomLong(0, upper);
    }
    
    /// Random number between 'lower' and 'upper' ('upper' excluded)
    public static double randomDouble(final double lower, final double upper) {
        if (lower == upper) {
            return lower;
        }
        final double l = Math.min(lower, upper);
        final double u = Math.max(lower, upper);
        return Math.random() * (u - l) + l;
    }

    /// Random number between 0 and |upper| (both included)
    public static double randomDouble(final double upper) {
        return randomDouble(0, Math.abs(upper));
    }

    public static Vec3 randomVec3(final double upper) {
        return new Vec3(randomDouble(upper), randomDouble(upper), randomDouble(upper));
    }

    public static Vec2 randomVec2(final double upper) {
        return new Vec2(randomDouble(upper), randomDouble(upper));
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

    /// Checks if two doubles are equal within a threshold (tests if a is in [b-threshold; b+threshold])
    public static boolean equalsThreshold(double a, double b, double threshold){
        return a < b + threshold && a > b - threshold;
    }

    /// Checks if two floats are equal within a threshold (tests if a is in [b-threshold; b+threshold])
    public static boolean equalsThreshold(float a, float b, float threshold){
        return a < b + threshold && a > b - threshold;
    }
}