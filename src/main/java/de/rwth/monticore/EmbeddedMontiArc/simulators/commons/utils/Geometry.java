/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils;

import java.util.Random;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;

/**
 * Geometry related utils.
 */
public final class Geometry {

    private static final Random random = new Random();

    private Geometry() {
    }

    public static double getRandom(final double upper) {
        final double u = Math.abs(upper);
        return getRandomBetween(0, u);
    }

    public static double getRandomBetween(final double lower, final double upper) {
        final double l = Math.min(lower, upper);
        final double u = Math.max(lower, upper);
        return l + random.nextDouble() * (u - l);
    }

    public static Point3D getRandomPoint3D(final double upper) {
        final double x = getRandom(upper);
        final double y = getRandom(upper);
        final double z = getRandom(upper);
        return new Point3D(x, y, z);
    }

    public static Point2D getRandomPoint2D(final double upper) {
        final double x = getRandom(upper);
        final double y = getRandom(upper);
        return new Point2D(x, y);
    }

    public static Point3D realVector2Point3D(RealVector v) {
        double x = v.getEntry(0);
        double y = v.getEntry(1);
        double z = v.getEntry(2);
        return new Point3D(x, y, z);
    }

    public static RealVector point3D2RealVector(Point3D p) {
        double x = p.getX();
        double y = p.getY();
        double z = p.getZ();
        return new ArrayRealVector(new double[]{x, y, z});
    }

    public static Point2D realVector2Point2D(RealVector v) {
        double x = v.getEntry(0);
        double y = v.getEntry(1);
        return new Point2D(x, y);
    }

    public static RealVector point2D2RealVector(Point2D p) {
        double x = p.getX();
        double y = p.getY();
        return new ArrayRealVector(new double[]{x, y, 0});
    }

}
