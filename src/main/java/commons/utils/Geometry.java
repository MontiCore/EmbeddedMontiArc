/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package commons.utils;

import java.util.Random;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import commons.utils.Point2D;
import commons.utils.Point3D;

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
