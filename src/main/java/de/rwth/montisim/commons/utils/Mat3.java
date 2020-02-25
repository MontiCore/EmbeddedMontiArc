/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.utils;

public class Mat3 {
    public Vec3 col1;
    public Vec3 col2;
    public Vec3 col3;

    /// Creates a 3x3 Matrix from its COLUMN vectors
    public Mat3(Vec3 col1, Vec3 col2, Vec3 col3) {
        this.col1 = col1.clone();
        this.col2 = col2.clone();
        this.col3 = col3.clone();
    }

    /// Creates a 3x3 Matrix with all entries set to 0
    public Mat3() {
        this.col1 = new Vec3();
        this.col2 = new Vec3();
        this.col3 = new Vec3();
    }

    public static Mat3 unit() {
        return new Mat3(new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1));
    }

    public static Mat3 fromRows(Vec3 row1, Vec3 row2, Vec3 row3) {
        return new Mat3(new Vec3(row1.x, row2.x, row3.x), new Vec3(row1.y, row2.y, row3.y),
                new Vec3(row1.z, row2.z, row3.z));
    }

    /// Performs Matrix Vector Multiplication
    public Vec3 multiply(Vec3 b) {
        return new Vec3(col1.x * b.x + col2.x * b.y + col3.x * b.z, col1.y * b.x + col2.y * b.y + col3.y * b.z,
                col1.z * b.x + col2.z * b.y + col3.z * b.z);
    }
}