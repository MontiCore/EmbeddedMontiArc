/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.*;

// TODO add constructors as in Mat3
@JsonType(Type.ARRAY)
public class Mat4 {
    public Vec4 col1;
    public Vec4 col2;
    public Vec4 col3;
    public Vec4 col4;

    /** Sets all entries to 0. */
    public Mat4() {
        this.col1 = new Vec4();
        this.col2 = new Vec4();
        this.col3 = new Vec4();
        this.col4 = new Vec4();
    }

    public Mat4(Vec4 col1, Vec4 col2, Vec4 col3, Vec4 col4) {
        this.col1 = col1.clone();
        this.col2 = col2.clone();
        this.col3 = col3.clone();
        this.col4 = col4.clone();
    }


    public static Mat4 unit() {
        return new Mat4(new Vec4(1, 0, 0, 0), new Vec4(0, 1, 0, 0), new Vec4(0, 0, 1, 0), new Vec4(0, 0, 0, 1));
    }

    public static Mat4 fromRows(Vec4 row1, Vec4 row2, Vec4 row3, Vec4 row4) {
        return new Mat4(new Vec4(row1.x, row2.x, row3.x, row4.x), new Vec4(row1.y, row2.y, row3.y, row4.y),
                new Vec4(row1.z, row2.z, row3.z, row4.z), new Vec4(row1.w, row2.w, row3.w, row4.w));
    }

    /// Performs Matrix Vector Multiplication
    public Vec4 multiply(Vec4 b) {
        return new Vec4(col1.x * b.x + col2.x * b.y + col3.x * b.z + col4.x * b.w,
                col1.y * b.x + col2.y * b.y + col3.y * b.z + col4.y * b.w,
                col1.z * b.x + col2.z * b.y + col3.z * b.z + col4.z * b.w,
                col1.w * b.x + col2.w * b.y + col3.w * b.z + col4.w * b.w);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        return equals((Mat4) obj, 0.000001);
    }

    public boolean equals(Mat4 m, double threshold) {
        return this.col1.equals(m.col1, threshold) 
        && this.col2.equals(m.col2, threshold)
        && this.col3.equals(m.col3, threshold)
        && this.col4.equals(m.col4, threshold);
    }

    public double at(int row, int collumn){
        if (collumn < 0 || collumn >= 4) throw new IndexOutOfBoundsException("Accessing Mat4 at collumn: " + collumn);
        switch (collumn){
            case 1: return col2.at(row);
            case 2: return col3.at(row);
            case 3: return col4.at(row);
            default: return col1.at(row);
        }
    }

}