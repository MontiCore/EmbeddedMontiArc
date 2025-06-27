/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.*;

@JsonType(Type.ARRAY)
public class Mat3 {
    public Vec3 col1;
    public Vec3 col2;
    public Vec3 col3;

    /** Sets all entries to 0. */
    public Mat3() {
        this.col1 = new Vec3();
        this.col2 = new Vec3();
        this.col3 = new Vec3();
    }

    public Mat3(Vec3 col1, Vec3 col2, Vec3 col3) {
        this.col1 = col1.clone();
        this.col2 = col2.clone();
        this.col3 = col3.clone();
    }

    public Mat3(
        double m11, double m12, double m13,
        double m21, double m22, double m23,
        double m31, double m32, double m33
        ) 
    {
        this.col1 = new Vec3(m11, m21, m31);
        this.col2 = new Vec3(m12, m22, m32);
        this.col3 = new Vec3(m13, m23, m33);
    }

    public void set(Vec3 col1, Vec3 col2, Vec3 col3) {
        this.col1.set(col1);
        this.col2.set(col2);
        this.col3.set(col3);
    }

    public void set(
        double m11, double m12, double m13,
        double m21, double m22, double m23,
        double m31, double m32, double m33
        ) 
    {
        this.col1.x = m11;
        this.col1.y = m21;
        this.col1.z = m31;
        this.col2.x = m12;
        this.col2.y = m22;
        this.col2.z = m32;
        this.col3.x = m13;
        this.col3.y = m23;
        this.col3.z = m33;
    }

    public void setZero() {
        this.col1.x = 0;
        this.col1.y = 0;
        this.col1.z = 0;
        this.col2.x = 0;
        this.col2.y = 0;
        this.col2.z = 0;
        this.col3.x = 0;
        this.col3.y = 0;
        this.col3.z = 0;
    }

    public void setUnit(){
        this.col1.x = 1;
        this.col1.y = 0;
        this.col1.z = 0;
        this.col2.x = 0;
        this.col2.y = 1;
        this.col2.z = 0;
        this.col3.x = 0;
        this.col3.y = 0;
        this.col3.z = 1;
    }

    public void setDiagonal(double a, double b, double c){
        this.col1.x = a;
        this.col1.y = 0;
        this.col1.z = 0;
        this.col2.x = 0;
        this.col2.y = b;
        this.col2.z = 0;
        this.col3.x = 0;
        this.col3.y = 0;
        this.col3.z = c;
    }

    public void setDiagonal(Vec3 v){
        this.col1.x = v.x;
        this.col1.y = 0;
        this.col1.z = 0;
        this.col2.x = 0;
        this.col2.y = v.y;
        this.col2.z = 0;
        this.col3.x = 0;
        this.col3.y = 0;
        this.col3.z = v.z;
    }


    public static Mat3 unit() {
        return new Mat3(new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1));
    }

    public static Mat3 fromRows(Vec3 row1, Vec3 row2, Vec3 row3) {
        return new Mat3(new Vec3(row1.x, row2.x, row3.x), new Vec3(row1.y, row2.y, row3.y),
                new Vec3(row1.z, row2.z, row3.z));
    }

    public Vec3 multiply(Vec3 b) {
        return new Vec3(
            col1.x * b.x + col2.x * b.y + col3.x * b.z, 
            col1.y * b.x + col2.y * b.y + col3.y * b.z,
            col1.z * b.x + col2.z * b.y + col3.z * b.z
        );
    }


    public Mat3 multiply(Mat3 m){
        return new Mat3(
            multiply(m.col1),
            multiply(m.col2),
            multiply(m.col3)
        );
    }


    public Mat3 multiply(double a){
        return new Mat3(
            this.col1.multiply(a),
            this.col2.multiply(a),
            this.col3.multiply(a)
        );
    }

    public Mat3 add(Mat3 m){
        return new Mat3(
            this.col1.add(m.col1),
            this.col2.add(m.col2),
            this.col3.add(m.col3)
        );
    }



    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        return equals((Mat3) obj, 0.000001);
    }

    public boolean equals(Mat3 m, double threshold) {
        return this.col1.equals(m.col1, threshold) 
        && this.col2.equals(m.col2, threshold)
        && this.col3.equals(m.col3, threshold);
    }

    public static Mat3 translationMatrix(Vec2 translation) {
        return new Mat3(new Vec3(1,0,0), new Vec3(0,1,0), new Vec3(translation.x, translation.y, 1));
    }

    public static Mat3 rotationMatrix(double angle) {
        return new Mat3(
            new Vec3(Math.cos(angle),Math.sin(angle),0), 
            new Vec3(-Math.sin(angle),Math.cos(angle),0), 
            new Vec3(0, 0, 1)
        );
    }

    public static Mat3 crossMatrix(Vec3 v) {
        return new Mat3(
            new Vec3(0,   v.z,  -v.y), 
            new Vec3(-v.z,  0,   v.x), 
            new Vec3(v.y,  -v.x,  0)
        );
    }
    public static Mat3 scaleMatrix(double scale) {
        return new Mat3(new Vec3(scale,0,0), new Vec3(0,scale,0), new Vec3(0, 0, 1));
    }
    public static Mat3 scaleMatrix(Vec2 scale) {
        return new Mat3(new Vec3(scale.x,0,0), new Vec3(0,scale.y,0), new Vec3(0, 0, 1));
    }
    
    public double at(int row, int collumn){
        if (collumn < 0 || collumn >= 3) throw new IndexOutOfBoundsException("Accessing Mat3 at collumn: " + collumn);
        switch (collumn){
            case 1: return col2.at(row);
            case 2: return col3.at(row);
            default: return col1.at(row);
        }
    }
}