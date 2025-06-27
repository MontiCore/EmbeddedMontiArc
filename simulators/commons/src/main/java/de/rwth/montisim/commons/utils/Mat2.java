/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.*;

@JsonType(Type.ARRAY)
public class Mat2 {
    public Vec2 col1;
    public Vec2 col2;

    
    /** Sets all entries to 0. */
    public Mat2() {
        this.col1 = new Vec2();
        this.col2 = new Vec2();
    }
    public Mat2(Vec2 col1, Vec2 col2) {
        this.col1 = col1.clone();
        this.col2 = col2.clone();
    }

    public Mat2(
        double m11, double m12,
        double m21, double m22
        ) 
    {
        this.col1 = new Vec2(m11, m21);
        this.col2 = new Vec2(m12, m22);
    }

    public void set(Vec2 col1, Vec2 col2) {
        this.col1.set(col1);
        this.col2.set(col2);
    }

    public void set(
        double m11, double m12, 
        double m21, double m22
        ) 
    {
        this.col1.x = m11;
        this.col1.y = m21;
        this.col2.x = m12;
        this.col2.y = m22;
    }

    public void setZero() {
        this.col1.x = 0;
        this.col1.y = 0;
        this.col2.x = 0;
        this.col2.y = 0;
    }

    public void setUnit(){
        this.col1.x = 1;
        this.col1.y = 0;
        this.col2.x = 0;
        this.col2.y = 1;
    }

    public void setDiagonal(double a, double b){
        this.col1.x = a;
        this.col1.y = 0;
        this.col2.x = 0;
        this.col2.y = b;
    }

    public void setDiagonal(Vec2 v){
        this.col1.x = v.x;
        this.col1.y = 0;
        this.col2.x = 0;
        this.col2.y = v.y;
    }


    public static Mat2 unit() {
        Mat2 m = new Mat2();
        m.setUnit();
        return m;
    }

    public static Mat2 fromRows(Vec2 row1, Vec2 row2) {
        return new Mat2(row1.x, row1.y, row2.x, row2.y);
    }

    public Vec2 multiply(Vec2 b) {
        Vec2 m = new Vec2();
        IPM.multiply(this, m);
        return m;
    }


    public Mat2 multiply(Mat2 m){
        Mat2 r = new Mat2();
        IPM.multiplyTo(r, this, m);
        return r;
    }


    public Mat2 multiply(double a){
        Mat2 m = new Mat2();
        IPM.multiplyTo(m, this, a);
        return m;
    }

    public Mat2 add(Mat2 a){
        Mat2 m = new Mat2();
        IPM.addTo(m, this, a);
        return m;
    }



    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        return equals((Mat2) obj, 0.000001);
    }

    public boolean equals(Mat2 m, double threshold) {
        return this.col1.equals(m.col1, threshold) 
        && this.col2.equals(m.col2, threshold);
    }

    public double at(int row, int collumn){
        if (collumn < 0 || collumn >= 2) throw new IndexOutOfBoundsException("Accessing Mat2 at collumn: " + collumn);
        switch (collumn){
            case 1: return col2.at(row);
            default: return col1.at(row);
        }
    }

}