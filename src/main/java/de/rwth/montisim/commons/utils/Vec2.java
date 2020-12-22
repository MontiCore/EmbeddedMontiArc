/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.*;

/**
 * Implementation of the javafx Point2D class.
 * https://docs.oracle.com/javase/8/javafx/api/javafx/geometry/Point2D.html
 */
@JsonType(Type.ARRAY)
public class Vec2 {
    public double x;
    public double y;

    public Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vec2(Vec3 v) {
        this.x = v.x;
        this.y = v.y;
    }

    /** Sets all entries to 'a' */
    public Vec2(double a) {
        this.x = a;
        this.y = a;
    }

    /** Sets all entries to 0. */
    public Vec2() {
        this(0);
    }

    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void set(Vec2 v){
        this.x = v.x;
        this.y = v.y;
    }
    public void set(Vec3 v){
        this.x = v.x;
        this.y = v.y;
    }
    public void set(Vec4 v){
        this.x = v.x;
        this.y = v.y;
    }

    /// Returns a NEW Vec2 with the result
    public Vec2 add(double x, double y) {
        return new Vec2(x + this.x, y + this.y);
    }

    /// Returns a NEW Vec2 with the result
    public Vec2 add(Vec2 p) {
        return new Vec2(x + p.x, y + p.y);
    }

    public double angle(double x, double y) {
        return Math.acos(dotProduct(x, y));
    }

    public double angle(Vec2 p) {
        return Math.acos(dotProduct(p));
    }

    public double angle(Vec2 p1, Vec2 p2) {
        return Math.acos(subtract(p1).dotProduct(subtract(p2)));
    }

    public Vec3 crossProduct(double x, double y) {
        return new Vec3(0, 0, this.x * y - this.y * x);
    }

    public Vec3 crossProduct(Vec2 p) {
        return new Vec3(0, 0, this.x * p.y - this.y * p.x);
    }

    public double distance(double x, double y) {
        double d_x = x - this.x;
        double d_y = y - this.y;
        return Math.sqrt(d_x * d_x + d_y * d_y);
    }

    public double distance(Vec2 p) {
        double d_x = p.x - this.x;
        double d_y = p.y - this.y;
        return Math.sqrt(d_x * d_x + d_y * d_y);
    }

    public double dotProduct(double x, double y) {
        return this.x * x + this.y * y;
    }

    public double dotProduct(Vec2 p) {
        return this.x * p.x + this.y * p.y;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        return equals((Vec2) obj, 0.000001);
    }

    public boolean equals(Vec2 p, double threshold) {
        return UMath.equalsThreshold(p.x, x, threshold) && UMath.equalsThreshold(p.y, y, threshold);
    }

    @Override
    public int hashCode() {
        return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode();
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Vec2 midpoint(double x, double y) {
        return new Vec2((x + this.x) * 0.5, (y + this.y) * 0.5);
    }

    public Vec2 midpoint(Vec2 p) {
        return new Vec2((p.x + this.x) * 0.5, (p.y + this.y) * 0.5);
    }

    /// Returns a NEW Vec2 with the result
    public Vec2 multiply(double f) {
        return new Vec2(x * f, y * f);
    }

    /// Returns a NEW Vec2 with the result
    public Vec2 normalize() {
        double length = magnitude();
        double i = 1 / length;
        if (length != 0)
            return new Vec2(x * i, y * i);
        return new Vec2(0, 0);
    }

    /// Returns a NEW Vec2 with the result
    public Vec2 subtract(Vec2 p) {
        return new Vec2(x - p.x, y - p.y);
    }

    /// Returns a NEW Vec2 with the result
    public Vec2 subtract(double x, double y) {
        return new Vec2(this.x - x, this.y - y);
    }

    public Vec2 clone() {
        return new Vec2(this.x, this.y);
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + "]";
    }

    public double at(int index) {
        if (index < 0 || index >= 2) throw new IndexOutOfBoundsException("Accessing Vec2 at index: " + index);
        switch (index) {
            case 1: return y;
            default: return x;
        }
    }

}
