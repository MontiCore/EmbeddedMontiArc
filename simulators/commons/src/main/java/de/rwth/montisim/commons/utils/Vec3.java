/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.*;

/**
 * Implementation of the javafx Point3D class.
 * https://docs.oracle.com/javase/8/javafx/api/javafx/geometry/Point3D.html
 */
@JsonType(Type.ARRAY)
public class Vec3 {
    public double x;
    public double y;
    public double z;

    public Vec3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public void set(Vec3 v){
        x = v.x;
        y = v.y;
        z = v.z;
    }

    public Vec3(Vec2 vec, double z) {
        this.x = vec.x;
        this.y = vec.y;
        this.z = z;
    }

    public void set(Vec2 vec, double z) {
        this.x = vec.x;
        this.y = vec.y;
        this.z = z;
    }

    /** Sets all entries to 'a' */
    public Vec3(double a) {
        this.x = a;
        this.y = a;
        this.z = a;
    }

    /** Sets all entries to 'a' */
    public void set(double a) {
        this.x = a;
        this.y = a;
        this.z = a;
    }

    /** Sets all entries to 0. */
    public Vec3() {
        this(0);
    }

    /// Returns a NEW Vec3 with the result
    public Vec3 add(Vec3 p) {
        return new Vec3(x + p.x, y + p.y, z + p.z);
    }

    /// Returns a NEW Vec3 with the result
    public Vec3 add(double x, double y, double z) {
        return new Vec3(x + this.x, y + this.y, z + this.z);
    }

    /// Returns a NEW Vec3 with the result
    public Vec3 subtract(Vec3 p) {
        return new Vec3(x - p.x, y - p.y, z - p.z);
    }

    /// Returns a NEW Vec3 with the result
    public Vec3 subtract(double x, double y, double z) {
        return new Vec3(this.x - x, this.y - y, this.z - z);
    }

    public double angle(double x, double y, double z) {
        return Math.acos(dotProduct(x, y, z));
    }

    public double angle(Vec3 p) {
        return Math.acos(dotProduct(p));
    }

    public double angle(Vec3 p1, Vec3 p2) {
        return Math.acos(subtract(p1).dotProduct(subtract(p2)));
    }

    public Vec3 crossProduct(double x, double y, double z) {
        return new Vec3(this.y * z - this.z * y, this.z * x - this.x * z, this.x * y - this.y * x);
    }

    public Vec3 crossProduct(Vec3 p) {
        return new Vec3(this.y * p.z - this.z * p.y, this.z * p.x - this.x * p.z, this.x * p.y - this.y * p.x);
    }

    public double distance(double x, double y, double z) {
        double d_x = x - this.x;
        double d_y = y - this.y;
        double d_z = z - this.z;
        return Math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
    }

    public double distance(Vec3 p) {
        double d_x = p.x - this.x;
        double d_y = p.y - this.y;
        double d_z = p.z - this.z;
        return Math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z);
    }

    public double dotProduct(double x, double y, double z) {
        return this.x * x + this.y * y + this.z * z;
    }

    public double dotProduct(Vec3 p) {
        return this.x * p.x + this.y * p.y + this.z * p.z;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        return equals((Vec3) obj, 0.000001);
    }

    public boolean equals(Vec3 p, double threshold) {
        return UMath.equalsThreshold(p.x, x, threshold) 
        && UMath.equalsThreshold(p.y, y, threshold)
        && UMath.equalsThreshold(p.z, z, threshold);
    }

    @Override
    public int hashCode() {
        return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode() ^ Double.valueOf(z).hashCode();
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vec3 midpoint(double x, double y, double z) {
        return new Vec3((x + this.x) * 0.5, (y + this.y) * 0.5, (z + this.z) * 0.5);
    }

    public Vec3 midpoint(Vec3 p) {
        return new Vec3((p.x + this.x) * 0.5, (p.y + this.y) * 0.5, (p.z + this.z) * 0.5);
    }

    public Vec3 multiply(double f) {
        return new Vec3(x * f, y * f, z * f);
    }

    public Vec3 normalize() {
        double length = magnitude();
        double i = 1 / length;
        if (length != 0)
            return new Vec3(x * i, y * i, z * i);
        return new Vec3(0, 0, 0);
    }

    public Vec2 asVec2() {
        return new Vec2(this.x, this.y);
    }

    public Vec3 clone() {
        return new Vec3(this.x, this.y, this.z);
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + ", " + z + "]";
    }

    public double at(int index) {
        if (index < 0 || index >= 3) throw new IndexOutOfBoundsException("Accessing Vec3 at index: " + index);
        switch (index) {
            case 1: return y;
            case 2: return z;
            default: return x;
        }
    }

}
