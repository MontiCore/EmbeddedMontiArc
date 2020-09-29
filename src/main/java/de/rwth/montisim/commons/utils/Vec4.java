/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import de.rwth.montisim.commons.utils.json.*;

@JsonType(Type.ARRAY)
public class Vec4 {
    public double x;
    public double y;
    public double z;
    public double w;

    public Vec4(double x, double y, double z, double w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public Vec4(Vec3 vec, double w) {
        this.x = vec.x;
        this.y = vec.y;
        this.z = vec.z;
        this.w = w;
    }

    /// Creates a vector where all entries are 'a'
    public Vec4(double a) {
        this.x = a;
        this.y = a;
        this.z = a;
        this.w = a;
    }

    /// Creates a vector with all entries set to 0
    public Vec4() {
        this(0);
    }

    /// Returns a NEW Vec4 with the result
    public Vec4 add(Vec4 p) {
        return new Vec4(x + p.x, y + p.y, z + p.z, w + p.w);
    }

    /// Returns a NEW Vec4 with the result
    public Vec4 add(double x, double y, double z, double w) {
        return new Vec4(x + this.x, y + this.y, z + this.z, w + this.w);
    }

    public double angle(double x, double y, double z, double w) {
        return Math.acos(dotProduct(x, y, z, w));
    }

    public double angle(Vec4 p) {
        return Math.acos(dotProduct(p));
    }

    public double angle(Vec4 p1, Vec4 p2) {
        return Math.acos(subtract(p1).dotProduct(subtract(p2)));
    }

    public double distance(double x, double y, double z, double w) {
        double d_x = x - this.x;
        double d_y = y - this.y;
        double d_z = z - this.z;
        double d_w = w - this.w;
        return Math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z + d_w * d_w);
    }

    public double distance(Vec4 p) {
        double d_x = p.x - this.x;
        double d_y = p.y - this.y;
        double d_z = p.z - this.z;
        double d_w = p.w - this.w;
        return Math.sqrt(d_x * d_x + d_y * d_y + d_z * d_z + d_w * d_w);
    }

    public double dotProduct(double x, double y, double z, double w) {
        return this.x * x + this.y * y + this.z * z + this.w * w;
    }

    public double dotProduct(Vec4 p) {
        return this.x * p.x + this.y * p.y + this.z * p.z + this.w * p.w;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null)
            return false;
        if (obj == this)
            return true;
        if (this.getClass() != obj.getClass())
            return false;
        return equals((Vec4) obj, 0.000001);
    }

    public boolean equals(Vec4 p, double threshold) {
        return UMath.equalsThreshold(p.x, x, threshold) 
        && UMath.equalsThreshold(p.y, y, threshold)
        && UMath.equalsThreshold(p.z, z, threshold)
        && UMath.equalsThreshold(p.w, w, threshold);
    }

    @Override
    public int hashCode() {
        return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode() ^ Double.valueOf(z).hashCode()
                ^ Double.valueOf(w).hashCode();
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z + w * w);
    }

    public Vec4 midpoint(double x, double y, double z, double w) {
        return new Vec4((x + this.x) * 0.5, (y + this.y) * 0.5, (z + this.z) * 0.5, (w + this.w) * 0.5);
    }

    public Vec4 midpoint(Vec4 p) {
        return new Vec4((p.x + this.x) * 0.5, (p.y + this.y) * 0.5, (p.z + this.z) * 0.5, (p.w + this.w) * 0.5);
    }

    public Vec4 multiply(double f) {
        return new Vec4(x * f, y * f, z * f, w * f);
    }

    public Vec4 normalize() {
        double length = magnitude();
        double i = 1 / length;
        if (length != 0)
            return new Vec4(x * i, y * i, z * i, w * i);
        return new Vec4(0, 0, 0, 0);
    }

    /// Returns a NEW Vec4 with the result
    public Vec4 subtract(Vec4 p) {
        return new Vec4(x - p.x, y - p.y, z - p.z, w - p.w);
    }

    /// Returns a NEW Vec4 with the result
    public Vec4 subtract(double x, double y, double z, double w) {
        return new Vec4(this.x - x, this.y - y, this.z - z, this.w - w);
    }

    public Vec4 clone() {
        return new Vec4(this.x, this.y, this.z, this.w);
    }

    public Vec3 asVec3() {
        return new Vec3(this.x, this.y, this.z);
    }

    public Vec2 asVec2() {
        return new Vec2(this.x, this.y);
    }

    @Override
    public String toString() {
        return "[" + x + ", " + y + ", " + z + ", " + w + "]";
    }

    public double at(int index) {
        if (index < 0 || index >= 4) throw new IndexOutOfBoundsException("Accessing Vec4 at index: " + index);
        switch (index) {
            case 1: return y;
            case 2: return z;
            case 3: return w;
            default: return x;
        }
    }

}
