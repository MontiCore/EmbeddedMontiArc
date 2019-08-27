/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils;


/**
 * Implementation of the javafx Point3D class.
 * https://docs.oracle.com/javase/8/javafx/api/javafx/geometry/Point3D.html
 */
public class Point3D {
    private double x;
    private double y; 
    private double z;
    public Point3D(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Point3D add(Point3D p){
        return new Point3D(x+p.x, y+p.y, z+p.z);
    }

    public Point3D add(double x, double y, double z){
        return new Point3D(x+this.x, y+this.y, z + this.z);
    }

    public double angle(double x, double y, double z){
        return Math.acos(dotProduct(x,y,z));
    }

    public double angle(Point3D p){
        return Math.acos(dotProduct(p));
    }

    public double angle(Point3D p1, Point3D p2){
        return Math.acos(subtract(p1).dotProduct(subtract(p2)));
    }

    public Point3D crossProduct(double x, double y, double z){
        return new Point3D( this.y * z - this.z * y, this.z * x - this.x * z, this.x * y - this.y * x );
    }

    public Point3D crossProduct(Point3D p){
        return new Point3D( this.y * p.z - this.z * p.y, this.z * p.x - this.x * p.z, this.x * p.y - this.y * p.x );
    }

    public double distance(double x, double y, double z){
        double d_x = x - this.x;
        double d_y = y - this.y;
        double d_z = z - this.z;
        return Math.sqrt(d_x*d_x + d_y*d_y + d_z*d_z);
    }

    public double distance(Point3D p){
        double d_x = p.x - this.x;
        double d_y = p.y - this.y;
        double d_z = p.z - this.z;
        return Math.sqrt(d_x*d_x + d_y*d_y + d_z*d_z);
    }


    public double dotProduct(double x, double y, double z){
        return this.x*x + this.y*y + this.z*z;
    }

    public double dotProduct(Point3D p){
        return this.x*p.x + this.y*p.y + this.z*p.z;
    }

    public boolean equals(Object obj){
        if (obj == null) return false;
        if (obj == this) return true;
        if (!(obj instanceof Point3D)) return false;
        return equals((Point3D) obj);
    }

    public boolean equals(Point3D p){
        return p.x == x && p.y == y && p.z == z;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getZ(){
        return z;
    }

    public int hashCode(){
        return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode() ^ Double.valueOf(z).hashCode();
    }

    public double magnitude(){
        return Math.sqrt(x*x + y*y + z*z);
    }

    public Point3D midpoint(double x, double y, double z){
        return new Point3D((x+this.x)*0.5, (y+this.y)*0.5, (z+this.z)*0.5);
    }

    public Point3D midpoint(Point3D p){
        return new Point3D((p.x+this.x)*0.5, (p.y+this.y)*0.5, (p.z+this.z)*0.5);
    }

    public Point3D multiply(double f){
        return new Point3D(x*f, y*f, z*f);
    }

    

    public Point3D normalize(){
        double length = magnitude();
        double i = 1 / length;
        if (length != 0) return new Point3D(x*i,y*i,z*i);
        return new Point3D(0,0,0);
    }


    public Point3D subtract(Point3D p){
        return new Point3D(x-p.x, y-p.y, z-p.z);
    }

    public Point3D subtract(double x, double y, double z){
        return new Point3D(this.x- x, this.y-y, this.z-z);
    }
    
   


    @Override
    public String toString(){
        return "[" + x + ", " + y + ", " + z + "]";
    }
}
