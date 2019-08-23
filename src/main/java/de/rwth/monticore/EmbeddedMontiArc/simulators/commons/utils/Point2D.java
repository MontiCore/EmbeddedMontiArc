/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils;


/**
 * Implementation of the javafx Point2D class.
 * https://docs.oracle.com/javase/8/javafx/api/javafx/geometry/Point2D.html
 */
public class Point2D {
    private double x;
    private double y;
    public Point2D(double x, double y){
        this.x = x;
        this.y = y;
    }


    public Point2D add(double x, double y){
        return new Point2D(x+this.x, y+this.y);
    }

    public Point2D add(Point2D p){
        return new Point2D(x+p.x, y+p.y);
    }

    public double angle(double x, double y){
        return Math.acos(dotProduct(x,y));
    }

    public double angle(Point2D p){
        return Math.acos(dotProduct(p));
    }

    public double angle(Point2D p1, Point2D p2){
        return Math.acos(subtract(p1).dotProduct(subtract(p2)));
    }

    public Point3D crossProduct(double x, double y){
        return new Point3D( 0, 0, this.x * y - this.y * x );
    }

    public Point3D crossProduct(Point2D p){
        return new Point3D( 0, 0, this.x * p.y - this.y * p.x );
    }

    public double distance(double x, double y){
        double d_x = x - this.x;
        double d_y = y - this.y;
        return Math.sqrt(d_x*d_x + d_y*d_y);
    }

    public double distance(Point2D p){
        double d_x = p.x - this.x;
        double d_y = p.y - this.y;
        return Math.sqrt(d_x*d_x + d_y*d_y);
    }


    
    public double dotProduct(double x, double y){
        return this.x*x + this.y*y;
    }

    public double dotProduct(Point2D p){
        return this.x*p.x + this.y*p.y;
    }

    public boolean equals(Object obj){
        if (obj == null) return false;
        if (obj == this) return true;
        if (!(obj instanceof Point2D)) return false;
        return equals((Point2D) obj);
    }

    public boolean equals(Point2D p){
        return p.x == x && p.y == y;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }

    public int hashCode(){
        return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode();
    }

    public double magnitude(){
        return Math.sqrt(x*x + y*y);
    }

    public Point2D midpoint(double x, double y){
        return new Point2D((x+this.x)*0.5, (y+this.y)*0.5);
    }

    public Point2D midpoint(Point2D p){
        return new Point2D((p.x+this.x)*0.5, (p.y+this.y)*0.5);
    }

    public Point2D multiply(double f){
        return new Point2D(x*f, y*f);
    }

    

    public Point2D normalize(){
        double length = magnitude();
        double i = 1 / length;
        if (length != 0) return new Point2D(x*i,y*i);
        return new Point2D(0,0);
    }


    public Point2D subtract(Point2D p){
        return new Point2D(x-p.x, y-p.y);
    }

    public Point2D subtract(double x, double y){
        return new Point2D(this.x- x, this.y-y);
    }
    
   

    @Override
    public String toString(){
        return "[" + x + ", " + y + "]";
    }
}
