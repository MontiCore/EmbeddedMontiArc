/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.util.List;

import javax.swing.JMenuItem;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;

public abstract class Renderer {

    public boolean dirty = true;
    public abstract void draw(Graphics2D g);
    public abstract void computeGeometry(Mat3 viewMatrix);
    // return "" if nothing to show
    public abstract String getInfo();
    // return "" if nothing to show
    public abstract String getHoverInfo(Vec2 worldPos);
    // Return null if nothing to show
    public abstract JMenuItem getClicMenuItem(Vec2 worldPos);

    
    /**
     * The 'points' array contains the points of a line in *World Space*.
     * The 'x' & 'y' arrays contains the points in *Screen Space* for the current
     * viewMatrix.
     */
    protected static class Polyline{
        public Vec3 points[];
        public int x[];
        public int y[];
        public Polyline(int size){
            points = new Vec3[size];
            x = new int[size];
            y = new int[size];
        }
    }

    
    /**
     * The 'points' array contains the points of a line in *World Space*.
     * The 'x' & 'y' arrays contains the points in *Screen Space* for the current
     * viewMatrix.
     */
    protected static class Polygonn {
        public Vec3 points[];
        public Polygon p;
        public Color color;
        public Polygonn(int size, Color c){
            this.color = c;
            points = new Vec3[size];
            p = new Polygon(new int[size],new int[size],size);
        }
    }

    
    // Helper functions


    protected static void computePolygonGeometry(Mat3 viewMatrix, List<Polygonn> polygons){
        for (Polygonn p : polygons){
            for (int i = 0; i < p.points.length; ++i){
                Vec3 res = viewMatrix.multiply(p.points[i]);
                p.p.xpoints[i] = (int) Math.round(res.x);
                p.p.ypoints[i] = (int) Math.round(res.y);
            }
        }
    }
    protected static void computeLineGeometry(Mat3 viewMatrix, List<Polyline> lines){
        for (Polyline p : lines){
            for (int i = 0; i < p.points.length; ++i){
                Vec3 res = viewMatrix.multiply(p.points[i]);
                p.x[i] = (int) Math.round(res.x);
                p.y[i] = (int) Math.round(res.y);
            }
        }
    }

    protected static void drawPolygons(Graphics2D g, List<Polygonn> polygons){
        for (Polygonn p : polygons){
            g.setColor(p.color);
            g.fill(p.p);
        }
    }

    protected static void drawLines(Graphics2D g, List<Polyline> lines, Color color, int thickness){
        g.setColor(color);
        g.setStroke(new BasicStroke(thickness));
        for (Polyline p : lines){
            g.drawPolyline(p.x,p.y,p.x.length);
        }
    }

    protected Color lerp(Color c1, Color c2, double alpha){
        return new Color(
            clampColor(lerp(c1.getRed(), c2.getRed(), alpha)),
            clampColor(lerp(c1.getGreen(), c2.getGreen(), alpha)),
            clampColor(lerp(c1.getBlue(), c2.getBlue(), alpha))
        );
    }
    protected int lerp(int a, int b, double alpha){
        return (int) ((1-alpha)*a + alpha*b);
    }
    protected int clampColor(int c){
        return c < 0 ? 0 : c > 255 ? 255 : c;
    }
}