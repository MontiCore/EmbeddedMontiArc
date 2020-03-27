/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.map.visualization;

import java.util.ArrayList;
import java.util.List;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.map.Map;
import de.rwth.montisim.simulation.environment.map.elements.Building;
import de.rwth.montisim.simulation.environment.map.elements.Road;

/**
 * Responsible for computing and storing the lines & polygons 
 * necessary to visualize a Map.
 */
public class MapRenderer {
    // Lines => drawPolyline(int[] xPoints, int[] yPoints, int nPoints) 
    // Polygons => fillPolygon(Polygon p) 
    // Create polygons for roads ? => Filled & Width?
    // Create QuadTree => Prune rendering, allow lookup

    /*
        Build-up:
        - Simple lines, no pruning
        - Polygons
        - QuadTree => Pruning, lookup
    */

    public static final Color ROAD_COLOR = new Color(130,130,130);
    public static final Color AREA_ROAD_COLOR = new Color(180,180,180);
    public static final Color BUILDING_COLOR = new Color(197,209,214);
    public static final Color BUILDING_COLOR_MAX_HEIGHT = new Color(222,238,244);
    public static final float MAX_HEIGHT = 20;

    /**
     * The 'points' array contains the points of a line in *World Space*.
     * The 'x' & 'y' arrays contains the points in *Screen Space* for the current
     * viewMatrix.
     */
    private static class Polyline{
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
    private static class Polygonn {
        public Vec3 points[];
        public Polygon p;
        public Color color;
        public Polygonn(int size, Color c){
            this.color = c;
            points = new Vec3[size];
            p = new Polygon(new int[size],new int[size],size);
        }
    }
    public final Map map;
    private final List<Polyline> roads = new ArrayList<>();
    private final List<Polygonn> area_roads = new ArrayList<>();
    private final List<Polygonn> buildings = new ArrayList<>();

    // Create the lines & polygons from the map
    public MapRenderer(Map map){
        this.map = map;
        constructRoads();
        constructBuildings();
    }

    private void constructRoads(){
        for (Road r : map.roads){
            int size = r.points.size();
            if (r.isArea){
                if (size >=3){
                    Polygonn p = new Polygonn(size, AREA_ROAD_COLOR);
                    int i = 0;
                    for (Vec3 ip : r.points){
                        p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                        ++i;
                    }
                    area_roads.add(p);
                }
            } else {
                if (size > 0){
                    Polyline p = new Polyline(size);
                    int i = 0;
                    for (Vec3 ip : r.points){
                        p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                        ++i;
                    }
                    roads.add(p);
                }
            }
        }
    }

    private void constructBuildings(){
        for (Building b : map.buildings){
            int size = b.boundary.size();
            if (size >=3){
                double h = Math.max(b.height, b.levels*3);
                Polygonn p = new Polygonn(size, lerp(BUILDING_COLOR, BUILDING_COLOR_MAX_HEIGHT, h / MAX_HEIGHT));
                int i = 0;
                for (Vec2 ip : b.boundary){
                    p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                    ++i;
                }
                buildings.add(p);
            }
        }
    }

    // Call when screen/view-matrix changed
    public void computeGeometry(Mat3 viewMatrix){
        computeLineGeometry(viewMatrix, roads);
        computePolygonGeometry(viewMatrix, buildings);
        computePolygonGeometry(viewMatrix, area_roads);
    }

    // Call when redraw necessary
    public void draw(Graphics g){
        Graphics2D g2 = (Graphics2D)g;
        drawLines(g2, roads, ROAD_COLOR, 2);
        drawPolygons(g2, area_roads);
        drawPolygons(g2, buildings);
    }


    // Helper functions


    private static void computePolygonGeometry(Mat3 viewMatrix, List<Polygonn> polygons){
        for (Polygonn p : polygons){
            for (int i = 0; i < p.points.length; ++i){
                Vec3 res = viewMatrix.multiply(p.points[i]);
                p.p.xpoints[i] = (int) Math.round(res.x);
                p.p.ypoints[i] = (int) Math.round(res.y);
            }
        }
    }
    private static void computeLineGeometry(Mat3 viewMatrix, List<Polyline> lines){
        for (Polyline p : lines){
            for (int i = 0; i < p.points.length; ++i){
                Vec3 res = viewMatrix.multiply(p.points[i]);
                p.x[i] = (int) Math.round(res.x);
                p.y[i] = (int) Math.round(res.y);
            }
        }
    }

    private static void drawPolygons(Graphics2D g, List<Polygonn> polygons){
        for (Polygonn p : polygons){
            g.setColor(p.color);
            g.fill(p.p);
        }
    }

    private static void drawLines(Graphics2D g, List<Polyline> lines, Color color, int thickness){
        g.setColor(color);
        g.setStroke(new BasicStroke(thickness));
        for (Polyline p : lines){
            g.drawPolyline(p.x,p.y,p.x.length);
        }
    }

    private Color lerp(Color c1, Color c2, double alpha){
        return new Color(
            clampColor(lerp(c1.getRed(), c2.getRed(), alpha)),
            clampColor(lerp(c1.getGreen(), c2.getGreen(), alpha)),
            clampColor(lerp(c1.getBlue(), c2.getBlue(), alpha))
        );
    }
    private int lerp(int a, int b, double alpha){
        return (int) ((1-alpha)*a + alpha*b);
    }
    private int clampColor(int c){
        return c < 0 ? 0 : c > 255 ? 255 : c;
    }
}