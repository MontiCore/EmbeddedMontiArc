/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.simulator.visualization.map;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import javax.swing.JMenuItem;

import java.awt.Color;
import java.awt.Graphics2D;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.map.Map;
import de.rwth.montisim.simulation.environment.map.elements.Building;
import de.rwth.montisim.simulation.environment.map.elements.Road;
import de.rwth.montisim.simulation.simulator.visualization.ui.Renderer;
import de.rwth.montisim.simulation.simulator.visualization.ui.CopyMenuItem;

/**
 * Responsible for computing and storing the lines & polygons necessary to
 * visualize a Map.
 */
public class MapRenderer extends Renderer {
    private static final DecimalFormat geoFormat = new DecimalFormat("##0.0000000",
            DecimalFormatSymbols.getInstance(Locale.ENGLISH));
    // Lines => drawPolyline(int[] xPoints, int[] yPoints, int nPoints)
    // Polygons => fillPolygon(Polygon p)
    // Create polygons for roads ? => Filled & Width?
    // Create QuadTree => Prune rendering, allow lookup

    /*
     * Build-up: - Simple lines, no pruning - Polygons - QuadTree => Pruning, lookup
     */

    public static final Color ROAD_COLOR = new Color(130, 130, 130);
    public static final Color AREA_ROAD_COLOR = new Color(180, 180, 180);
    public static final Color BUILDING_COLOR = new Color(197, 209, 214);
    public static final Color BUILDING_COLOR_MAX_HEIGHT = new Color(222, 238, 244);
    public static final float MAX_HEIGHT = 20;

    public final Map map;
    private final List<Polyline> roads = new ArrayList<>();
    private final List<Polygonn> area_roads = new ArrayList<>();
    private final List<Polygonn> buildings = new ArrayList<>();

    // Create the lines & polygons from the map
    public MapRenderer(Map map) {
        this.map = map;
        constructRoads();
        constructBuildings();
    }

    private void constructRoads() {
        for (Road r : map.roads) {
            int size = r.points.size();
            if (r.isArea) {
                if (size >= 3) {
                    Polygonn p = new Polygonn(size, AREA_ROAD_COLOR);
                    int i = 0;
                    for (Vec3 ip : r.points) {
                        p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                        ++i;
                    }
                    area_roads.add(p);
                }
            } else {
                if (size > 0) {
                    Polyline p = new Polyline(size);
                    int i = 0;
                    for (Vec3 ip : r.points) {
                        p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                        ++i;
                    }
                    roads.add(p);
                }
            }
        }
    }

    private void constructBuildings() {
        for (Building b : map.buildings) {
            int size = b.boundary.size();
            if (size >= 3) {
                double h = Math.max(b.height, b.levels * 3);
                Polygonn p = new Polygonn(size, lerp(BUILDING_COLOR, BUILDING_COLOR_MAX_HEIGHT, h / MAX_HEIGHT));
                int i = 0;
                for (Vec2 ip : b.boundary) {
                    p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                    ++i;
                }
                buildings.add(p);
            }
        }
    }

    // Call when screen/view-matrix changed
    @Override
    public void computeGeometry(Mat3 viewMatrix) {
        computeLineGeometry(viewMatrix, roads);
        computePolygonGeometry(viewMatrix, buildings);
        computePolygonGeometry(viewMatrix, area_roads);
    }

    // Call when redraw necessary
    @Override
    public void draw(Graphics2D g) {
        drawLines(g, roads, ROAD_COLOR, 2);
        drawPolygons(g, area_roads);
        drawPolygons(g, buildings);
    }

    @Override
    public String getInfo() {
        return "Map: " + map.name;
    }

    @Override
    public String getHoverInfo(Vec2 worldPos) {
        if (map.converter.isPresent()) {
            Coordinates coords = map.converter.get().metersToCoords(worldPos);
            return "lon: " + geoFormat.format(coords.lon) + " lat: " + geoFormat.format(coords.lat);
        }
        return "";
    }

    @Override
    public JMenuItem getClicMenuItem(Vec2 worldPos) {
        if (!map.converter.isPresent()) return null;
        Coordinates coords = map.converter.get().metersToCoords(worldPos);
        String info = geoFormat.format(coords.lon) + ", " + geoFormat.format(coords.lat);
        return new CopyMenuItem("Copy Coordinates", info, true);
    }


}