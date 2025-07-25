/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.map;

import javax.swing.JMenuItem;

import java.awt.*;
import java.text.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.PolygonConvexer.Edge;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.*;

/**
 * Responsible for computing and storing the lines & polygons necessary to
 * visualize a Map.
 */
public class WorldRenderer extends Renderer {
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
    public static final BasicStroke ROAD_STROKE = new BasicStroke(2);
    public static final Color AREA_ROAD_COLOR = new Color(180, 180, 180);
    public static final Color BUILDING_COLOR = new Color(197, 209, 214);
    public static final Color BUILDING_COLOR_MAX_HEIGHT = new Color(222, 238, 244);
    public static final float MAX_HEIGHT = 20;
    public static final float SEGMENT_OFFSET = -0.2f; // Positive is to the left in segment direction
    public static final Color SEPARATING_LINE_COLOR = new Color(99, 150, 226);
    public static final Color BUILDING_OUTLINE_COLOR = new Color(94, 106, 124);


    public World world;
    private final List<Polyline> roads = new ArrayList<>();
    private boolean segmentsLoaded = false;
    private final List<Polyline> roadSegments = new ArrayList<>();
    private final List<Polygonn> area_roads = new ArrayList<>();
    private final List<Polygonn> buildings = new ArrayList<>();
    private boolean buildingDebugLoaded = false;
    private final List<Polyline> buildingOutlines = new ArrayList<>();
    private final List<Polyline> buildingSeparatingEdges = new ArrayList<>();
    private boolean buildingAABBsLoaded = false;
    private final List<Polyline> buildingAABBs = new ArrayList<>();

    final Coordinates coords = new Coordinates();

    // Create the lines & polygons from the world
    public WorldRenderer(World world) {
        load(world);
    }

    public void load(World world) {
        this.world = world;
        constructRoads();
        constructBuildings();
    }

    private void constructRoads() {
        for (Way r : world.ways) {
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
                if (size > 1) {
                    Polyline p = new Polyline(size, ROAD_COLOR, ROAD_STROKE);
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

    private void constructSegments() {
        ThreadLocalRandom r = ThreadLocalRandom.current();
        Vector<Vec2> normals = new Vector<>(); // Working array
        Vec2 displacement = new Vec2();
        for (WaySegment s : world.waySegments) {
            int size = Math.abs(s.pointsEnd - s.pointsStart) + 1;
            if (size > 1) {
                int inc = s.pointsEnd > s.pointsStart ? 1 : -1;
                Polyline p = new Polyline(size, new Color(r.nextInt(50, 230), r.nextInt(50, 230), r.nextInt(50, 230)));
                roadSegments.add(p);

                if (normals.size() < size - 1) { // Resize working array
                    normals.setSize(size - 1);
                    for (int i = 0; i < size - 1; ++i) if (normals.elementAt(i) == null) normals.set(i, new Vec2());
                }

                Vec3 lastPoint = null;
                Vec3 currentPoint;

                // Get segment vectors -> normalize -> rotate +90 deg -> save in 'normals'
                int j = 0;
                for (int i = s.pointsStart; i != s.pointsEnd + inc; i += inc) {
                    currentPoint = s.way.points.elementAt(i);
                    if (lastPoint != null) {
                        Vec2 n = normals.elementAt(j);
                        // In place subtraction & conversion
                        n.x = currentPoint.x - lastPoint.x;
                        n.y = currentPoint.y - lastPoint.y;
                        IPM.normalize(n);
                        IPM.multiply(n, SEGMENT_OFFSET);
                        double t = n.x;
                        n.x = -n.y;
                        n.y = t;
                        ++j;
                    }
                    lastPoint = currentPoint;
                }

                j = 0;
                for (int i = s.pointsStart; i != s.pointsEnd + inc; i += inc) {
                    Vec3 pi = s.way.points.elementAt(i);
                    //Get displacement
                    if (i == s.pointsStart) displacement.set(normals.elementAt(j));
                    else if (i == s.pointsEnd) displacement.set(normals.elementAt(j - 1));
                    else IPM.midpointTo(displacement, normals.elementAt(j), normals.elementAt(j - 1));
                    p.points[j] = new Vec3(pi.x + displacement.x, pi.y + displacement.y, 1); // Transform to extended 2D coordinates
                    j++;
                }
            }
        }
        segmentsLoaded = true;
    }

    private void constructBuildings() {
        for (Building b : world.buildings) {
            int size = b.boundary.size();
            if (size >= 3) {
                double h = Math.max(b.height, b.levels * 3);
                Polygonn p = new Polygonn(size, lerp(BUILDING_COLOR, BUILDING_COLOR_MAX_HEIGHT, h / MAX_HEIGHT));
                int i = 0;
                for (Vec3 ip : b.boundary) {
                    p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                    ++i;
                }
                buildings.add(p);
            }
        }
    }

    private void constructBuildingDebug() {
        for (Building b : world.buildings) {
            for (Edge e : b.separatingEdges) {
                Polyline l = new Polyline(2, SEPARATING_LINE_COLOR);
                l.points[0] = new Vec3(e.p1.x, e.p1.y, 1.0);
                l.points[1] = new Vec3(e.p2.x, e.p2.y, 1.0);
                buildingSeparatingEdges.add(l);
            }

            int size = b.boundary.size();
            if (size >= 3) {
                double h = Math.max(b.height, b.levels * 3);
                Polyline p = new Polyline(size + 1, lerp(BUILDING_OUTLINE_COLOR, BUILDING_COLOR_MAX_HEIGHT, h / MAX_HEIGHT));
                int i = 0;
                for (Vec3 ip : b.boundary) {
                    p.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                    ++i;
                }
                Vec3 ip = b.boundary.get(0);
                p.points[i] = new Vec3(ip.x, ip.y, 1);
                buildingOutlines.add(p);
            }
        }
        buildingDebugLoaded = true;
    }

    private void constructBuildingAABBs() {
        for (Building b : world.buildings) {
            Polyline aabb = new Polyline(5, UIInfo.AABB_COLOR);
            aabb.points[0] = new Vec3(b.buildingAABB.min.x, b.buildingAABB.min.y, 1.0);
            aabb.points[1] = new Vec3(b.buildingAABB.max.x, b.buildingAABB.min.y, 1.0);
            aabb.points[2] = new Vec3(b.buildingAABB.max.x, b.buildingAABB.max.y, 1.0);
            aabb.points[3] = new Vec3(b.buildingAABB.min.x, b.buildingAABB.max.y, 1.0);
            aabb.points[4] = new Vec3(b.buildingAABB.min.x, b.buildingAABB.min.y, 1.0);
            buildingAABBs.add(aabb);
        }
        buildingAABBsLoaded = true;
    }

    // Call when screen/view-matrix changed
    @Override
    public void computeGeometry(Mat3 viewMatrix) {
        computeLineGeometry(viewMatrix, roads);
        if (UIInfo.showSegments) {
            if (!segmentsLoaded) constructSegments();
            computeLineGeometry(viewMatrix, roadSegments);
        }
        computePolygonGeometry(viewMatrix, buildings);
        computePolygonGeometry(viewMatrix, area_roads);
        if (UIInfo.showBuildingDebug) {
            if (!buildingDebugLoaded) constructBuildingDebug();
            computeLineGeometry(viewMatrix, buildingOutlines);
            computeLineGeometry(viewMatrix, buildingSeparatingEdges);
        }
        if (UIInfo.showAABBs) {
            if (!buildingAABBsLoaded) constructBuildingAABBs();
            computeLineGeometry(viewMatrix, buildingAABBs);
        }
    }

    // Call when redraw necessary
    @Override
    public void draw(Graphics2D g) {
        drawLines(g, roads);
        drawPolygons(g, area_roads);
        drawPolygons(g, buildings);
        if (UIInfo.showSegments)
            drawLines(g, roadSegments);
        if (UIInfo.showBuildingDebug) {
            drawLines(g, buildingOutlines);
            drawLines(g, buildingSeparatingEdges);
        }
        if (UIInfo.showAABBs) {
            drawLines(g, buildingAABBs);
        }
    }

    @Override
    public List<String> getInfo() {
        return new ArrayList<String>(Arrays.asList("Map: " + world.name));
    }

    @Override
    public String[] getHoverInfo(Vec2 worldPos) {
        if (world.converter.isPresent()) {
            world.converter.get().metersToCoords(worldPos, coords);
            return new String[]{"lon: " + geoFormat.format(coords.lon) + " lat: " + geoFormat.format(coords.lat)};
        }
        return null;
    }

    @Override
    public JMenuItem[] getClicMenuItem(Vec2 worldPos) {
        if (!world.converter.isPresent()) return null;
        world.converter.get().metersToCoords(worldPos, coords);
        String info = geoFormat.format(coords.lon) + ", " + geoFormat.format(coords.lat);
        return new JMenuItem[]{new CopyMenuItem("Copy Coordinates", info, true)};
    }


}