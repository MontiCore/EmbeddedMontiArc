/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.world.elements;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.PolygonConvexer.ConversionError;
import de.rwth.montisim.commons.utils.PolygonConvexer.Edge;

import de.rwth.montisim.simulation.commons.*;
import de.rwth.montisim.simulation.commons.boundingbox.*;

public class Building extends SimulationObject {
    public static double DEFAULT_LEVEL_HEIGHT = 4.0;
    public static double DEFAULT_BUILDING_HEIGHT = 4.0;
    public final long osmId;
    public final String name;
    public final String type;
    public final double height;
    public final int levels;
    public final List<Vec3> boundary = new ArrayList<>();
    public final Polygon fullPolygon = new Polygon();
    public final AABB buildingAABB = new AABB();
    public List<Polygon> convexParts;
    public StaticObject staticObject;
    public final List<Edge> separatingEdges = new ArrayList<>();

    public Building(String name, long osmId, String type, double height, int levels) {
        this.name = name + "(OSM id: " + Long.toString(osmId) + ")";
        this.osmId = osmId;
        this.type = type;
        this.height = Math.abs(height) < 0.001 ? (levels > 0 ? DEFAULT_LEVEL_HEIGHT * levels : DEFAULT_BUILDING_HEIGHT) : height;
        this.levels = levels;
    }

    void computeStaticObject() {
        staticObject = new StaticObject("building");
        staticObject.name = name;

        buildingAABB.init();

        for (int i = 0; i < boundary.size(); ++i) {
            Vec3 point = boundary.get(i);
            fullPolygon.points.add(new Vec2(point.x, point.y)); // Create polygon
            buildingAABB.include(point); // Expand AABB
            IPM.add(staticObject.pos, point); // Sum vertices for "center of mass" (not really of mass) for the position
        }
        IPM.multiply(staticObject.pos, 1.0 / boundary.size());

        try {
            PolygonConvexer conv = new PolygonConvexer(fullPolygon);
            convexParts = conv.getParts();
            separatingEdges.addAll(conv.separatingEdges);
        } catch (ConversionError e) {
            throw new IllegalArgumentException("Polygon conversion error on building '" + name + "'");
        }

        buildingAABB.min.z = 0.0;
        buildingAABB.max.z = height;

        staticObject.worldSpaceAABB = Optional.of(buildingAABB);

        Union union = new Union();

        for (Polygon part : convexParts) {
            ConvexHull hull = new ConvexHull();
            int vertexCount = part.points.size();
            hull.halfSpaceNormals = new Vec3[vertexCount + 2]; // One half-space per edge + top & bottom
            hull.halfSpacePos = new double[vertexCount + 2];
            hull.corners = new Vec3[vertexCount * 2]; // "Extrude" 2D shape of the building to 'height'

            // Add a half-space per edge of the boundary part
            Vec2 lastPoint = part.points.get(vertexCount - 1);
            for (int i = 0; i < vertexCount; ++i) {
                Vec2 point = part.points.get(i);
                Vec3 point3D = new Vec3(point.x, point.y, 0);

                hull.halfSpaceNormals[i] = new Vec3(point.y - lastPoint.y, -(point.x - lastPoint.x), 0.0);
                hull.halfSpacePos[i] = IPM.dot(point3D, hull.halfSpaceNormals[i]);
                hull.corners[i * 2] = point3D;
                hull.corners[i * 2 + 1] = new Vec3(point.x, point.y, height);

                lastPoint = point;
            }

            // Add top & bottom half-spaces
            hull.halfSpaceNormals[vertexCount] = new Vec3(0, 0, 1); // Top: up normal
            hull.halfSpacePos[vertexCount] = height;
            hull.halfSpaceNormals[vertexCount + 1] = new Vec3(0, 0, -1); // Bottom: down normal
            hull.halfSpacePos[vertexCount + 1] = 0;


            union.boundingBoxes.add(hull);
        }

        staticObject.bbox = Optional.of(union);
    }

    @Override
    public void registerComponents(ISimulator simulator) {
        computeStaticObject();

        simulator.registerStaticObject(this, staticObject);
    }
}