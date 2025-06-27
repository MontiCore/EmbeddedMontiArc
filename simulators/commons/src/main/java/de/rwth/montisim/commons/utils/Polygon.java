package de.rwth.montisim.commons.utils;

import java.util.List;
import java.util.ArrayList;

public class Polygon {
    // Assumes edge2 comes after edge1
    public static double getAlignment(Vec2 edge1, Vec2 edge2) {
        return edge1.y * edge2.x - edge1.x*edge2.y;
    }
    public static boolean isClockwise(double alignment) {
        return alignment > -0.00001;
    }
    public static boolean isCounterClockwise(double alignment) {
        return alignment < 0.00001;
    }

    public List<Vec2> points = new ArrayList<>(); // Closed => there is an edge between the last point and the first
    public boolean clockwise = false; // false -> counter-clockwise
    public boolean vertexConvex[];
    
    public void checkClockwise() {
        int maxId = 0;
        double maxX = Double.NEGATIVE_INFINITY;
        // Get furthest point along X axis -> must be a convex vertex
        for (int i = 0; i < points.size(); ++i) {
            Vec2 point = points.get(i);
            if (point.x > maxX) {
                maxX = point.x;
                maxId = i;
            }
        }

        // Use this vertex convex and its neighboring edges to get if the points are listed CW or CCW
        Vec2 prevPoint = points.get((maxId-1+points.size())%points.size());
        Vec2 point = points.get(maxId);
        Vec2 nextPoint = points.get((maxId+1)%points.size());
        Vec2 edgeVec1 = point.subtract(prevPoint);
        Vec2 edgeVec2 = nextPoint.subtract(point);
        double res = getAlignment(edgeVec1, edgeVec2);
        if (Math.abs(res) < 0.0001) {
            System.out.println("Degenerate Clock-wise interpretation. (Building.java)");
        }

        clockwise = isClockwise(res);
    }

    public void checkVerticesConvex() {
        int vertexCount = points.size();
        vertexConvex = new boolean[vertexCount];
        Vec2 point = points.get(vertexCount-1);
        Vec2 nextPoint = points.get(0);
        Vec2 edge1 = new Vec2();
        Vec2 edge2 = nextPoint.subtract(point);
        for (int i = 0; i < vertexCount; ++i) {
            point = nextPoint;
            nextPoint = points.get((i+1)%vertexCount);
            edge1.set(edge2);
            IPM.subtractTo(edge2, nextPoint, point);
            vertexConvex[i] = isCounterClockwise(getAlignment(edge1, edge2)) != clockwise; // Use isCounterClockwise() so that vertices in a straight line are not taken as concave
        }
    }

    public void convertToCounterClockwise() {
        if (clockwise) {
            int pointCount = points.size();
            ArrayList<Vec2> newPoints = new ArrayList<>(pointCount);
            for (int i = 0; i < pointCount; ++i) {
                newPoints.add(points.get(pointCount-1-i));
            }
            points = newPoints;
            clockwise = false;
        }
    }
}
