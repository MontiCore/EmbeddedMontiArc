package de.rwth.montisim.commons.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class PolygonConvexer {
    public static class ConversionError extends Exception {
        public ConversionError() {
            super("Could not convert concave polygon to convex polygon");
        }
    }
    public static class Edge {
        public final Vec2 p1 = new Vec2();
        public final Vec2 p2 = new Vec2();

        public Edge(Vec2 p1, Vec2 p2) {
            this.p1.set(p1);
            this.p2.set(p2);
        }
    }

    Polygon polygon;
    final List<Polygon> parts = new ArrayList<>();
    public final List<Edge> separatingEdges = new ArrayList<>();

    public PolygonConvexer(Polygon polygon) {
        this.polygon = polygon;
        polygon.checkClockwise();
        polygon.convertToCounterClockwise();
        polygon.checkVerticesConvex();
    }

    public List<Polygon> getParts() throws ConversionError {
        toConvexParts();
        return parts;
    }

    
    private void toConvexParts() throws ConversionError {
        int checkedVertex = 0;
        boolean hasConcave = false;
        while (true) {
            // Get concave vertex with convex neighbor
            int vertexCount = polygon.points.size();
            while (checkedVertex < vertexCount) {
                if (!polygon.vertexConvex[checkedVertex]) {
                    hasConcave = true;
                    if (polygon.vertexConvex[(checkedVertex+1)%vertexCount] || polygon.vertexConvex[(checkedVertex-1+vertexCount)%vertexCount]) break; // Found valid concave vertex
                }
                ++checkedVertex;
            }

            if (!hasConcave) break;
            if (checkedVertex >= vertexCount) {
                throw new ConversionError();
            }

            // Get angle to first CCW edge
            int angleCount = vertexCount-1;
            Integer sortedId[] = new Integer[angleCount];
            double angle[] = new double[vertexCount];

            int nextId = (checkedVertex+1)%vertexCount;
            Vec2 center = polygon.points.get(checkedVertex);
            Vec2 nx = polygon.points.get(nextId).subtract(center); // These are not normalized by of the same length -> works for atan2()
            Vec2 ny = new Vec2(-nx.y, nx.x); // Rotate 90° CCW
            Vec2 delta = new Vec2();

            int k = 0;
            for (int i = 0; i < vertexCount; ++i) {
                if (i == checkedVertex) continue;
                sortedId[k] = i;
                ++k;

                IPM.subtractTo(delta, polygon.points.get(i), center);
                double a = Math.atan2(IPM.dot(ny, delta), IPM.dot(nx, delta));
                if (a < 0.0) a += Math.PI * 2.0;
                angle[i] = a;
            }
            Arrays.sort(sortedId, (idA, idB) -> Double.compare(angle[idA], angle[idB]) );


            // Traverse CCW
            int ccwCount = traverse(true, checkedVertex, sortedId);
            boolean hasCCW = ccwCount > 2;
            // Traverse CW
            int cwCount = traverse(false, checkedVertex, sortedId);
            boolean hasCW = cwCount > 2;

            boolean didSplit = false;
            if (hasCCW && (!hasCW || ccwCount > cwCount)) {
                split(true, checkedVertex, ccwCount);
                didSplit = true;
            } else if (hasCW) {
                split(false, checkedVertex, cwCount);
                didSplit = true;
            }

            if (didSplit) {
                // Reset main polygon traversal
                checkedVertex = 0;
                hasConcave = false;
            } else {
                ++checkedVertex;
            }
        }
        parts.add(polygon); // No more convex vertices -> add rest
    }

    /**
     * @return the number of points forming a convex polygon
     */
    private int traverse(boolean ccw, int firstPointId, Integer sortedId[]) {
        Vec2 a = new Vec2(); // Working vec
        int vertexCount = polygon.points.size();
        int angleCount = vertexCount-1;
        int inc = ccw ? 1 : -1;
        int secondPointId = (firstPointId+inc+vertexCount)%vertexCount;
        Vec2 firstPoint = polygon.points.get(firstPointId);
        Vec2 secondPoint = polygon.points.get(secondPointId);
        Vec2 edge = secondPoint.subtract(firstPoint);
        Vec2 validHalfSpace = new Vec2(-edge.y, edge.x);
        IPM.multiply(validHalfSpace, inc);

        int j = 0;
        while (sortedId[j] != secondPointId) ++j; // Get start position in "angle array"
        Vec2 lastPoint = secondPoint;
        int lastVid = secondPointId;

        int i = 2;
        traversalLoop:
        for (; i < vertexCount; ++i) {
            int vid = (firstPointId+i*inc+vertexCount)%vertexCount;
            if (!polygon.vertexConvex[lastVid]) break; // Check that vertex is convex

            Vec2 v = polygon.points.get(vid);
            IPM.subtractTo(a, v, firstPoint);
            // Check that the angle between the first edge of the traversal and the edge from the start-vertex to the current one is smaller than 180°.
            if (IPM.dot(a, validHalfSpace) < -0.00001) break;
            
            // Get "edge half space"
            IPM.subtractTo(edge, v, lastPoint);
            edge.set(-edge.y, edge.x);
            IPM.multiply(edge, inc);

            // Check that there is no other vertex inside the currently traversed polygon
            j = (j +inc+angleCount)%angleCount;
            while (sortedId[j] != vid) {
                IPM.subtractTo(a, polygon.points.get(sortedId[j]), lastPoint);
                if (IPM.dot(edge, a) > 0.0001) break traversalLoop;
                j = (j +inc+angleCount)%angleCount;
            }
            lastPoint = v;
            lastVid = vid;
        }
        return i;
    }

    private void split(boolean ccw, int firstPointId, int subPartVertexCount) {
        int inc = ccw ? 1 : -1;
        int vertexCount = polygon.points.size();
        Polygon newPolygon = new Polygon();
        Polygon newPart = new Polygon();

        Vec2 firstPoint = polygon.points.get(firstPointId);
        Vec2 lastPoint = polygon.points.get((firstPointId+(subPartVertexCount-1)*inc+vertexCount)%vertexCount);
        separatingEdges.add(new Edge(firstPoint, lastPoint));

        for (int i = 0; i < subPartVertexCount; ++i) {
            int vid = (firstPointId+i*inc+vertexCount)%vertexCount;
            newPart.points.add(polygon.points.get(vid));
        }
        newPart.clockwise = !ccw;
        newPart.convertToCounterClockwise();
        parts.add(newPart);

        int remainingCount = vertexCount - subPartVertexCount + 2;
        int remainingStart = (firstPointId+(subPartVertexCount-1)*inc)%vertexCount;
        for (int i = 0; i < remainingCount; ++i) {
            int vid = (remainingStart+i*inc+vertexCount)%vertexCount;
            newPolygon.points.add(polygon.points.get(vid));
        }
        newPolygon.clockwise = !ccw;
        newPolygon.convertToCounterClockwise();
        newPolygon.checkVerticesConvex();
        polygon = newPolygon;
    }

}
