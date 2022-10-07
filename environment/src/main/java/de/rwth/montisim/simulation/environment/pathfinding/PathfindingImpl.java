/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.environment.pathfinding;

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.logging.Logger;

import de.rwth.montisim.commons.map.Path;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;
import de.rwth.montisim.simulation.environment.world.elements.WaySegment;


public class PathfindingImpl implements Pathfinding {
    final World world;

    final double[] edgeCosts;

    final EdgeComparator comparator;

    final PriorityQueue<Integer> nextEdges;

    public PathfindingImpl(World world) {
        this.world = world;

        int vertexCount = world.nodes.size();
        // Allocate Vertex Colors
        visited = new boolean[vertexCount];
        bestCost = new double[vertexCount];
        pathIndex = new int[vertexCount];
        vertexPredecessorEdge = new int[vertexCount];

        int edgeCount = world.waySegments.size();
        edgeCosts = new double[edgeCount];
        for (int i = 0; i < edgeCount; ++i) {
            WaySegment rs = world.waySegments.elementAt(i);
            edgeCosts[i] = rs.length * rs.travelCost;
        }
        // Allocate Edge Colors
        pathCosts = new double[edgeCount];

        comparator = new EdgeComparator(pathCosts);
        nextEdges = new PriorityQueue<>(comparator);
    }


    public static class SegmentReference {
        Vec3 originalPoint;
        int roadSegmentID = -1; // -1 if no ref
        int reverseId = -1; // If the current best segment has a reverse segment -> its ID.
        boolean isPoint = false; // True if the nearest element of the RoadSegment is a point (end or corner)
        int roadPointID; // if 'isPoint' -> Nearest point else -> nearest segment = (road.points[roadPointID] to road.points[roadPointID+1])
        Vec3 point = null;
        double distance = Double.POSITIVE_INFINITY; // Distance from start of segment to nearest point
    }

    private SegmentReference getNearestSegment(Vec3 pos) {
        // TODO Should be accelerated using a spatial data-structure
        double currentNearestDistance = Double.POSITIVE_INFINITY;
        SegmentReference ref = new SegmentReference();
        ref.originalPoint = pos;
        ref.point = new Vec3();
        int count = world.waySegments.size();
        Vec3 normal = new Vec3();
        Vec3 dir = new Vec3();
        Vec3 delta = new Vec3();
        double dist;
        boolean checked[] = new boolean[count];
        for (int i = 0; i < checked.length; ++i) checked[i] = false;
        for (int rsID = 0; rsID < count; rsID++) {
            if (checked[rsID]) continue;
            WaySegment rs = world.waySegments.elementAt(rsID);
            if (rs.reverseId >= 0) checked[rs.reverseId] = true;
            Way r = rs.way;

            Vec3 lastPoint = null;
            double totalDist = 0;
            int inc = rs.pointsStart < rs.pointsEnd ? 1 : -1;
            for (int i = rs.pointsStart; i != rs.pointsEnd + inc; i += inc) {
                Vec3 p = r.points.elementAt(i);


                if (lastPoint != null) {
                    // Check segment

                    // 1) Get segment "normal"
                    IPM.subtractTo(dir, p, lastPoint);
                    // Manual normalization to keep the length
                    double length = dir.magnitude();
                    if (length > 0.001) {
                        IPM.multiply(dir, 1 / length);
                    } else {
                        dir.set(Double.NaN, Double.NaN, Double.NaN);
                    }

                    // 2) check if in segment bounds
                    IPM.subtractTo(delta, pos, lastPoint);
                    double projPos = IPM.dot(dir, delta);
                    if (projPos > 0 && projPos < length) {

                        // 3) get distance
                        normal.set(-dir.y, dir.x, 0);
                        dist = Math.abs(IPM.dot(normal, delta));
                        if (dist < currentNearestDistance) {
                            currentNearestDistance = dist;
                            ref.roadSegmentID = rsID;
                            ref.isPoint = false;
                            IPM.multiplyTo(ref.point, dir, projPos);
                            IPM.add(ref.point, lastPoint);
                            ref.roadPointID = i - inc;
                            ref.distance = totalDist + projPos;
                            ref.reverseId = rs.reverseId;
                        }
                    }

                    totalDist += length; // Track where we are along the segment
                }


                //Check point
                dist = p.distance(pos);
                if (dist < currentNearestDistance) {
                    currentNearestDistance = dist;
                    ref.roadSegmentID = rsID;
                    ref.isPoint = true;
                    ref.point.set(p);
                    ref.roadPointID = i;
                    ref.distance = totalDist;
                    ref.reverseId = rs.reverseId;
                }

                lastPoint = p;
            }
        }
        return ref;
    }


    private static class EdgeComparator implements Comparator<Integer> {
        final double[] costArray;

        public EdgeComparator(double[] costArray) {
            this.costArray = costArray;
        }

        @Override
        public int compare(Integer o1, Integer o2) {
            double c1 = costArray[o1];
            double c2 = costArray[o2];
            if (c1 < c2) return -1;
            if (c1 > c2) return 1;
            return 0;
        }
    }


    // Vertex Colors <=> Intersection
    final boolean[] visited;
    final double[] bestCost;
    final int[] pathIndex;
    final int[] vertexPredecessorEdge; // Road Segment ID of the edge leading into this vertex.

    // Edge colors <=> RoadSegment
    final double[] pathCosts;

    public SegmentReference startRef;
    public SegmentReference targetRef;

    WaySegment segmentStart = null;
    WaySegment segmentStartReverse = null;
    WaySegment segmentTarget = null;
    WaySegment segmentTargetReverse = null;

    // TODO: Remove this public version and put back the private one
    public Path findShortestPath(Vec2 startCoords, Vec2 targetCoords) throws Exception {
        return new PathfindingImpl(world)._findShortestPath(startCoords, targetCoords);
    }

    /*
        Vertex <=> Intersection
        Edge <=> RoadSegment
    */
    private Path _findShortestPath(Vec2 startCoords, Vec2 targetCoords) throws Exception {
        startRef = getNearestSegment(new Vec3(startCoords, 0));
        targetRef = getNearestSegment(new Vec3(targetCoords, 0));
        if (startRef.roadSegmentID < 0 || targetRef.roadSegmentID < 0)
            throw new Exception("Error finding nearest segment for trajectory.");

        if (startRef.roadSegmentID == targetRef.roadSegmentID) {
            return getSameSegmentPath();
        }

        segmentStart = world.getWaySegment(startRef.roadSegmentID);
        if (startRef.reverseId >= 0) segmentStartReverse = world.getWaySegment(startRef.reverseId);
        segmentTarget = world.getWaySegment(targetRef.roadSegmentID);
        if (targetRef.reverseId >= 0) segmentTargetReverse = world.getWaySegment(targetRef.reverseId);

        //Dijkstra !

        // Init colors
        for (int i = 0; i < visited.length; ++i) {
            visited[i] = false;
            bestCost[i] = Double.POSITIVE_INFINITY;
        }

        nextEdges.clear();

        // Add first vertices (2 ends of the start segment)
        if (segmentStart.endNodeID >= 0) {
            int index = Math.abs(segmentStart.pointsEnd - startRef.roadPointID) + 1;
            if (startRef.isPoint) index--;
            addVertex(
                    segmentStart.endNodeID,
                    (segmentStart.length - startRef.distance) * segmentStart.travelCost, // Partial cost to end
                    index,  // Number of points in the path for the start segment
                    startRef.roadSegmentID,
                    startRef.reverseId
            );
        }
        if (segmentStartReverse != null && segmentStartReverse.endNodeID >= 0) {
            int index = Math.abs(startRef.roadPointID - segmentStart.pointsStart) + 1;
            if (startRef.isPoint) index--;
            addVertex(
                    segmentStartReverse.endNodeID,
                    startRef.distance * segmentStartReverse.travelCost, // Partial cost to end
                    index, // Number of points in the path for the start segment
                    startRef.reverseId,
                    startRef.roadSegmentID
            );
        }


        boolean found = false;
        int lastEdgeID = -1;

        while (!nextEdges.isEmpty()) {
            int edgeID = nextEdges.poll(); // The returned edge is always the edge that leads to the next closest vertex

            // Check if it is (one of) the target segment
            if (edgeID == targetRef.roadSegmentID || edgeID == targetRef.reverseId) {
                found = true;
                lastEdgeID = edgeID;
                break;
            }

            WaySegment rs = world.getWaySegment(edgeID);
            int index = pathIndex[rs.startNodeID] + Math.abs(rs.pointsEnd - rs.pointsStart);
            if (!visited[rs.endNodeID]) // We only receive edges with an existing end vertex
                addVertex(rs.endNodeID, pathCosts[edgeID], index, edgeID, rs.reverseId);
        }

        if (!found) {
            Logger.getGlobal().warning("No Path Found (Pathfinding).");
            return new Path(0);
        }

        return getPath(lastEdgeID);
    }

    private void addVertex(int IID, double costs, int index, int predEdge, int reversePredEdge) {
        // Add vertex information
        visited[IID] = true;
        bestCost[IID] = costs;
        pathIndex[IID] = index;
        vertexPredecessorEdge[IID] = predEdge;

        // Add outgoing edges
        Node is = world.getNode(IID);
        for (int outRSID : is.outRoadSegmentIDs) {
            if (outRSID == reversePredEdge) continue;
            WaySegment rs = world.getWaySegment(outRSID);
            // Check if Target segment OR Not a dead end and not visited
            if ((outRSID == targetRef.roadSegmentID || outRSID == targetRef.reverseId) || (rs.endNodeID >= 0 && !visited[rs.endNodeID])) {
                double cost = costs;
                if (outRSID == targetRef.roadSegmentID) {
                    cost += targetRef.distance * segmentTarget.travelCost;
                } else if (outRSID == targetRef.reverseId) {
                    cost += (segmentTarget.length - targetRef.distance) * segmentTargetReverse.travelCost;
                } else {
                    cost += edgeCosts[outRSID];
                }
                // Add edge
                pathCosts[outRSID] = cost;
                nextEdges.add(outRSID);
            }
        }
    }


    private Path getPath(int lastEdgeID) {
        WaySegment s = world.getWaySegment(lastEdgeID);
        // Get total point count in the result path
        int pointCount = pathIndex[s.startNodeID] + 2 + 2; // 2 points for start and 2 points for end

        if (lastEdgeID == targetRef.reverseId) {
            pointCount += Math.abs(targetRef.roadPointID - segmentTarget.pointsEnd);
            if (targetRef.isPoint && targetRef.roadPointID != segmentTarget.pointsEnd) {
                pointCount--;
            }
        } else {
            pointCount += Math.abs(targetRef.roadPointID - segmentTarget.pointsStart);
            if (targetRef.isPoint && targetRef.roadPointID != segmentTarget.pointsStart) {
                pointCount--;
            }
        }

        Path path = new Path(pointCount);
        int pos = path.getLength();

        // Fill the path in reverse:
        // Set the the target point
        path.set(--pos, targetRef.originalPoint.x, targetRef.originalPoint.y);
        path.set(--pos, targetRef.point.x, targetRef.point.y);

        int inc;
        int start;
        int end;
        if (lastEdgeID == targetRef.reverseId) {
            // Fill from target point to segment end (don't include last point -> added from previous segment)
            inc = segmentTarget.pointsStart < segmentTarget.pointsEnd ? 1 : -1;
            start = targetRef.roadPointID;
            end = segmentTarget.pointsEnd;

            if (targetRef.isPoint && targetRef.roadPointID != segmentTarget.pointsEnd) {
                start += inc;
            }
        } else {
            // Fill from target point to segment start (don't include first point -> added from previous segment)
            inc = segmentTarget.pointsStart < segmentTarget.pointsEnd ? -1 : 1;
            start = targetRef.roadPointID;
            end = segmentTarget.pointsStart;

            if (targetRef.isPoint && targetRef.roadPointID != segmentTarget.pointsStart) {
                start += inc;
            }
        }
        // Fill the target segment
        for (int i = start; i != end; i += inc) {
            Vec3 p = segmentTarget.way.points.elementAt(i);
            path.set(--pos, p.x, p.y);
        }

        // Add all completely traversed segments
        while (true) {
            int tmp = lastEdgeID;
            lastEdgeID = vertexPredecessorEdge[s.startNodeID];
            if (lastEdgeID == -1) return new Path(0);
            s = world.getWaySegment(lastEdgeID);
            if (lastEdgeID == startRef.roadSegmentID || lastEdgeID == startRef.reverseId)
                break; // Reached start segment

            inc = s.pointsStart < s.pointsEnd ? -1 : 1;

            // Fill from segment end to start, don't include first -> added by 'previous' segment
            for (int i = s.pointsEnd; i != s.pointsStart; i += inc) {
                Vec3 p = s.way.points.elementAt(i);
                path.set(--pos, p.x, p.y);
            }
        }

        if (lastEdgeID == startRef.reverseId) {
            // Fill from segment start to target point
            inc = segmentStart.pointsStart < segmentStart.pointsEnd ? 1 : -1;
            start = segmentStart.pointsStart;
            end = startRef.roadPointID;

            if (startRef.isPoint) {
                end -= inc;
            }
        } else {
            // Fill from segment end to target point
            inc = segmentStart.pointsStart < segmentStart.pointsEnd ? -1 : 1;
            start = segmentStart.pointsEnd;
            end = startRef.roadPointID;

            if (startRef.isPoint) {
                end -= inc;
            }
        }
        // Fill start segment
        for (int i = start; i != end + inc; i += inc) {
            Vec3 p = s.way.points.elementAt(i);
            path.set(--pos, p.x, p.y);
        }

        // Set the start point
        path.set(--pos, startRef.point.x, startRef.point.y);
        path.set(--pos, startRef.originalPoint.x, startRef.originalPoint.y);

        if (pos != 0) {
            Logger.getGlobal().warning("Uncorrect path length (Pathfinding.getPath()), " + (pos) + " too long.");
            return new Path(0);
        }

        return path;
    }

    private Path getSameSegmentPath() {
        if (startRef.roadPointID == targetRef.roadPointID) {
            boolean shared = startRef.isPoint && targetRef.isPoint;
            Path path = new Path(shared ? 3 : 4);
            int pos = 0;
            path.set(pos++, startRef.originalPoint.x, startRef.originalPoint.y);
            path.set(pos++, startRef.point.x, startRef.point.y);
            if (!shared) path.set(pos++, targetRef.point.x, targetRef.point.y);
            path.set(pos++, targetRef.originalPoint.x, targetRef.originalPoint.y);
            return path;
        }
        segmentStart = world.getWaySegment(startRef.roadSegmentID);
        int delta = startRef.roadPointID < targetRef.roadPointID ? 1 : -1;
        // Iteration params
        int inc = segmentStart.pointsStart < segmentStart.pointsEnd ? 1 : -1;
        int start = startRef.roadPointID + delta;
        int end = targetRef.roadPointID;

        int ScompT = delta == inc ? 1 : -1;

        // Number of points in the path
        int count = 4; // 2 original points & 2 start/target points
        // Count from start to target (excluded)
        count += Math.abs(startRef.roadPointID - targetRef.roadPointID) - 1;
        if (ScompT < 0 && !startRef.isPoint) {
            ++count;
            start -= delta;
        }
        // Count target segment point if needed
        if (ScompT > 0 && !targetRef.isPoint) {
            ++count;
            end += delta;
        }

        // TODO add intermediate segment points
        Path path = new Path(count);
        int pos = 0;
        path.set(pos++, startRef.originalPoint.x, startRef.originalPoint.y);
        path.set(pos++, startRef.point.x, startRef.point.y);

        // Add intermediate points
        for (int i = start; i != end; i += delta) {
            Vec3 p = segmentStart.way.points.elementAt(i);
            path.set(pos++, p.x, p.y);
        }

        path.set(pos++, targetRef.point.x, targetRef.point.y);
        path.set(pos++, targetRef.originalPoint.x, targetRef.originalPoint.y);
        return path;
    }
}
