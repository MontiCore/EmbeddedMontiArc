/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.simulation.eecomponents.navigation;

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.logging.Logger;

import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;
import de.rwth.montisim.simulation.environment.world.elements.WaySegment;


public class Pathfinding {
    final World world;

    final double[] edgeCosts;

    final EdgeComparator comparator;

    final PriorityQueue<Integer> nextEdges;

    public Pathfinding(World world){
        this.world = world;

        int vertexCount = world.nodes.size();
        // Allocate Vertex Colors
        visited = new boolean[vertexCount];
        bestCost = new double[vertexCount];
        pathIndex = new int[vertexCount];
        vertexPredecessorEdge = new int[vertexCount];

        int edgeCount = world.waySegments.size();
        edgeCosts = new double[edgeCount];
        for (int i = 0; i < edgeCount; ++i){
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
        boolean checked[] = new boolean[count];
        for (int i = 0; i < checked.length; ++i) checked[i] = false;
        for (int rsID = 0; rsID < count; rsID++){
            if (checked[rsID]) continue;
            WaySegment rs = world.waySegments.elementAt(rsID);
            if (rs.reverseId >= 0) checked[rs.reverseId] = true;
            Way r = rs.way;

            Vec3 lastPoint = null;
            double totalDist = 0;
            int inc = rs.pointsStart < rs.pointsEnd ? 1 : -1;
            for (int i = rs.pointsStart; i != rs.pointsEnd+inc; i+=inc){
                Vec3 p = r.points.elementAt(i);

                //Check point
                double dist = p.distance(pos);
                if (dist < currentNearestDistance){
                    currentNearestDistance = dist;
                    ref.roadSegmentID = rsID;
                    ref.isPoint = true;
                    ref.point.set(p);;
                    ref.roadPointID = i;
                    ref.distance = totalDist;
                    ref.reverseId = rs.reverseId;
                }

                if (lastPoint != null){
                    // Check segment

                    // 1) Get segment "normal"
                    IPM.subtractToVec(p, lastPoint, dir);
                    // Manual normalization to keep the length
                    double length = dir.magnitude();
                    if (length > 0.001){
                        IPM.multiply(dir, 1/length);
                    } else {
                        dir.set(Double.NaN,Double.NaN,Double.NaN);
                    }

                    // 2) check if in segment bounds
                    IPM.subtractToVec(pos, lastPoint, delta);
                    double projPos = IPM.dot(dir, delta);
                    if (projPos > 0 && projPos < length) {
                    
                        // 3) get distance
                        normal.set(-dir.y, dir.x, 0);
                        dist = Math.abs(IPM.dot(normal, delta));
                        if (dist < currentNearestDistance){
                            currentNearestDistance = dist;
                            ref.roadSegmentID = rsID;
                            ref.isPoint = false;
                            IPM.multiplyToVec(dir, projPos, ref.point);
                            IPM.add(ref.point, lastPoint);
                            ref.roadPointID = i-inc;
                            ref.distance = totalDist + projPos;
                            ref.reverseId = rs.reverseId;
                        }
                    }
                    
                    totalDist += length; // Track where we are along the segment
                }

                lastPoint = p;
            }
        }
        return ref;
    }

    
    private static class EdgeComparator implements Comparator<Integer> {
        final double[] costArray;
        public EdgeComparator(double[] costArray){
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


    /*
        Vertex <=> Intersection
        Edge <=> RoadSegment
    */
    public Vec2[] findShortestPath(Vec2 startCoords, Vec2 targetCoords) throws Exception {
        startRef = getNearestSegment(new Vec3(startCoords, 0));
        targetRef = getNearestSegment(new Vec3(targetCoords, 0));
        if (startRef.roadSegmentID < 0 || targetRef.roadSegmentID < 0) throw new Exception("Error finding nearest segment for trajectory.");
        
        if (startRef.roadSegmentID == targetRef.roadSegmentID){
            // TODO add intermediate segment points
            Vec2[] res = new Vec2[2];
            res[0] = new Vec2(startRef.point.x, startRef.point.y);
            res[1] = new Vec2(targetRef.point.x, targetRef.point.y);
            return res;
        }
        
        segmentStart = world.getWaySegment(startRef.roadSegmentID);
        if (startRef.reverseId>= 0) segmentStartReverse = world.getWaySegment(startRef.reverseId);
        segmentTarget = world.getWaySegment(targetRef.roadSegmentID);
        if (targetRef.reverseId>= 0) segmentTargetReverse = world.getWaySegment(targetRef.reverseId);

        //Dijkstra !

        // Init colors
        for (int i = 0; i < visited.length; ++i){
            visited[i] = false;
            bestCost[i] = Double.POSITIVE_INFINITY;
        }

        nextEdges.clear();

        // Add first vertices (2 ends of the start segment)
        if (segmentStart.endNodeID>=0){
            int index = Math.abs(segmentStart.pointsEnd - startRef.roadPointID) + 1;
            addVertex(
                segmentStart.endNodeID, 
                (segmentStart.length - startRef.distance)*segmentStart.travelCost, // Partial cost to end
                index,  // Number of points in the path for the start segment
                startRef.roadSegmentID
            );
        }
        if (segmentStartReverse != null && segmentStartReverse.endNodeID >= 0) {
            int index = Math.abs(startRef.roadPointID - segmentStart.pointsStart) + 1;
            if (!startRef.isPoint) index++;
            addVertex(
                segmentStartReverse.endNodeID, 
                startRef.distance*segmentStartReverse.travelCost, // Partial cost to end
                index, // Number of points in the path for the start segment
                startRef.roadSegmentID
            );
        }
            
        
        boolean found = false;
        int lastEdgeID = -1;

        while(!nextEdges.isEmpty()){
            int edgeID = nextEdges.poll(); // The returned edge is always the edge that leads to the next closest vertex
            
            // Check if it is (one of) the target segment
            if (edgeID == targetRef.roadSegmentID || edgeID == targetRef.reverseId) {
                found = true;
                lastEdgeID = edgeID;
                break;
            }

            WaySegment rs = world.getWaySegment(edgeID);
            int index = pathIndex[rs.startNodeID] + (rs.pointsEnd - rs.pointsStart);
            if (!visited[rs.endNodeID]) // We only receive edges with an existing end vertex
                addVertex(rs.endNodeID, pathCosts[edgeID], index, edgeID);
        }

        if (!found) {
            Logger.getGlobal().warning("No Path Found (Pathfinding).");
            return new Vec2[0];
        }

        return getPath(lastEdgeID);
    }

    private void addVertex(int IID, double costs, int index, int predEdge) {
        // Add vertex information
        visited[IID] = true;
        bestCost[IID] = costs;
        pathIndex[IID] = index;
        vertexPredecessorEdge[IID] = predEdge;

        // Add outgoing edges
        Node is = world.getNode(IID);
        for (int outRSID : is.outRoadSegmentIDs){
            if (outRSID == predEdge) continue;
            WaySegment rs = world.getWaySegment(outRSID);
            // Check if Target segment OR Not a dead end and not visited
            if ((outRSID == targetRef.roadSegmentID || outRSID == targetRef.reverseId) || (rs.endNodeID >= 0 && !visited[rs.endNodeID])){
                double cost = bestCost[IID];
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

    
    private Vec2[] getPath(int lastEdgeID) {
        WaySegment s = world.getWaySegment(lastEdgeID);
        // Get total point count in the result path
        int pointCount = pathIndex[s.startNodeID] + 2; // Count the original start & target points as well
        if (lastEdgeID == targetRef.reverseId){
            pointCount += Math.abs(targetRef.roadPointID - segmentTarget.pointsEnd);
        } else {
            pointCount += Math.abs(targetRef.roadPointID - segmentTarget.pointsStart);
            if (!targetRef.isPoint) pointCount++;
        }

        Vec2[] path = new Vec2[pointCount];
        int pos = path.length;
        
        // Fill the path in reverse: 
        // Set the the target point
        path[--pos] = new Vec2(targetRef.originalPoint.x, targetRef.originalPoint.y);
        path[--pos] = new Vec2(targetRef.point.x, targetRef.point.y);

        int inc;
        int start;
        int end;
        if (lastEdgeID == targetRef.reverseId){
            // Fill from target point to segment end (don't include last point -> added from previous segment)
            inc = segmentTarget.pointsStart < segmentTarget.pointsEnd ? 1 : -1;
            start = targetRef.roadPointID+inc;
            end = segmentTarget.pointsEnd;
        } else {
            // Fill from target point to segment start (don't include first point -> added from previous segment)
            inc = segmentTarget.pointsStart < segmentTarget.pointsEnd ? -1 : 1;
            start = targetRef.roadPointID;
            end = segmentTarget.pointsStart;
        }
        // Fill the target segment
        for (int i = start; i != end; i += inc){
            Vec3 p = segmentTarget.way.points.elementAt(i);
            path[--pos] = new Vec2(p.x, p.y);
        }

        // Add all completely traversed segments
        do {
            lastEdgeID = vertexPredecessorEdge[s.startNodeID];
            s = world.getWaySegment(lastEdgeID);
            if (lastEdgeID == startRef.roadSegmentID || lastEdgeID == startRef.reverseId) break; // Reached start segment
            inc = s.pointsStart < s.pointsEnd ? -1 : 1;
            // Fill from segment end to start, don't include first -> added by 'previous' segment
            for (int i = s.pointsEnd; i != s.pointsStart; i += inc){
                Vec3 p = s.way.points.elementAt(i);
                path[--pos] = new Vec2(p.x, p.y);
            }
        } while (true);

        if (lastEdgeID == startRef.roadSegmentID) {
            // Fill from segment end to start point
            inc = segmentStart.pointsStart < segmentStart.pointsEnd ? -1 : 1;
            start = segmentStart.pointsEnd;
            end = startRef.roadPointID;
        } else {
            // Fill from segment end to start point
            inc = segmentStart.pointsStart < segmentStart.pointsEnd ? 1 : -1;
            start = segmentStart.pointsStart;
            end = startRef.roadPointID + inc;
        }
        // Fill start segment
        for (int i = start; i != end; i += inc){
            Vec3 p = s.way.points.elementAt(i);
            path[--pos] = new Vec2(p.x, p.y);
        }
        
        // Set the start point
        path[--pos] = new Vec2(startRef.point.x, startRef.point.y);
        path[--pos] = new Vec2(startRef.originalPoint.x, startRef.originalPoint.y);
        if (pos != 0) {
            Logger.getGlobal().warning("Uncorrect path length (Pathfinding.getPath()), "+(pos)+" too long.");
            return new Vec2[0];
        }
        
        return path;
    }

}