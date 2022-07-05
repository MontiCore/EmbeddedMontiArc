package de.rwth.montisim.simulation.simulator.randomization;

import java.util.Optional;
import java.util.Random;
import java.util.Vector;

import org.apache.http.impl.client.NullBackoffStrategy;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;
import de.rwth.montisim.simulation.environment.world.elements.WaySegment;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

public class PlatooningStrategy extends RandomizationStrategy {

    private final World world;

    private final int maxNumberOfVehicles;
    private final int minNumberOfVehicles;
    private final double minDistanceBtwVehicles;
    private final double maxDistanceBtwVehicles;
    private final double minGoalDistance;
    private final double maxGoalDistance;

    private final SimpleCoordinateConverter converter;

    private Vector<Pair<Coordinates, Double>> generatedStartPoses;
    private Vector<Pair<Coordinates, Double>> generatedEndPoses;

    private Vector<Node> generatedPath;

    private Vector<Integer> avoidSegmentIDs;

    public PlatooningStrategy(Optional<Long> seed, World world,
                              int minNumberOfVehicles, int maxNumberOfVehicles,
                              double minDistanceBtwVehicles, double maxDistanceBtwVehicles,
                              double minGoalDistance, double maxGoalDistance) {
        super(seed);
        this.world = world;
        this.minNumberOfVehicles = minNumberOfVehicles;
        this.maxNumberOfVehicles = maxNumberOfVehicles;
        this.minDistanceBtwVehicles = minDistanceBtwVehicles;
        this.maxDistanceBtwVehicles = maxDistanceBtwVehicles;
        this.minGoalDistance = minGoalDistance;
        this.maxGoalDistance = maxGoalDistance;

        this.converter = this.world.converter.get();

        // Select random number of vehicles
        int numbOfVehicles = this.random.nextInt(this.maxNumberOfVehicles - this.minNumberOfVehicles + 1) + this.minNumberOfVehicles;

        generatedStartPoses = null;
        generatedEndPoses = null;
        while (generatedStartPoses == null || generatedEndPoses == null) {
            // Pick a random starting node and path
            Node start = null;
            generatedPath = null;
            while (start == null || generatedPath == null) {
                Vector<Node> nodes = this.world.nodes;
                start = nodes.get(this.random.nextInt(nodes.size()));
                generatedPath = traverse(start);
            }

            assert (start != null);

            // get all WaySegments on generatedPath
            avoidSegmentIDs = new Vector<>();
            Node lastNode = start;
            for (Node nextNode : generatedPath) {
                WaySegment outSegment = lastNode.outRoadSegmentIDs.stream()
                        .map(id -> this.world.getWaySegment(id))
                        .filter(segment -> segment.endNodeID == nextNode.localID)
                        .findFirst().get();
                avoidSegmentIDs.add(outSegment.localID);
                avoidSegmentIDs.add(outSegment.reverseId);
                lastNode = nextNode;
            }

            //TODO: Generate End Poses in a similar fashion
            // Generate Start and End Poses randomly
            generatedStartPoses = randomizedPoses(start, true, numbOfVehicles);
            generatedEndPoses = randomizedPoses(generatedPath.lastElement(), false, numbOfVehicles);
        }
    }

    /**
     * Traverse the map randomly from the start node to find a route
     *
     * @param start
     * @param minGoalDistance
     * @param maxGoalDistance
     * @param random
     * @return
     */
    private Vector<Node> traverse(Node start) {
        return traverse(start, new Vector<Node>(), 0);
    }

    private Vector<Node> traverse(Node start, Vector<Node> currPath, double currDistance) {
        Vector<Node> adjeNodes = new Vector<>();
        for (int wayID : start.outRoadSegmentIDs) {
            WaySegment segment = this.world.getWaySegment(wayID);
            int outNodeID = segment.endNodeID;
            if (outNodeID < 0) { // not a node. Skip for now
                continue;
            }
            Node outNode = this.world.getNode(outNodeID);
            if (!currPath.contains(outNode)) {
                adjeNodes.add(outNode);
            }
        }
        if (adjeNodes.isEmpty()) {
            return null; // backtrack
        }

        // check if any adjecent Nodes are sufficient for min length
        for (Node outNode : adjeNodes) {
            double distance = start.point.distance(outNode.point);
            distance += currDistance;
            if (minGoalDistance <= distance && distance <= maxGoalDistance) {
                currPath.add(outNode);
                return currPath;
            }
            if (maxGoalDistance < distance) { // node would make travel distance to large
                adjeNodes.remove(outNode);
            }
        }
        if (adjeNodes.isEmpty()) {
            return null; // backtrack
        }

        // recursivly find a path if possible
        for (Node outNode : adjeNodes) {
            double distance = start.point.distance(outNode.point);
            Vector<Node> path = new Vector<>(currPath);
            path.add(outNode);
            Vector<Node> result = traverse(outNode, path, currDistance + distance);
            if (result != null) {
                return result;
            }
        }

        return null; // backtrack
    }


    private Vector<Pair<Coordinates, Double>> randomizedPoses(Node node, boolean inward, int poses_to_generate) {
        Vector<Pair<Integer, Double>> segments = randomizedSegments(node, new Vector<Pair<Integer, Double>>(), this.avoidSegmentIDs, inward, 0);
        if (segments.isEmpty()) {
            return null;
        }

        Vector<Pair<Coordinates, Double>> poses = new Vector<>();

        // spawn vehicles randomly
        for (int i = 0; i < poses_to_generate; i++) {
            if (segments.isEmpty()) { // because we remove segments
                return null;
            }
            // get random WaySegment to generate pose on
            Pair<Integer, Double> pair = segments.get(0);
            Integer segmentID = pair.getKey();
            WaySegment segment = world.getWaySegment(segmentID);
            Double spawnLength = pair.getValue();

            // traverse segment to find a spawn point
            double traverse_length = spawnLength + this.random.nextDouble() * (this.maxDistanceBtwVehicles - this.minDistanceBtwVehicles) + this.minDistanceBtwVehicles;

            if (traverse_length >= segment.length) { // cannot place pose here, but at child // >= not > makes it easier later (no checking if last element)
                // remove segment
                segments.remove(0);

                // recursivly add childs
                int childNodeID = inward ? segment.startNodeID : segment.endNodeID;
                if (childNodeID >= 0) { // child is a node
                    Node childNode = world.getNode(childNodeID);
                    Vector<Integer> newAvoidSegmentIDs = new Vector<>(this.avoidSegmentIDs);
                    newAvoidSegmentIDs.add(segment.localID);
                    newAvoidSegmentIDs.add(segment.reverseId);
                    segments = randomizedSegments(childNode, segments, newAvoidSegmentIDs, inward, 0);
                }

                // try again for vehicle i
                i--;
                continue;
            } else { // place pose here: traverse the way to find the nearest point
                // get way
                Way way = segment.way;
                // determine if waysegment is reversed
                boolean reversed = (segment.pointsStart > segment.pointsEnd);
                // travel traverse_length from pointsStart to pointsEnd
                double currDistance = 0;

                int currPointID = inward ? segment.pointsEnd : segment.pointsStart;
                int inc = reversed ^ !inward ? 1 : -1;

                Vec3 currPoint = way.points.get(currPointID);
                int nextPointID;
                do {
                    nextPointID = currPointID + inc; // step forward

                    Vec3 nextPoint = way.points.get(nextPointID);
                    double distance = currPoint.distance(nextPoint);

                    currDistance += distance;
                    currPointID = nextPointID;
                    currPoint = nextPoint;

                    if (currDistance >= traverse_length) {
                        break;
                    }
                } while (!((segment.pointsStart <= nextPointID || nextPointID <= segment.pointsEnd) && (segment.pointsEnd <= nextPointID || nextPointID <= segment.pointsStart)));

                int prevPointID = currPointID - inc; // step backward
                Vec3 prevPoint = way.points.get(prevPointID);

                // interpolate point
                double total_distance = prevPoint.distance(currPoint);
                double distance = currDistance - (traverse_length);
                double fraction = distance / total_distance;

                Vec2 spawnPoint = new Vec2(prevPoint.x + fraction * (currPoint.x - prevPoint.x), prevPoint.y + fraction * (currPoint.y - prevPoint.y));

                double orientation = Math.toDegrees(Math.atan2(spawnPoint.y - prevPoint.y, spawnPoint.x - prevPoint.x)) - 180;
                Coordinates spawnCoordinates = new Coordinates();
                this.converter.metersToCoords(spawnPoint, spawnCoordinates);
                poses.add(new Pair<Coordinates, Double>(spawnCoordinates, orientation));

                // update segments
                segments.remove(0);
                segments.add(new Pair<Integer, Double>(segmentID, traverse_length));

                // add segment to avoidSegmentIDs
                this.avoidSegmentIDs.add(segment.localID);
                this.avoidSegmentIDs.add(segment.reverseId);
            }
        }
        assert (poses.size() == poses_to_generate);
        return poses;
    }

    private Vector<Pair<Integer, Double>> randomizedSegments(Node startNode, Vector<Pair<Integer, Double>> existingSegments, Vector<Integer> avoidSegmentIDs, boolean inward, double spawnOffset) {
        Vector<Integer> newSegmentsIDs = inward ? startNode.inRoadSegmentIDs : startNode.outRoadSegmentIDs;
        Vector<Pair<Integer, Double>> segments = new Vector<Pair<Integer, Double>>(existingSegments);
        for (Integer newSegmentID : newSegmentsIDs) {
            WaySegment segment = this.world.getWaySegment(newSegmentID);
            boolean skipSegment = avoidSegmentIDs.contains(segment.localID);
            skipSegment = skipSegment || (segments.parallelStream().anyMatch(pair -> pair.getKey() == segment.localID || pair.getKey() == segment.reverseId));
            if (!skipSegment) {
                segments.add(new Pair<Integer, Double>(segment.localID, spawnOffset));
            }
        }
        return segments;
    }

    @Override
    public Vector<VehicleProperties> randomizeCars(Vector<VehicleProperties> vehicles) {
        VehicleProperties template = vehicles.firstElement();
        Vector<VehicleProperties> generatedVehicles = new Vector<VehicleProperties>(generatedStartPoses.size());
        for (int i = 0; i < generatedStartPoses.size(); i++) {

            // Assemble the randomized vehicle
            VehicleProperties newVehicle = new VehicleProperties();
            newVehicle.setName("Car_" + i);

            // Copy the standard properties from the first vehicle of the config
            newVehicle.components.clear();
            newVehicle.components.addAll(template.components);
            newVehicle.body = template.body;
            newVehicle.wheels = template.wheels;
            newVehicle.physics = template.physics;
            newVehicle.powertrain = template.powertrain;

            // Add randomized properties
            newVehicle.start_coords = Optional.of(generatedStartPoses.get(i).getKey());
            newVehicle.start_orientation = generatedStartPoses.get(i).getValue();

            TaskProperties task = new TaskProperties();
            PathGoalProperties goal = new PathGoalProperties();
            goal.ltl_operator = LTLOperator.EVENTUALLY;

            // add path points
            for (Node node : generatedPath) {
                goal.reach(node.point.asVec2());
            }

            // add end point
            Vec2 endPoint = new Vec2();
            this.converter.coordsToMeters(generatedEndPoses.get(i).getKey(), endPoint);
            goal.reach(endPoint);

            task.addGoal(goal);
            newVehicle.task = task;

            // Add the vehicle to the list
            generatedVehicles.add(newVehicle);
        }
        return generatedVehicles;
    }

}
