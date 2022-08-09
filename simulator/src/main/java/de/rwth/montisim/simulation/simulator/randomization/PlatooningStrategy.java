package de.rwth.montisim.simulation.simulator.randomization;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;
import de.rwth.montisim.simulation.environment.world.elements.WaySegment;
import de.rwth.montisim.simulation.simulator.rewards.PlatooningRewardFunctionProperties;
import de.rwth.montisim.simulation.simulator.rewards.RewardFunctionProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

import java.util.Optional;
import java.util.Vector;

public class PlatooningStrategy extends RandomizationStrategy {

  private final World world;

  private final double minDistanceBtwVehicles;
  private final double maxDistanceBtwVehicles;
  private final double minGoalDistance;
  private final double maxGoalDistance;

  private final SimpleCoordinateConverter converter;

  private Vector<Pair<Coordinates, Double>> generatedStartPoses;
  private Vector<Pair<Coordinates, Double>> generatedEndPoses;

  private Vector<Node> generatedPath;

  private Vector<Integer> avoidSegmentIDs;

  public PlatooningStrategy(Optional<Long> seed, World world, int minNumberOfVehicles, int maxNumberOfVehicles, double minDistanceBtwVehicles, double maxDistanceBtwVehicles, double minGoalDistance, double maxGoalDistance) {
    super(seed);
    this.world = world;
    this.minDistanceBtwVehicles = minDistanceBtwVehicles;
    this.maxDistanceBtwVehicles = maxDistanceBtwVehicles;
    this.minGoalDistance = minGoalDistance;
    this.maxGoalDistance = maxGoalDistance;

    this.converter = this.world.converter.get();

    // Select random number of vehicles
    int numbOfVehicles = this.random.nextInt(maxNumberOfVehicles - minNumberOfVehicles + 1) + minNumberOfVehicles;

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
        WaySegment outSegment = lastNode.outRoadSegmentIDs.stream().map(this.world::getWaySegment).filter(segment -> segment.endNodeID == nextNode.localID).findFirst().get();
        avoidSegmentIDs.add(outSegment.localID);
        avoidSegmentIDs.add(outSegment.reverseId);
        lastNode = nextNode;
      }

      // Generate Start and End Poses randomly
      generatedStartPoses = randomizedPoses(start, true, numbOfVehicles);
      generatedEndPoses = randomizedPoses(generatedPath.lastElement(), false, numbOfVehicles);
    }
  }

  /**
   * Traverse the map randomly from the start node to find a route
   *
   * @param start The node from which to start traversing the world from.
   * @return A vector of nodes which form a valid path.
   */
  private Vector<Node> traverse(Node start) {
    return traverse(start, new Vector<>(), 0);
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

    // check if any adjacent Nodes are sufficient for min length
    for (Node outNode : new Vector<>(adjeNodes)) { // can modify adjeNodes, so copy it for iteration
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

    // recursively find a path if possible
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
    // get segments adjacent to node, that which not on the path
    Vector<Pair<Integer, Double>> segments = randomizedSegments(node, new Vector<>(), this.avoidSegmentIDs, inward, this.minDistanceBtwVehicles);
    if (segments.isEmpty()) {
      return null;
    }

    Vector<Pair<Coordinates, Double>> poses = new Vector<>();

    // spawn vehicles randomly
    for (int i = 0; i < poses_to_generate; i++) {
      if (segments.isEmpty()) { // because we remove segments
        // did not determine all poses yet, but also no more segments to choose from
        // try the whole randomization again
        return null;
      }

      // get random WaySegment to generate pose on
      Pair<Integer, Double> pair = segments.get(this.random.nextInt(segments.size()));
      Integer segmentID = pair.getKey();
      WaySegment segment = world.getWaySegment(segmentID);
      Double spawnLength = pair.getValue();

      // traverse segment to find a spawn point
      double traverse_length = spawnLength + this.random.nextDouble() * (this.maxDistanceBtwVehicles - this.minDistanceBtwVehicles);

      if (traverse_length >= segment.length) { // cannot place pose here, but at child // >= not > makes it easier later (no checking if last element)
        // remove segment
        segments.remove(pair);

        // recursively add children
        int childNodeID = inward ? segment.startNodeID : segment.endNodeID;
        if (childNodeID >= 0) { // child is a node
          Node childNode = world.getNode(childNodeID);
          // get new segments
          // spawnOffset depending on the traverse_length
          segments = randomizedSegments(childNode, segments, avoidSegmentIDs, inward, traverse_length - segment.length);
        }

        // try again for vehicle i
        i--;
        continue;
      }
      else { // place pose here: traverse the way to find the nearest point
        // get way
        Way way = segment.way;
        // determine if way-segment is reversed
        boolean reversed = (segment.pointsStart > segment.pointsEnd);
        // travel traverse_length from pointsStart to pointsEnd
        double currDistance = 0;

        // get start point of the segment (if reversed, end point)
        int currPointID = inward ? segment.pointsEnd : segment.pointsStart;
        int inc = reversed ^ inward ? -1 : 1;

        // determine the way-point at which the vehicle will spawn
        // is the way-point at which the vehicle will have traveled traverse_length from the start point of the segment
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
        double distance = currDistance - traverse_length;
        assert(distance >= 0);
        double fraction = 1 - (distance / total_distance);

        // determine accurate point based on linear interpolation
        Vec2 spawnPoint = new Vec2(prevPoint.x + fraction * (currPoint.x - prevPoint.x), prevPoint.y + fraction * (currPoint.y - prevPoint.y));
        Coordinates spawnCoordinates = new Coordinates();
        this.converter.metersToCoords(spawnPoint, spawnCoordinates);

        // determine the orientation of the vehicle at spawnPoint
        double orientation = Math.toDegrees(Math.atan2(spawnPoint.y - prevPoint.y, spawnPoint.x - prevPoint.x)) - 180;

        // add the final pose to the list of poses
        poses.add(new Pair<>(spawnCoordinates, orientation));

        // update segments
        segments.remove(pair);
        segments.add(new Pair<>(segmentID, traverse_length + this.minDistanceBtwVehicles));
      }
    }
    assert (poses.size() == poses_to_generate);
    return poses;
  }

  /*
    Add all adjacent segments from the startNode, excluding the avoidSegments and excludeSegments.
   */
  private Vector<Pair<Integer, Double>> randomizedSegments(Node startNode, Vector<Pair<Integer, Double>> existingSegments, Vector<Integer> avoidSegmentIDs, boolean inward, double spawnOffset) {
    // Additional Safety Precaution: If Node is a junction with very sharp corners. Doesn't guarantee collision-free spawning, but helps a lot.
    spawnOffset = Math.max(spawnOffset, this.minDistanceBtwVehicles * 3 / 4);

    // get all adjacent segments
    Vector<Integer> newSegmentsIDs = inward ? startNode.inRoadSegmentIDs : startNode.outRoadSegmentIDs;

    Vector<Pair<Integer, Double>> segments = new Vector<>(existingSegments);
    for (Integer newSegmentID : newSegmentsIDs) {
      WaySegment segment = this.world.getWaySegment(newSegmentID);
      assert(newSegmentID == segment.localID);

      // do not add segment if it should be excluded
      if (!avoidSegmentIDs.contains(newSegmentID)) {
        segments.add(new Pair<>(newSegmentID, spawnOffset));

        // avoid this segment in the future
        avoidSegmentIDs.add(segment.localID);
        avoidSegmentIDs.add(segment.reverseId);
      }
    }
    return segments;
  }

  @Override
  public Vector<VehicleProperties> randomizeCars(Vector<VehicleProperties> vehicles) {
    VehicleProperties template = vehicles.firstElement();
    Vector<VehicleProperties> generatedVehicles = new Vector<>(generatedStartPoses.size());
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

      // Add the start pose
      newVehicle.start_coords = Optional.of(generatedStartPoses.get(i).getKey());
      newVehicle.start_orientation = generatedStartPoses.get(i).getValue();

      // Create a task to follow the path
      TaskProperties task = new TaskProperties();
      PathGoalProperties goal = new PathGoalProperties();
      goal.ltl_operator = LTLOperator.EVENTUALLY;

      // Add the path points
      for (Node node : generatedPath) {
        goal.reach(node.point.asVec2());
      }

      // Add the end point
      Vec2 endPoint = new Vec2();
      this.converter.coordsToMeters(generatedEndPoses.get(generatedEndPoses.size() - i - 1).getKey(), endPoint); // assign vehicles end poses in reverse order (first vehicle in platoon should have a further distant parking pose)
      goal.reach(endPoint);

      task.addGoal(goal);
      newVehicle.task = task;

      // Add the vehicle to the list
      generatedVehicles.add(newVehicle);
    }
    return generatedVehicles;
  }

  @Override
  public Optional<RewardFunctionProperties> randomizeRewardFunction(Optional<RewardFunctionProperties> rewardFunction) {
    if (rewardFunction.isPresent())
      return rewardFunction;
    return Optional.of(new PlatooningRewardFunctionProperties());
  }

}
