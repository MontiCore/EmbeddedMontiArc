package de.rwth.montisim.simulation.simulator.randomization;

import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.WaySegment;
import de.rwth.montisim.simulation.simulator.rewards.BasicRewardFunctionProperties;
import de.rwth.montisim.simulation.simulator.rewards.RewardFunctionProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

import java.util.Optional;
import java.util.Vector;

public class BasicDrivingStrategy extends RandomizationStrategy {

  private final World world;

  private final double minGoalDistance;
  private final double maxGoalDistance;

  private final Pair<Coordinates, Double> startPose;

  private Vector<Node> path;

  public BasicDrivingStrategy(Optional<Long> seed, World world, double minGoalDistance, double maxGoalDistance) {
    super(seed);
    this.world = world;
    SimpleCoordinateConverter converter = world.converter.get();
    this.minGoalDistance = minGoalDistance;
    this.maxGoalDistance = maxGoalDistance;

    // Find a valid path which the vehicle should follow
    Node start = null;
    path = null;
    while (path == null) {
      start = world.getNode(random.nextInt(world.nodes.size()));
      path = traverse(start);
    }

    // calculate starting orientation
    int secondNodeID = path.firstElement().localID;
    int firstSegmentID = start.outRoadSegmentIDs.stream().filter(id -> world.getWaySegment(id).endNodeID == secondNodeID).findFirst().get();
    WaySegment firstSegment = world.getWaySegment(firstSegmentID);
    Vec3 firstPoint = firstSegment.way.points.get(firstSegment.pointsStart);
    boolean reversed = firstSegment.pointsStart > firstSegment.pointsEnd;
    int nextPointIndex = reversed ? firstSegment.pointsStart - 1 : firstSegment.pointsStart + 1;
    Vec3 nextPoint = firstSegment.way.points.get(nextPointIndex);
    double orientation = Math.toDegrees(Math.atan2(nextPoint.y - firstPoint.y, nextPoint.x - firstPoint.x));

    // convert meters to coordinates
    Coordinates startCoordinates = new Coordinates();
    converter.metersToCoords(firstPoint, startCoordinates);

    // set start and end
    this.startPose = new Pair<>(startCoordinates, orientation);
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

  @Override
  public Vector<VehicleProperties> randomizeCars(Vector<VehicleProperties> vehicles) {
    VehicleProperties template = vehicles.firstElement();
    Vector<VehicleProperties> generatedVehicles = new Vector<>(1);
    // Assemble the randomized vehicle
    VehicleProperties newVehicle = new VehicleProperties();
    newVehicle.setName("Car_" + 1);

    // Copy the standard properties from the first vehicle of the config
    newVehicle.components.clear();
    newVehicle.components.addAll(template.components);
    newVehicle.body = template.body;
    newVehicle.wheels = template.wheels;
    newVehicle.physics = template.physics;
    newVehicle.powertrain = template.powertrain;

    // Add randomized properties
    newVehicle.start_coords = Optional.of(startPose.getKey());
    newVehicle.start_orientation = startPose.getValue();

    TaskProperties task = new TaskProperties();
    PathGoalProperties goal = new PathGoalProperties();
    goal.ltl_operator = LTLOperator.EVENTUALLY;

    // add path points
    for (Node node : this.path) {
      goal.reach(node.point.asVec2());
    }
    task.addGoal(goal);
    newVehicle.task = task;

    // Add the vehicle to the list
    generatedVehicles.add(newVehicle);
    return generatedVehicles;
  }

  @Override
  public Optional<RewardFunctionProperties> randomizeRewardFunction(Optional<RewardFunctionProperties> rewardFunction) {
    if (rewardFunction.isPresent())
      return rewardFunction;
    return Optional.of(new BasicRewardFunctionProperties());
  }

}
