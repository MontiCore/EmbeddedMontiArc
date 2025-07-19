package de.rwth.montisim.simulation.simulator.randomization;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;
import java.util.stream.Collectors;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.WaySegment;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

public class IntersectionStrategy extends RandomizationStrategy {
  
  private final static double MIN_SEGMENT_LENGTH = 10;
  private final static double DISTANCE_TO_MIDDLE = 0;
  
  public int maxNumberOfVehicles;
  public int minNumberOfVehicles;
  public double minDistanceBtwVehicles;
  public double maxDistanceBtwVehicles;
  public double minDistanceFromIntersection;
  public double maxDistanceFromIntersection;
  
  private Vector<VehicleBlueprint> vehicleBlueprints;
  
  private World world;
  
  SimpleCoordinateConverter converter;
  
  private class VehicleBlueprint {
    
    public Coordinates startCoords;
    public Double startOrientation;
    public Vec2 goalPosition;
    public Double distanceFromIntersection;
    public int segmentID;
    
  }
  
  public IntersectionStrategy(Optional<Long> seed, World world, int maxNumberOfVehicles, int minNumberOfVehicles, 
      double minDistanceBtwVehicles, double maxDistanceBtwVehicles, double minDistanceFromIntersection,
      double maxDistanceFromIntersection) {
    
    super(seed);
    
    this.maxNumberOfVehicles = maxNumberOfVehicles;
    this.minNumberOfVehicles = minNumberOfVehicles;
    this.minDistanceBtwVehicles = minDistanceBtwVehicles;
    this.maxDistanceBtwVehicles = maxDistanceBtwVehicles;
    this.minDistanceFromIntersection = minDistanceFromIntersection;
    this.maxDistanceFromIntersection = maxDistanceFromIntersection;
    this.world = world;
    
    if (world.converter.isPresent()) {
      converter = world.converter.get();
    } else {
      return;
    }
    
    // Select a random intersection
    List<Node> intersections = world.nodes.stream().filter((node) -> node.ways.size() > 2).collect(Collectors.toList());
    Node intersection = intersections.get(random.nextInt(intersections.size()));
    
    System.out.println("Intersection: " + intersection.point.x + " : " + intersection.point.y);
    
    // Select a random number of vehicles
    int numbOfVehicles = random.nextInt((maxNumberOfVehicles + 1) - minNumberOfVehicles) + minNumberOfVehicles;
    vehicleBlueprints = new Vector<>(numbOfVehicles);
    
    // Initialize segments
    Map<Integer, Double> capacities = new TreeMap<>();
    Map<Integer, Double> distances = new TreeMap<>();
    Set<Integer> segments = new TreeSet<>();
    
    segments.addAll(intersection.inRoadSegmentIDs);
    
    // Initialize capacities of existing segments
    for (Integer segmentID : segments) {
      WaySegment segment = world.getWaySegment(segmentID);
      capacities.put(segmentID, segment.length - minDistanceFromIntersection -  
          random.nextDouble() * (maxDistanceFromIntersection - minDistanceFromIntersection));
      distances.put(segmentID, 0d);
    }
    
    // Spawn each vehicle on a random segment and select a random goal
    for (int i = 0; i < numbOfVehicles; i++) {
      placeVehicle(segments, capacities, distances);
      // Stop if no segments are available
      if (segments.isEmpty()) break;
    }
    
    // Sort the vehicles by their distance to the intersection
    Collections.sort(vehicleBlueprints, Collections.reverseOrder(
        (v1, v2) -> Double.compare(v1.distanceFromIntersection, v2.distanceFromIntersection)));
   
    // Reset segments
    capacities = new TreeMap<>();
    segments = new TreeSet<>();
    
    segments.addAll(intersection.outRoadSegmentIDs);
    
    // Initialize capacities of existing segments
    for (Integer segmentID : segments) {
      WaySegment segment = world.getWaySegment(segmentID);
      capacities.put(segmentID, segment.length - minDistanceFromIntersection -  
          random.nextDouble() * (maxDistanceFromIntersection - minDistanceFromIntersection));
    }
    
    // Select the goals for each vehicle
    for (int i = 0; i < numbOfVehicles; i++) {
      VehicleBlueprint veh = vehicleBlueprints.get(i);
      WaySegment vehSegment = world.getWaySegment(veh.segmentID);
      /* A simple strategy to prevent cars from spawning at their 
       * goal is to remove their "reversed" roadsegment from the 
       * set of potential spawn locations.
       */
      int revID = vehSegment.reverseId;
      boolean remove = segments.contains(revID);
      if (remove) {
        segments.remove(revID);
      }
      veh.goalPosition = placeGoal(segments, capacities);
      Vec2 startPosition = new Vec2();
      converter.coordsToMeters(veh.startCoords, startPosition);
      if (veh.goalPosition.distance(startPosition) < 0.5) {
        System.out.println("[Warning]: " + veh.goalPosition + " and " + startPosition + " are too close");
      }
      if (veh.goalPosition == null) {
        System.out.println("[Warning]: One Vehicle was removed");
        vehicleBlueprints.remove(i);
        i--;
      }
      if (remove) {
        segments.add(revID);
      }
    }
    
  }
  
  public Vec2 placeGoal(Set<Integer> segments, Map<Integer, Double> capacities) {
    
    if (segments.isEmpty()) return null;
    
    Vec2 goalPosition = null;
    
    Integer segmentID = getRandomSetElement(segments);
    WaySegment segment = world.getWaySegment(segmentID);
    
    // If the segment is too small, add its successors and take a random other segment
    while (capacities.get(segmentID) < MIN_SEGMENT_LENGTH) {
      segments.remove(segmentID);
      if (segment.endNodeID != -1) {
        for(Integer successorID :  world.getNode(segment.endNodeID).outRoadSegmentIDs) {
          if (!capacities.containsKey(successorID)) {
            segments.add(successorID);
            capacities.put(successorID, world.getWaySegment(successorID).length - minDistanceFromIntersection -  
                random.nextDouble() * (maxDistanceFromIntersection - minDistanceFromIntersection));
          }
        }
      }
      if (segments.isEmpty()) return null;
      segmentID = getRandomSetElement(segments);
      segment = world.getWaySegment(segmentID);
    }
    
    // Find the points of the segment
    Vector<Vec3> segmentPoints = new Vector<Vec3> (segment.way.points.subList(
                                                    Math.min(segment.pointsStart, segment.pointsEnd), 
                                                    Math.max(segment.pointsStart, segment.pointsEnd) + 1 ));
    // Make sure the order of the points is reversed
    if (segment.pointsStart > segment.pointsEnd) Collections.reverse(segmentPoints);
    
    // Find the next available point on the current segment
    
    int i1 = 0;
    int i2 = 1;
    
    Vec2 p1 = new Vec2(segmentPoints.get(i1));
    Vec2 p2 = new Vec2(segmentPoints.get(i2));
    
    double distance = p2.distance(p1);
    double distanceLeft = segment.length - capacities.get(segmentID);
    
    while (distanceLeft > distance) {
      distanceLeft -= distance;
      i1++;
      i2++;
      p1 = new Vec2(segmentPoints.get(i1));
      p2 = new Vec2(segmentPoints.get(i2));
      distance = p1.distance(p2);
    }
    
    // Calculate the next goal
    Vec2 diff = p1.subtract(p2).normalize();
    Vec2 normRight = new Vec2(diff.y, -diff.x);
    
    goalPosition = p1.add(p2.subtract(p1).normalize().multiply(distanceLeft)).add(normRight.multiply(DISTANCE_TO_MIDDLE));
    
    // Update capacity
    capacities.put(segmentID, capacities.get(segmentID) - minDistanceBtwVehicles - 
        random.nextDouble() * (maxDistanceBtwVehicles - minDistanceBtwVehicles));
    
    // return generated goal
    return goalPosition;
  }
  
  public void placeVehicle(Set<Integer> segments, Map<Integer, Double> capacities, 
      Map<Integer, Double> distances) {
    
    Vec2 startPosition = null;
    double startOrientation = 0;

    Integer segmentID = getRandomSetElement(segments);
    WaySegment segment = world.getWaySegment(segmentID);
    
    // If the segment is too small, add its predecessors and take a random other segment
    while (capacities.get(segmentID) < MIN_SEGMENT_LENGTH) {
      segments.remove(segmentID);
      if (segment.startNodeID != -1) {
        for(Integer predecessorID :  world.getNode(segment.startNodeID).inRoadSegmentIDs) {
          if (!capacities.containsKey(predecessorID)) {
            segments.add(predecessorID);
            capacities.put(predecessorID, world.getWaySegment(predecessorID).length - minDistanceFromIntersection -  
                random.nextDouble() * (maxDistanceFromIntersection - minDistanceFromIntersection));
            distances.put(predecessorID, segment.length + distances.get(segmentID));
          }
        }
      }
      if (segments.isEmpty()) return;
      segmentID = getRandomSetElement(segments);
      segment = world.getWaySegment(segmentID);
    }
    
    // Find the points of the segment
    Vector<Vec3> segmentPoints = new Vector<Vec3> (segment.way.points.subList(
                                                    Math.min(segment.pointsStart, segment.pointsEnd), 
                                                    Math.max(segment.pointsStart, segment.pointsEnd) + 1 ));
    // Make sure the order of the points is reversed
    if (segment.pointsStart < segment.pointsEnd) Collections.reverse(segmentPoints);
    
    // Find the next available point on the current segment
    
    int i1 = 0;
    int i2 = 1;
    
    Vec2 p1 = new Vec2(segmentPoints.get(i1));
    Vec2 p2 = new Vec2(segmentPoints.get(i2));
    
    double distance = p1.distance(p2);
    double distanceLeft = segment.length - capacities.get(segmentID);
    
    while (distanceLeft > distance) {
      distanceLeft -= distance;
      i1++;
      i2++;
      p1 = new Vec2(segmentPoints.get(i1));
      p2 = new Vec2(segmentPoints.get(i2));
      distance = p1.distance(p2);
    }
    
    // Calculate the next starting point
    Vec2 diff = p1.subtract(p2).normalize();
    Vec2 normRight = new Vec2(diff.y, -diff.x);
    
    startPosition = p1.add(p2.subtract(p1).normalize().multiply(distanceLeft)).add(normRight.multiply(DISTANCE_TO_MIDDLE));
    
    // Calculate the orientation
    Vec2 right = new Vec2(1, 0);
    Vec2 up = new Vec2(0, 1);
    startOrientation = diff.angle(right) / Math.PI * 180;
    if (diff.dotProduct(up) < 0) startOrientation = 360 - startOrientation;
    
    // Calculate distance from intersection
    double distFromIntersection = distances.get(segmentID) + segment.length - capacities.get(segmentID);
    
    // Update capacity
    capacities.put(segmentID, capacities.get(segmentID) - minDistanceBtwVehicles - 
        random.nextDouble() * (maxDistanceBtwVehicles - minDistanceBtwVehicles));
    
    // Add generated vehicle
    Coordinates startCoords = new Coordinates();
    converter.metersToCoords(startPosition, startCoords);
    VehicleBlueprint veh = new VehicleBlueprint();
    veh.startCoords = startCoords;
    veh.startOrientation = startOrientation;
    veh.distanceFromIntersection = distFromIntersection;
    veh.segmentID = segmentID;
    vehicleBlueprints.add(veh);
  }
  
  @Override
  public Vector<VehicleProperties> randomizeCars(Vector<VehicleProperties> vehicles) {
    VehicleProperties template = vehicles.firstElement();
    Vector<VehicleProperties> generatedVehicles = new Vector<VehicleProperties>(vehicleBlueprints.size());
    for (int i = 0; i < vehicleBlueprints.size(); i++) {
      
      // Assemble the randomized vehicle
      VehicleProperties newVehicle = new VehicleProperties();
      newVehicle.setName("Car_" + i);
      
      System.out.println("Car_" + i + ":\n");
      Vec2 startPos = new Vec2();
      converter.coordsToMeters(vehicleBlueprints.get(i).startCoords, startPos);
      System.out.println("  startPos: " + startPos);
      System.out.println("  goalPos:  " + vehicleBlueprints.get(i).goalPosition);
      
      
      // Copy the standard properties from the first vehicle of the config
      newVehicle.body = template.body;
      newVehicle.wheels = template.wheels;
      newVehicle.physics = template.physics;
      newVehicle.powertrain = template.powertrain;
      newVehicle.components.clear();
      newVehicle.components.addAll(template.components);
      
      // Add randomized properties
      newVehicle.start_coords = Optional.of(vehicleBlueprints.get(i).startCoords);
      newVehicle.start_orientation = vehicleBlueprints.get(i).startOrientation;
      TaskProperties task = new TaskProperties();
      PathGoalProperties goal = new PathGoalProperties();
      goal.ltl_operator = LTLOperator.EVENTUALLY;
      Vector<Vec2> path = new Vector<>(1);
      path.add(vehicleBlueprints.get(i).goalPosition);
      goal.path = path;
      task.addGoal(goal);
      newVehicle.task = task;
      
      // Add the vehicle to the list
      generatedVehicles.add(newVehicle);
     
    }
    return generatedVehicles;
  }
  
  private <E> E getRandomSetElement(Set<E> set) {
    return set.stream().skip(random.nextInt(set.size())).findFirst().orElse(null);
  }
  
}
