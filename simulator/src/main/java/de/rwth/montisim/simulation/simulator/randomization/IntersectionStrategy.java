package de.rwth.montisim.simulation.simulator.randomization;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Vector;
import java.util.stream.Collectors;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.LTLOperator;
import de.rwth.montisim.commons.utils.SimpleCoordinateConverter;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

public class IntersectionStrategy extends RandomizationStrategy {

    public int maxNumberOfVehicles;
    public int minNumberOfVehicles;
    public double minDistanceBtwVehicles;
    public double maxDistanceBtwVehicles;
    public double minDistanceFromIntersection;
    public double maxDistanceFromIntersection;
    public double minGoalDistance;
    public double maxGoalDistance;

    private Vector<Coordinates> generatedStartPos;
    private Vector<Double> generatedStartOrient;
    private Vector<Vec2> generatedGoalPos;

    SimpleCoordinateConverter converter;

    public IntersectionStrategy(Optional<Long> seed, World world, int maxNumberOfVehicles, int minNumberOfVehicles,
                                double minDistanceBtwVehicles, double maxDistanceBtwVehicles, double minDistanceFromIntersection,
                                double maxDistanceFromIntersection, double minGoalDistance, double maxGoalDistance) {
        super(seed);
        this.maxNumberOfVehicles = maxNumberOfVehicles;
        this.minNumberOfVehicles = minNumberOfVehicles;
        this.minDistanceBtwVehicles = minDistanceBtwVehicles;
        this.maxDistanceBtwVehicles = maxDistanceBtwVehicles;
        this.minDistanceFromIntersection = minDistanceFromIntersection;
        this.maxDistanceFromIntersection = maxDistanceFromIntersection;
        this.minGoalDistance = minGoalDistance;
        this.maxGoalDistance = maxGoalDistance;

        if (world.converter.isPresent()) {
            converter = world.converter.get();
        } else {
            return;
        }

        Random rnd = new Random(System.nanoTime());
        // Select a random intersection
        List<Node> intersections = world.nodes.stream().filter((node) -> node.ways.size() > 2).collect(Collectors.toList());
        Node intersection = intersections.get(rnd.nextInt(intersections.size()));

        // Select a random number of vehicles
        int numbOfVehicles = rnd.nextInt(maxNumberOfVehicles - minNumberOfVehicles) + minNumberOfVehicles;
        generatedStartPos = new Vector<>(numbOfVehicles);
        generatedStartOrient = new Vector<>(numbOfVehicles);
        generatedGoalPos = new Vector<>(numbOfVehicles);
        Vector<Double> capacities = new Vector<Double>(intersection.ways.size());
        for (int i = 0; i < intersection.ways.size(); i++) {
            capacities.add(Double.valueOf(minDistanceFromIntersection
                    + rnd.nextDouble() * maxDistanceFromIntersection - minDistanceFromIntersection));
        }
        // Spawn each vehicle on a random way and select a random goal
        for (int i = 0; i < numbOfVehicles; i++) {
            int index = rnd.nextInt(intersection.ways.size());
            Way way = intersection.ways.get(index);
            Vec3 p = way.points.get(0);
            boolean reversed = p.equals(intersection.point) ? false : true;
            double newCapacity = placeVehicle(capacities.get(index).doubleValue(), intersection.ways, index, reversed, rnd);
            capacities.set(index, newCapacity);
        }
    }

    public double placeVehicle(double capacity, List<Way> ways, int index, boolean reversed, Random rnd) {
        Way way = ways.get(index);
        Vec3 startPosition = null;
        Vec3 goalPosition;
        double startOrientation = 0;
        double currCapacity = 0;

        // Find start position
        for (int i = 0; i < way.points.size() - 1; i++) {
            Vec3 p1;
            Vec3 p2;
            if (reversed) {
                p1 = way.points.get(way.points.size() - 1 - i);
                p2 = way.points.get(way.points.size() - 2 - i);
            } else {
                p1 = way.points.get(i);
                p2 = way.points.get(i + 1);
            }

            // Calculate distance between current and next segment
            double distance = p1.distance(p2);
            // Find a point on this segment or move on to the next segment
            if (capacity - currCapacity > distance) {
                currCapacity += distance;
            } else {
                startPosition = p1.add(p2.subtract(p1).normalize().multiply(distance));
                startOrientation = p2.subtract(p1).angle(new Vec3(1, 0, 0));
                break;
            }
        }
        // If no position was found return -1
        if (startPosition == null) return -1;

        // Find a goal position
        int goalWayIndex = rnd.nextInt(ways.size() - 1);
        if (goalWayIndex == index) goalWayIndex++;
        goalPosition = new Vec3(0, 0, 0); //TODO: Implement the actual goal position here

        // Add generated vehicle
        Coordinates startCoords = new Coordinates();
        converter.metersToCoords(startPosition, startCoords);
        generatedStartPos.add(startCoords);
        generatedStartOrient.add(startOrientation);
        generatedGoalPos.add(new Vec2(goalPosition.x, goalPosition.y));

        // Return new capacity
        return capacity + minDistanceBtwVehicles + rnd.nextDouble() * (maxDistanceBtwVehicles - minDistanceBtwVehicles);
    }

    @Override
    public Vector<VehicleProperties> randomizeCars(Vector<VehicleProperties> vehicles) {
        VehicleProperties template = vehicles.firstElement();
        Vector<VehicleProperties> generatedVehicles = new Vector<VehicleProperties>(generatedStartPos.size());
        for (int i = 0; i < generatedStartPos.size(); i++) {

            // Assemble the randomized vehicle
            VehicleProperties newVehicle = new VehicleProperties();
            newVehicle.setName("Car_" + i);

            // Copy the standard properties from the first vehicle of the config
            newVehicle.body = template.body;
            newVehicle.wheels = template.wheels;
            newVehicle.physics = template.physics;
            newVehicle.powertrain = template.powertrain;
            newVehicle.components.clear();
            newVehicle.components.addAll(template.components);

            // Add randomized properties
            newVehicle.start_coords = Optional.of(generatedStartPos.get(i));
            newVehicle.start_orientation = generatedStartOrient.get(i);
            TaskProperties task = new TaskProperties();
            PathGoalProperties goal = new PathGoalProperties();
            goal.ltl_operator = LTLOperator.EVENTUALLY;
            Vector<Vec2> path = new Vector<>(1);
            path.add(generatedGoalPos.get(i));
            goal.path = path;
            task.addGoal(goal);
            newVehicle.task = task;

            // Add the vehicle to the list
            generatedVehicles.add(newVehicle);

        }
        return generatedVehicles;
    }

}
