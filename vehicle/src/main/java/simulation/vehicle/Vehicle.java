/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.Vertex;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.IControllerNode;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObject;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.controller.navigation.navigationBlock.NavigationBlock;
import org.apache.commons.math3.linear.RealVector;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.NavigationBlockAsEEComponent;

import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.List;
import java.util.*;

/**
 * Simulation objects for a generic vehicle.
 */
public class Vehicle implements SimulationLoopExecutable {
    /** Default average values for vehicle constructor */
    /** Minimum acceleration that can be made by the motor */
    public static final double VEHICLE_DEFAULT_MOTOR_ACCELERATION_MIN = -1.5;

    /** Maximum acceleration that can be made by the motor */
    public static final double VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX = 3.5;

    /** Rate at which the acceleration of the motor can change */
    public static final double VEHICLE_DEFAULT_MOTOR_ACCELERATION_RATE = 2.0;

    /** Minimum acceleration that can be made by the brakes */
    public static final double VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN = 0.0;

    /** Maximum acceleration that can be made by the brakes */
    public static final double VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX = 5.0;

    /** Rate at which the acceleration of the brakes can change */
    public static final double VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE = 5.0;

    /** Minimum steering angle */
    public static final double VEHICLE_DEFAULT_STEERING_ANGLE_MIN = -6;

    /** Maximum steering angle */
    public static final double VEHICLE_DEFAULT_STEERING_ANGLE_MAX = 6;

    /** Rate at which the steering angle can change */
    public static final double VEHICLE_DEFAULT_STEERING_ANGLE_RATE = 3;

    /** Maximum velocity of the vehicle */
    public static final double VEHICLE_DEFAULT_APPROX_MAX_VELOCITY = 100.0;

    /**Brakecyclinder min, max and changerate in KiloPascal*/

    public static final double VEHICLE_DEFAULT_BRAKE_PRESSURE_MIN = 0;
    public static final double VEHICLE_DEFAULT_BRAKE_PRESSURE_RATE = 5000;
    public static final double VEHICLE_DEFAULT_BRAKE_PRESSURE_MAX = 17000;

    /**ClutchPedal min, max and changerate*/
    public static final double VEHICLE_DEFAULT_CLUTCH_POSITION_MIN = 0;
    public static final double VEHICLE_DEFAULT_CLUTCH_POSITION_MAX = 1;
    public static final double VEHICLE_DEFAULT_CLUTCH_POSITION_RATE = 0.5;
    /**ThrottlePedal min, max and changerate*/
    public static final double VEHICLE_DEFAULT_THROTTLE_POSITION_MIN = 0;
    public static final double VEHICLE_DEFAULT_THROTTLE_POSITION_MAX = 1;
    public static final double VEHICLE_DEFAULT_THROTTLE_POSITION_RATE = 0.2;
    /**Gear min, max and changerate*/
    public static final double VEHICLE_DEFAULT_GEAR_MIN = 0;
    public static final double VEHICLE_DEFAULT_GEAR_MAX = 5;
    public static final double VEHICLE_DEFAULT_GEAR_RATE = 1;

    /** Status logging module */
    private StatusLogger statusLogger;

    /** Navigation for vehicle */
    private Optional<NavigationBlockAsEEComponent> navigation;

    /** EEVehicle that models the in Vehicle communication */
    private EEVehicle eeVehicle;

    /** PhysicalVehicle that models the physical part of this vehicle */
    private PhysicalVehicle physicalVehicle;

    /** Internal Attributes */

    /** Camera image from visualization */
    private Optional<Image> cameraImage;

    /** Maximum temporary allowed velocity of vehicle */
    private double maxTemporaryAllowedVelocity;

    private Instant lastSimulationTime = Instant.EPOCH;


    /**
     * Constructor for a vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     */
    public Vehicle(PhysicalVehicleBuilder physicalVehicleBuidler, EEVehicleBuilder eeVehicleBuilder) {
    	//Create physicalVehicle
        this.physicalVehicle = physicalVehicleBuidler.buildPhysicalVehicle(this);
    	//create eeVehicle
        this.eeVehicle = eeVehicleBuilder.buildEEVehicle(this, physicalVehicle);
        initVehicle();
    }

    public Vehicle(PhysicalVehicleBuilder physicalVehicleBuidler, EESimulator simulator, File file) {
    	//Create physicalVehicle
        this.physicalVehicle = physicalVehicleBuidler.buildPhysicalVehicle(this);
    	//create eeVehicle
        this.eeVehicle = new EEVehicle(this, simulator, file);
    	initVehicle();
    }
    
    public Vehicle(File massPointPhysicalVehicleFile, EEVehicleBuilder eeVehicleBuilder) throws IOException {
    	//Create physicalVehicle
        this.physicalVehicle = new MassPointPhysicalVehicleBuilder().loadFromFile(this, massPointPhysicalVehicleFile);
    	//create eeVehicle
        this.eeVehicle = eeVehicleBuilder.buildEEVehicle(this, physicalVehicle);
        initVehicle();
    }

    public Vehicle(File massPointPhysicalVehicleFile, EESimulator simulator, File eeFile) throws IOException {
    	//Create physicalVehicle
        this.physicalVehicle = new MassPointPhysicalVehicleBuilder().loadFromFile(this, massPointPhysicalVehicleFile);
    	//create eeVehicle
        this.eeVehicle = new EEVehicle(this, simulator, eeFile);
    	initVehicle();
    }

	private void initVehicle() {
    	// Create the status logger
        this.statusLogger = new StatusLogger();
        // register the navigation unit
        this.navigation = eeVehicle.getNavigation();
        //Register actuators at physicalVehicle
        this.physicalVehicle.initializeActuators();
        // Initialise camera image with empty optional
        this.cameraImage = Optional.empty();
        // When created, maximum temporary allowed velocity is not limited
        this.maxTemporaryAllowedVelocity = Double.MAX_VALUE;
	}

    public Optional<AbstractSensor> getSensorByType(BusEntry type){
    	return this.eeVehicle.getSensorByType(type);
    }

    @Override
    public void executeLoopIteration(Duration timeDiff) {

        this.lastSimulationTime = this.lastSimulationTime.plus(timeDiff);
        this.eeVehicle.executeLoopIteration(this.lastSimulationTime);
        this.physicalVehicle.setCollision(false);
    }

	public EEVehicle getEEVehicle() {
		return this.eeVehicle;
	}

	public PhysicalVehicle getPhysicalVehicle() {
		return this.physicalVehicle;
	}

	public boolean isInitialized() {
			return this.physicalVehicle.getPhysicalVehicleInitialised();
	}

    /**
     * Function that returns the status logger of the vehicle
     *
     * @return ToDo
     */
    public StatusLogger getStatusLogger() {
        return statusLogger;
    }

    /**
     * Function that sets the status logger of the vehicle
     *
     * @param statusLogger ToDo
     */
    public void setStatusLogger(StatusLogger statusLogger) {
        this.statusLogger = statusLogger;
    }

    /**
     * Function that returns the optional navigation
     *
     * @return Optional navigation of the vehicle
     */
    public Optional<NavigationBlock> getNavigation()
    {
        if(navigation.isPresent()) {
            return Optional.of(navigation.get().getNavigationBlock());
        }
        return Optional.empty();
    }


    /**
     * Function that returns the last navigation target of the vehicle
     *
     * @return ToDo
     */
    public Optional<IControllerNode> getLastNavigationTarget() {
        if(navigation.isPresent()) {
            return navigation.get().getLastNavigationTarget();
        }
        return Optional.empty();
    }

    /**
     * Function that gets the camera image of the vehicle
     *
     * @return Optional of the current camera image
     */
    public Optional<Image> getCameraImage() {
        return cameraImage;
    }

    /**
     * Function that sets the camera image
     *
     * @param cameraImage Optional of the new camera image to be stored
     */
    public void setCameraImage(Optional<Image> cameraImage) {
        this.cameraImage = cameraImage;
    }

    /**
     * Function that returns the maximum temporary allowed velocity of simulated car
     *
     * @return Maximum temporary allowed velocity
     */
    public double getMaxTemporaryAllowedVelocity() {
        return maxTemporaryAllowedVelocity;
    }

    /**
     * Function that sets the maximum temporary allowed velocity of simulated car
     *
     * @param maxTemporaryAllowedVelocity New maximum temporary allowed velocity of simulated car
     */
    public void setMaxTemporaryAllowedVelocity(double maxTemporaryAllowedVelocity) {
        this.maxTemporaryAllowedVelocity = maxTemporaryAllowedVelocity;
    }

    //TODO: What is it and do we still need it?
    /**
     * Get current trajectory of the vehicle, if available. Otherwise return empty list.
     *
     * @return Current trajectory of the vehicle, if not available return empty list
     */
    public List<Vertex> getTrajectory() {
        if(navigation.isPresent()) {
            return navigation.get().getTrajectory();
        }
        return new ArrayList<>();
    }

	public void navigateTo(IControllerNode target, LinkedList<RealVector> avoidCoordinates) {
        if(navigation.isPresent()) {
            navigation.get().navigateTo(target, avoidCoordinates);
        }
	}

    /**
     * Get nearest position that is located on the ordered trajectory
     *
     * @param inputTrajectory Input trajectory for which the nearest position should be computed
     * @param position Source position from which the nearest trajectory position should to be computed
     * @param accuracy Positive integer, the higher the more accurate the result is, 20 is recommended
     * @return Map entry containing: Integer of nextVertex index in trajectory and RealVector of nearest position that is located on the input trajectory
     * @throws IllegalArgumentException if input trajectory is empty
     */
    public static Map.Entry<Integer, RealVector> getNearestPositionOnTrajectory(List<Vertex> inputTrajectory, RealVector position, int accuracy) {
        // Exception for empty input trajectory
        if (inputTrajectory.isEmpty()) {
            throw new IllegalArgumentException("Vehicle - getNearestPositionOnTrajectory: inputTrajectory is empty!");
        }

        // Exception for invalid accuracy value
        if (accuracy <= 0) {
            throw new IllegalArgumentException("Vehicle - getNearestPositionOnTrajectory: Accuracy has invalid value " + accuracy);
        }

        List<RealVector> positionList = Collections.synchronizedList(new LinkedList<>());
        for (int i = 0; i < inputTrajectory.size(); ++i) {
            RealVector pos = inputTrajectory.get(i).getPosition();
            positionList.add(pos.copy());
        }

        return getNearestPositionOnPositionList(positionList, position, accuracy);
    }

    /**
     * Get nearest position that is located on the ordered input positions list
     *
     * @param inputPositions Input list of positions for which the nearest position should be computed
     * @param position Source position from which the nearest trajectory position should to be computed
     * @param accuracy Positive integer, the higher the more accurate the result is, 20 is recommended
     * @return Map entry containing: Integer of nextVertex index in trajectory and RealVector of nearest position that is located on the input trajectory
     * @throws IllegalArgumentException if input trajectory is empty
     */
    public static Map.Entry<Integer, RealVector> getNearestPositionOnPositionList(List<RealVector> inputPositions, RealVector position, int accuracy) {
        // Exception for empty input positions
        if (inputPositions.isEmpty()) {
            throw new IllegalArgumentException("Vehicle - getNearestPositionOnTrajectory: inputTrajectory is empty!");
        }

        // Exception for invalid accuracy value
        if (accuracy <= 0) {
            throw new IllegalArgumentException("Vehicle - getNearestPositionOnTrajectory: Accuracy has invalid value " + accuracy);
        }

        // Find index of nearest trajectory vertex
        int nearestVertex = 0;
        double minDist = Double.MAX_VALUE;

        for (int i = 0; i < inputPositions.size(); ++i) {
            RealVector v = inputPositions.get(i);
            double distance = ((v.getEntry(0) - position.getEntry(0)) * (v.getEntry(0) - position.getEntry(0)))
                            + ((v.getEntry(1) - position.getEntry(1)) * (v.getEntry(1) - position.getEntry(1)))
                            + ((v.getEntry(2) - position.getEntry(2)) * (v.getEntry(2) - position.getEntry(2)));

            if (distance < minDist) {
                minDist = distance;
                nearestVertex = i;
            }
        }

        // Find second nearest trajectory vertex
        int secondNearestVertex = 0;
        double secondMinDist = Double.MAX_VALUE;

        for (int i = 0; i < inputPositions.size(); ++i) {
            RealVector v = inputPositions.get(i);
            double distance = ((v.getEntry(0) - position.getEntry(0)) * (v.getEntry(0) - position.getEntry(0)))
                            + ((v.getEntry(1) - position.getEntry(1)) * (v.getEntry(1) - position.getEntry(1)))
                            + ((v.getEntry(2) - position.getEntry(2)) * (v.getEntry(2) - position.getEntry(2)));

            if (distance < secondMinDist && i != nearestVertex) {
                secondMinDist = distance;
                secondNearestVertex = i;
            }
        }

        // Compute next and prev vertex on trajectory and use them to get nearest point on trajectory
        int nextVertex = Math.max(nearestVertex, secondNearestVertex);
        int prevVertex = Math.min(nearestVertex, secondNearestVertex);
        RealVector nextVertexPos = inputPositions.get(nextVertex);
        RealVector prevVertexPos = inputPositions.get(prevVertex);
        RealVector fromPrevToNext = nextVertexPos.subtract(prevVertexPos);

        RealVector nearestPosOnTrajectory = prevVertexPos.copy();
        double minDistNearestPos = Double.MAX_VALUE;

        for (int i = 0; i <= accuracy; ++i) {
            double factor = (double)(i) / (double)(accuracy);
            RealVector possibleNearestPoint = prevVertexPos.add(fromPrevToNext.mapMultiply(factor));

            double distance = ((possibleNearestPoint.getEntry(0) - position.getEntry(0)) * (possibleNearestPoint.getEntry(0) - position.getEntry(0)))
                            + ((possibleNearestPoint.getEntry(1) - position.getEntry(1)) * (possibleNearestPoint.getEntry(1) - position.getEntry(1)))
                            + ((possibleNearestPoint.getEntry(2) - position.getEntry(2)) * (possibleNearestPoint.getEntry(2) - position.getEntry(2)));

            if (distance < minDistNearestPos) {
                minDistNearestPos = distance;
                nearestPosOnTrajectory = possibleNearestPoint.copy();
            }
        }

        // Return final result
        return new AbstractMap.SimpleEntry<>(nextVertex, nearestPosOnTrajectory);
    }


    /**
     * Overwrite toString() to get a nice output for vehicles
     *
     * @return String that contains all information of vehicles
     */
    @Override
    public String toString() {
        return "Vehicle " + hashCode() + ": length: " +
                " , maxTemporaryAllowedVelocity: " + maxTemporaryAllowedVelocity +
                " , navigation:" + navigation +
                " , cameraImage:" + cameraImage;
    }
}
