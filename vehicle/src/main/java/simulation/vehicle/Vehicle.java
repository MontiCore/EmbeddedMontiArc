/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.vehicle;

import commons.controller.commons.BusEntry;
import commons.controller.commons.Vertex;
import commons.controller.interfaces.FunctionBlockInterface;
import commons.map.IControllerNode;
import sensors.CameraSensor;
import sensors.CompassSensor;
import sensors.DayNightSensor;
import sensors.DistanceToLeftSensor;
import sensors.DistanceToRightSensor;
import sensors.LeftBackWheelDistanceToStreetSensor;
import sensors.LeftFrontDistanceSensor;
import sensors.LeftFrontWheelDistanceToStreetSensor;
import sensors.LocationSensor;
import sensors.ObstacleSensor;
import sensors.RightBackWheelDistanceToStreetSensor;
import sensors.RightFrontDistanceSensor;
import sensors.RightFrontWheelDistanceToStreetSensor;
import sensors.SpeedSensor;
import sensors.StaticPlannedTrajectoryXSensor;
import sensors.StaticPlannedTrajectoryYSensor;
import sensors.SteeringAngleSensor;
import sensors.WeatherSensor;
import sensors.abstractsensors.AbstractSensor;


import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.NavigationBlockAsEEComponent;
import simulation.bus.Bus;
import simulation.bus.InstantBus;
import simulation.environment.WorldModel;
import simulation.environment.osm.IntersectionFinder;
import simulation.util.Log;
import simulator.integration.HardwareEmulatorInterface;

import java.awt.*;
import java.time.Instant;
import java.util.*;
import java.util.List;
import static commons.controller.commons.BusEntry.*;

/**
 * Simulation objects for a generic vehicle.
 */
public class Vehicle {
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


    /**
     * Constructor for a vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     */
    public Vehicle(PhysicalVehicle physicalVehicle) {
        // Create the status logger
        this.statusLogger = new StatusLogger();

        //Set physicalVehicle
        this.physicalVehicle = physicalVehicle;
        physicalVehicle.setVehicle(this);
        //Set eeVehicle
        this.eeVehicle = createEEVehicle(physicalVehicle);
        //Register actuators at physical vehicle
        physicalVehicle.initializeActuators();
        //Register navigation at vehicle
        this.navigation = eeVehicle.getNavigation();
        // Initialise camera image with empty optional
        cameraImage = Optional.empty();
        // When created, maximum temporary allowed velocity is not limited
        this.maxTemporaryAllowedVelocity = Double.MAX_VALUE;
    }

    
    private EEVehicle createEEVehicle(PhysicalVehicle physicalVehicle) {
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		Bus bus = new InstantBus(eeSimulator);
		List<EEComponent> components = new ArrayList<EEComponent>();
		
		//create all sensors
		components.add(AbstractSensor.createSensor(CameraSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(CompassSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(DayNightSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(DistanceToLeftSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(DistanceToRightSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(LeftBackWheelDistanceToStreetSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(LeftFrontDistanceSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(LeftFrontWheelDistanceToStreetSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(LocationSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(ObstacleSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(RightBackWheelDistanceToStreetSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(RightFrontDistanceSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(RightFrontWheelDistanceToStreetSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(SpeedSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(StaticPlannedTrajectoryXSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(StaticPlannedTrajectoryYSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(SteeringAngleSensor.class, physicalVehicle, bus).get());
		components.add(AbstractSensor.createSensor(WeatherSensor.class, physicalVehicle, bus).get());

		//create all actuators
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, bus));
        components.add(VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE, bus));

        //create navigation
        components.add(NavigationBlockAsEEComponent.createNavigationBlockAsEEComponent(bus));

        return new EEVehicle(this, eeSimulator, Collections.singletonList(bus), components);
    }
    
    public Optional<AbstractSensor> getSensorByType(BusEntry type){
    	return this.eeVehicle.getSensorByType(type);
    }
    
    public void executeLoopIteration(Instant time) {
    	//update physical vehicle?
    	this.eeVehicle.executeLoopIteration(time);
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
    public Optional<FunctionBlockInterface> getNavigation()
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