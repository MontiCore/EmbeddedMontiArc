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
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.controller.navigation.navigationBlock.NavigationBlock;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Geometry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.NavigationBlockAsEEComponent;
import simulation.environment.WorldModel;
import simulation.environment.object.ChargingStation;
import simulation.environment.osm.IntersectionFinder;
import simulation.environment.util.ChargingStationNavigator;
import simulation.environment.util.IBattery;
import simulation.environment.util.VehicleType;
import static de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry.*;

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
    private Optional<NavigationBlockAsEEComponent> navigation = Optional.empty();

    /** EEVehicle that models the in Vehicle communication */
    private EEVehicle eeVehicle;

    /** PhysicalVehicle that models the physical part of this vehicle */
    private PhysicalVehicle physicalVehicle;

    /** Internal Attributes */

    /** Manage the battery state of the vehicle */
    private Optional<IBattery> battery;

    /** Battery Stuff */
    /** Flag go to Chargingstation */
    private boolean gotoCharginstation = false;

    RealVector lastdestinationrealvector = null;
    /** Last Destination */
    private IControllerNode lastdestination = null;

    Boolean batteryProblem = false;

    /** Camera image from visualization */
    private Optional<Image> cameraImage;

    /** Maximum temporary allowed velocity of vehicle */
    private double maxTemporaryAllowedVelocity;

    private Instant lastSimulationTime = Instant.EPOCH;

    
    /** For server to keep track of vehicles between sectors */
    protected String globalId;


    /**
     * Constructor for a vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     * @param physicalVehicleBuilder Builder for the physical vehicle that belongs to this vehicle
     * @param eeVehicleBuilder Builder for the ee vehicle that belongs to this vehicle.
     */
    public Vehicle(PhysicalVehicleBuilder physicalVehicleBuilder, EEVehicleBuilder eeVehicleBuilder) {
    	//Create physicalVehicle
        this.physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle(this);
    	//create eeVehicle
        this.eeVehicle = eeVehicleBuilder.buildEEVehicle(this, physicalVehicle);
        initVehicle();
    }

    /**
     * Constructor for a vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     * @param physicalVehicleBuilder Builder for the physical vehicle that belongs to this vehicle
     * @param eeVehicleBuilder Builder for the ee vehicle that belongs to this vehicle.
     * @param eeFile File from which the ee vehicle should be parsed
     */
    public Vehicle(PhysicalVehicleBuilder physicalVehicleBuilder, EEVehicleBuilder eeVehicleBuilder, File eeFile) {
    	//Create physicalVehicle
        this.physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle(this);
    	//create eeVehicle
        this.eeVehicle = eeVehicleBuilder.buildEEVehicle(this, eeFile);
    	initVehicle();
    }

    /**
     * Constructor for a vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     * @param massPointPhysicalVehicleFile File from which the physical vehicle should be parsed
     * @param eeVehicleBuilder Builder for the ee vehicle that belongs to this vehicle.
     */
    public Vehicle(File massPointPhysicalVehicleFile, EEVehicleBuilder eeVehicleBuilder) throws IOException {
    	//Create physicalVehicle
        this.physicalVehicle = new MassPointPhysicalVehicleBuilder().loadFromFile(this, massPointPhysicalVehicleFile);
    	//create eeVehicle
        this.eeVehicle = eeVehicleBuilder.buildEEVehicle(this, physicalVehicle);
        initVehicle();
    }

    /**
     * Constructor for a vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     * @param massPointPhysicalVehicleFile File from which the physical vehicle should be parsed
     * @param eeVehicleBuilder Builder for the ee vehicle that belongs to this vehicle.
     * @param eeFile File from which the ee vehicle should be parsed
     */
    public Vehicle(File massPointPhysicalVehicleFile, EEVehicleBuilder eeVehicleBuilder, File eeFile) throws IOException {
    	//Create physicalVehicle
        this.physicalVehicle = new MassPointPhysicalVehicleBuilder().loadFromFile(this, massPointPhysicalVehicleFile);
    	//create eeVehicle
        this.eeVehicle = eeVehicleBuilder.buildEEVehicle(this, eeFile);
    	initVehicle();
    }

    public void setVehicleType(VehicleType vehicleType, double fuellPercentage) throws Exception {
        physicalVehicle.setVehicleType(vehicleType);
        if (vehicleType == VehicleType.ELECTRIC) {
            setBattery(new Battery(this, 3000000, fuellPercentage));
        }
    }

    /**
     * Common initialisation process
     */
	private void initVehicle() {
    	// Create the status logger
        this.statusLogger = new StatusLogger();
        //Register actuators at physicalVehicle
        this.physicalVehicle.initializeActuators();
        // Initialise camera image with empty optional
        this.cameraImage = Optional.empty();
        // When created, maximum temporary allowed velocity is not limited
        this.maxTemporaryAllowedVelocity = Double.MAX_VALUE;
        this.battery = Optional.empty();
	}

    /**
     * function that is used by EEVehicleBuilder to initialize the navigation
     * @param navigation navigation should be used by the vehicle
     */
    protected void initNavigation(NavigationBlockAsEEComponent navigation) {
        if (this.navigation.isPresent()) {
            throw new IllegalStateException("Navigation can only be set once");
        }
        this.navigation = Optional.of(navigation);

    }

    /**
     * Get the sensor that belongs to type. If multiple exist, return arbitrary one.
     * If no such sensor exist return Optional.Empty()
     * @param type Type of the sensor that should be returned
     * @return Optional of respective sensor or Optional.Empty()
     */
    public Optional<AbstractSensor> getSensorByType(BusEntry type){
    	return this.eeVehicle.getSensorByType(type);
    }

    @Override
    public void executeLoopIteration(Duration timeDiff) {
        this.lastSimulationTime = this.lastSimulationTime.plus(timeDiff);
        this.updateBattery();
        this.eeVehicle.executeLoopIteration(this.lastSimulationTime);
        this.batteryOverride();
        this.physicalVehicle.setCollision(false);
    }

    public void batteryOverride(){
        ChargingStation nearestCS = ChargingStationNavigator.getNearestCS();
        if (nearestCS != null){
            if (isParkedChargingStation(nearestCS)) {
                stopVehicle();
            }
        }
        if(lastdestinationrealvector != null) {
            if (isAtLocation(lastdestinationrealvector)) {
                stopVehicle();
            }
        }

        //      battery discharging failed, cannot accelerate the vehicle
        //      set either motor OR throttle to zero, based on type of the car
            if (batteryProblem) {
            if (physicalVehicle instanceof MassPointPhysicalVehicle) {
                VehicleActuator motor = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).get();
                motor.setActuatorValueTarget(0.0);
            } else {
                VehicleActuator throttle = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE).get();
                throttle.setActuatorValueTarget(0.0);
            }
        }
    }

    private void stopVehicle() {
        if (physicalVehicle instanceof MassPointPhysicalVehicle) {
            VehicleActuator motor = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).get();
            motor.setActuatorValueTarget(0.0);
            VehicleActuator brakesBackLeft = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).get();
            VehicleActuator brakesBackRight = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).get();
            VehicleActuator brakesFrontLeft = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).get();
            VehicleActuator brakesFrontRight = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).get();
            brakesBackLeft.setActuatorValueTarget(brakesBackLeft.getActuatorValueMax());
            brakesBackRight.setActuatorValueTarget(brakesBackRight.getActuatorValueMax());
            brakesFrontLeft.setActuatorValueTarget(brakesFrontLeft.getActuatorValueMax());
            brakesFrontRight.setActuatorValueTarget(brakesFrontRight.getActuatorValueMax());
        } else {
            VehicleActuator throttle = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE).get();
            throttle.setActuatorValueTarget(0.0);
            VehicleActuator brakes = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE).get();
            brakes.setActuatorValueTarget(brakes.getActuatorValueMax());
        }
    }

    public void updateBattery(){
        batteryProblem = false;

        if (!battery.isPresent()) return;
        
        IBattery bat = battery.get();

        
        // check if connection to ChargingStation is established
        if (bat.getChargingStationConnectionStatus() == true ) {
            
            // dummy placeholders for ChargingStation specifications
            double ChargingStationVoltage = 100;
            double ChargingStationAmpere = 1;
            
            // depending on the ChargingStation implementation, 
            // they also can invoke this function themselves
            //      here, for demonstration purposes, we set them to some dummy values
            bat.setVoltageChargingStation(ChargingStationVoltage);
            bat.setAmpereChargingStation(ChargingStationAmpere);
            // leave charging initiation to ChargingStation, not to the Vehicle
            //bat.charge();
        }
        
        // check vehicle type,
        //      set consumption calculation based on the vehicle type
        if (physicalVehicle instanceof MassPointPhysicalVehicle)
            bat.setConsumptionMethod(IBattery.ConsumptionMethod.CONSUMPTION_MASS_VELOCITY);
        else
            bat.setConsumptionMethod(IBattery.ConsumptionMethod.CONSUMPTION_THROTTLE_GEAR);
        
        try {
            bat.discharge();
        }
        catch (IllegalArgumentException e) {
            // this flag would cause motor/throttle to be updated to zero,
            //      exactly at the end of this function
            batteryProblem = true;
        } 

        /**
         * Check Battery state and move to the next Chargingstation if needed
         */
        if (!gotoCharginstation && battery.get().getBatteryPercentage() <= 20) {
            gotoCharginstation = true;
            try {
                List<Double> x1;
                List<Double> y1;
                x1 = ((List<Double>)getSensorByType(PLANNED_TRAJECTORY_X).get().getValue());
                y1 = ((List<Double>)getSensorByType(PLANNED_TRAJECTORY_Y).get().getValue());
                lastdestination = new ControllerNodeImpl(new Point3D( x1.get(x1.size()-1), y1.get(y1.size()-1), 0), ChargingStationNavigator.getNearestOsmNodeFrom(
                        new ArrayRealVector(new double[]{x1.get(x1.size()-1), y1.get(y1.size()-1),0})));
                lastdestinationrealvector = new ArrayRealVector(new double[]{x1.get(x1.size()-1), y1.get(y1.size()-1)});
                long nearestcharg = ChargingStationNavigator.getNearestChargingStation(
                        getGlobalId(),
                        ChargingStationNavigator.getNearestOsmNodeFrom(this.physicalVehicle.getPosition())
                );
                RealVector point3d = ChargingStationNavigator.getPositionOfOsmNode(nearestcharg);
                navigateTo(new ControllerNodeImpl(Geometry.realVector2Point3D(point3d), nearestcharg));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
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
     * Set battery
     * @param battery
     */
    public void setBattery(IBattery battery) {
        this.battery = Optional.of(battery);
    }

    /**
     * @return Battery of the vehicle
     */
    public Optional<IBattery> getBattery() {
        return battery;
    }


    public boolean isParkedChargingStation(ChargingStation station) {
        if(gotoCharginstation){
            RealVector location2D = new ArrayRealVector(new double[]{station.getLocation().getEntry(0),station.getLocation().getEntry(1)});
            RealVector vehiclePos2D = new ArrayRealVector(new double[]{physicalVehicle.getPosition().getEntry(0),physicalVehicle.getPosition().getEntry(1)});
            return location2D.getDistance(vehiclePos2D) < station.getStationRadius();
        }
        return false;
    }

    public boolean isAtLocation(RealVector location2D){
        RealVector vehiclePos2D = new ArrayRealVector(new double[]{physicalVehicle.getPosition().getEntry(0),physicalVehicle.getPosition().getEntry(1)});
        return location2D.getDistance(vehiclePos2D) < 2.0;
    }
    /**
     * @return true if it's an EV
     */
    public boolean isElectricVehicle(){
        return this.battery.isPresent();
    }

    /**
     * Can be called from the Chargingstation
     */
    public void onRechargeReady(){
        gotoCharginstation = false;
        navigateTo(lastdestination);
    }

    public boolean isGotoCharginstation() {
        return gotoCharginstation;
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

    public void navigateTo(IControllerNode target) {
        if(navigation.isPresent()) {
            navigation.get().navigateTo(target);
        }
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

    /**
     * Getter and setter for globalId
     *
     * @return
     */
    public String getGlobalId() {
        return globalId;
    }

    public void setGlobalId(String globalId) {
        this.globalId = globalId;
    }
}
