/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

import commons.controller.commons.BusEntry;
import commons.controller.commons.NavigationEntry;
import commons.controller.commons.Surface;
import commons.controller.commons.Vertex;
import commons.controller.interfaces.Bus;
import commons.controller.interfaces.FunctionBlockInterface;
import commons.map.ControllerNode;
import commons.map.IAdjacency;
import commons.map.IControllerNode;
import commons.simulation.Sensor;
import commons.utils.Geometry;
import de.topobyte.osm4j.core.model.iface.OsmNode;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.WorldModel;
import simulation.environment.object.ChargingStation;
import simulation.environment.osm.IntersectionFinder;
import simulation.environment.util.Chargeable;
import simulation.environment.util.ChargingStationNavigator;
import simulation.environment.util.IBattery;
import simulation.util.Log;
import java.awt.*;
import java.util.*;
import java.util.List;
import static commons.controller.commons.BusEntry.*;
import static simulation.vehicle.VehicleActuatorType.*;

/**
 * Simulation objects for a generic vehicle.
 */
public class Vehicle{

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
    /** Length of the vehicle in meters */
    public static final double VEHICLE_DEFAULT_LENGTH = 4.236423828125;

    /** Width of the vehicle in meters */
    public static final double VEHICLE_DEFAULT_WIDTH = 2.02712567705637776;

    /** Height of the vehicle in meters */
    public static final double VEHICLE_DEFAULT_HEIGHT = 1.19524474896355328;

    /** Radius of the wheels in meters */
    public static final double VEHICLE_DEFAULT_WHEEL_RADIUS = 0.3334;

    /** Mass of the vehicle */
    public static final double VEHICLE_DEFAULT_MASS = 1800.0;

    /** Distance between the left and the right wheels in meters */
    public static final double VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_FRONT = 1.62025;

    /**Track width rear wheels in meters */
    public static final double VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_REAR = 1.505;

    /** Distance between front and back wheels in meters */
    public static final double VEHICLE_DEFAULT_WHEEL_DIST_TO_FRONT = 1.379;

    /** Distance between back wheels and center of mass in meters */
    public static final double VEHICLE_DEFAULT_WHEEL_DIST_TO_BACK = 1.542;


    /**Components */
    /** Motor of vehicle */
    private VehicleActuator motor;

    /** Brakes of vehicle */
    private VehicleActuator brakesFrontLeft;
    private VehicleActuator brakesFrontRight;
    private VehicleActuator brakesBackLeft;
    private VehicleActuator brakesBackRight;

    /** Steering of vehicle */
    private VehicleActuator steering;

    /**Inputs of vehicle from controller*/
    private  VehicleActuator gear;
    private  VehicleActuator clutch;
    private  VehicleActuator brakes;
    private  VehicleActuator throttle;

    /** Status logging module */
    private StatusLogger statusLogger;

    /** List of all the sensors of the vehicle */
    private List<Sensor> sensorList;

    /** Manage the battery state of the vehicle */
    private Optional<IBattery> battery;

    /** Bus for the controller */
    private Optional<Bus> controllerBus;

    /** Controller for vehicle */
    private Optional<FunctionBlockInterface> controller;

    /** Navigation for vehicle */
    private Optional<FunctionBlockInterface> navigation;

    /** PhysicalVehicle that this vehicle is part of */
    private PhysicalVehicle physicalVehicle;

    /** Battery Stuff */
    /** Flag go to Chargingstation */
    private boolean gotoCharginstation = false;

    /** Last Destination */
    private IControllerNode lastdestination;

    /** Properties */
    /** M of formula */
    /** Mass of the vehicle */
    private double mass;

    /** Dimensions of vehicle in meters */
    private double width, length, height;

    /** Radius of vehicle wheels */
    private double wheelRadius;

    /** Track of the vehicle wheels at the front axel */
    private double wheelDistLeftRightFrontSide;

    /** Track of the vehicle wheels at the back axel */
    private double wheelDistLeftRightBackSide;

    /** Wheelbase of the vehicle to the front axel */
    private double wheelDistToFront;

    /** Wheelbase of the vehicle to the back axel */
    private double wheelDistToBack;


    /** Internal Attributes */
    /** Last navigation target for vehicle */
    private Optional<IControllerNode> lastNavigationTarget;

    /** Camera image from visualization */
    private Optional<Image> cameraImage;

    /** Maximum temporary allowed velocity of vehicle */
    private double maxTemporaryAllowedVelocity;


    /** Internal flags */
    /** Flag whether the constant bus data was sent */
    private boolean constantBusDataSent;

    /** Flag whether the vehicle is initialised */
    private boolean vehicleInitialised;

    Boolean batteryProblem = false;

    /**
     * Constructor for a vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     */
    protected Vehicle(PhysicalVehicle physicalVehicle){
        // Create the motor
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_MOTOR, VEHICLE_DEFAULT_MOTOR_ACCELERATION_MIN, VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX, VEHICLE_DEFAULT_MOTOR_ACCELERATION_RATE);
        // Create the brakes
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE);
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE);
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE);
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE);
        // Create the steering
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_STEERING, VEHICLE_DEFAULT_STEERING_ANGLE_MIN, VEHICLE_DEFAULT_STEERING_ANGLE_MAX, VEHICLE_DEFAULT_STEERING_ANGLE_RATE);
        //Inputs from Controller
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_BRAKE, VEHICLE_DEFAULT_BRAKE_PRESSURE_MIN, VEHICLE_DEFAULT_BRAKE_PRESSURE_MAX, VEHICLE_DEFAULT_BRAKE_PRESSURE_RATE);
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_CLUTCH, VEHICLE_DEFAULT_CLUTCH_POSITION_MIN, VEHICLE_DEFAULT_CLUTCH_POSITION_MAX, VEHICLE_DEFAULT_CLUTCH_POSITION_RATE);
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_THROTTLE, VEHICLE_DEFAULT_THROTTLE_POSITION_MIN, VEHICLE_DEFAULT_THROTTLE_POSITION_MAX, VEHICLE_DEFAULT_THROTTLE_POSITION_RATE);
        setActuatorProperties(VEHICLE_ACTUATOR_TYPE_GEAR, VEHICLE_DEFAULT_GEAR_MIN, VEHICLE_DEFAULT_GEAR_MAX, VEHICLE_DEFAULT_GEAR_RATE);
        // Create the status logger
        this.statusLogger = new StatusLogger();
        // Create the sensor list
        this.sensorList = new ArrayList<>();
        // Create battery
        this.battery = Optional.empty();
        // Create the controller bus
        this.controllerBus = Optional.empty();
        // Create the controller
        this.controller = Optional.empty();
        // Create the navigation unit
        this.navigation = Optional.empty();
        // Set physicalVehicle that this vehicle is part of
        this.physicalVehicle = physicalVehicle;
        // Set width
        this.width = VEHICLE_DEFAULT_WIDTH;
        // Set length
        this.length = VEHICLE_DEFAULT_LENGTH;
        // Set height
        this.height = VEHICLE_DEFAULT_HEIGHT;
        // Set front mass
        this.mass = VEHICLE_DEFAULT_MASS;
        // Set wheel radius
        this.wheelRadius = VEHICLE_DEFAULT_WHEEL_RADIUS;
        // Set track
        this.wheelDistLeftRightFrontSide =VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_FRONT;
        this.wheelDistLeftRightBackSide = VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_REAR;
        // Set wheel base
        this.wheelDistToFront = VEHICLE_DEFAULT_WHEEL_DIST_TO_FRONT;
        this.wheelDistToBack = VEHICLE_DEFAULT_WHEEL_DIST_TO_BACK;
        // Initialise last navigation target with empty optional
        this.lastNavigationTarget = Optional.empty();
        // Initialise camera image with empty optional
        cameraImage = Optional.empty();
        // When created, maximum temporary allowed velocity is not limited
        this.maxTemporaryAllowedVelocity = Double.MAX_VALUE;
        // When created, the constant bus data is not sent yet
        this.constantBusDataSent = false;
        // When created, the physical vehicle is not initialised
        this.vehicleInitialised = false;
    }

    /**
     * Function that returns the current vehicle actuators
     *
     * @param type Type of the vehicle actuator to get
     * @return Current vehicle actuator object for the type, otherwise null
     */
    public VehicleActuator getVehicleActuator(VehicleActuatorType type) {
        switch (type) {
            case VEHICLE_ACTUATOR_TYPE_MOTOR:
                return motor;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT:
                return brakesFrontLeft;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT:
                return brakesFrontRight;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT:
                return brakesBackLeft;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT:
                return brakesBackRight;
            case VEHICLE_ACTUATOR_TYPE_STEERING:
                return steering;
            case VEHICLE_ACTUATOR_TYPE_GEAR:
                return gear;
            case VEHICLE_ACTUATOR_TYPE_BRAKE:
                return brakes;
            case VEHICLE_ACTUATOR_TYPE_CLUTCH:
                return clutch;
            case VEHICLE_ACTUATOR_TYPE_THROTTLE:
                return throttle;
            default:
                return null;
        }
    }

    /**
     * Function that sets the actuator properties
     *
     * @param actuatorType       Type of the actuator
     * @param actuatorValueMin   Minimum allowed value of the actuator
     * @param actuatorValueMax   Maximum allowed value of the actuator
     * @param actuatorChangeRate Change rate of the actuator
     */
    void setActuatorProperties(VehicleActuatorType actuatorType, double actuatorValueMin, double actuatorValueMax, double actuatorChangeRate) {
        switch (actuatorType) {
            case VEHICLE_ACTUATOR_TYPE_MOTOR:
                motor = new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT:
                brakesFrontLeft = new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT:
                brakesFrontRight = new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT:
                brakesBackLeft = new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT:
                brakesBackRight = new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_STEERING:
                steering = new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_GEAR:
                gear =  new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_BRAKE:
                brakes =  new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_CLUTCH:
                clutch =  new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            case VEHICLE_ACTUATOR_TYPE_THROTTLE:
                throttle =  new VehicleActuator(actuatorType, actuatorValueMin, actuatorValueMax, actuatorChangeRate);
                break;
            default:
                break;
        }
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
     * Function that returns an optional of the sensor of the requested type
     * If no such sensor is available return an <code>Optional.empty()</code>
     *
     * @param type ToDo
     * @return ToDo
     */
    public Optional<Sensor> getSensorByType(BusEntry type) {
        for (Sensor sensor : sensorList) {
            if (sensor.getType() == type) {
                return Optional.of(sensor);
            }
        }
        return Optional.empty();
    }

    /**
     * Function that adds a sensor the the vehicle
     *
     * @param sensor ToDo
     */
    public void addSensor(Sensor sensor) {
        this.sensorList.add(sensor);
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
            List<Vertex> trajectory = getTrajectory();
            if (trajectory.isEmpty()) {
                return true;
            } else {
                return false;
            }
        }
        return false;
    }

    /**
     * @return true if it's an EV
     */
    public boolean isElectricVehicle(){
        return this.battery.isPresent();
    }

    /**
     * Function that returns the optional controller bus
     *
     * @return Optional controller bus of the vehicle
     */
    protected Optional<Bus> getControllerBus() {
        return controllerBus;
    }

    /**
     * Function that sets the optional controller bus
     *
     * @param controllerBus Optional controller bus of the vehicle
     */
    protected void setControllerBus(Optional<Bus> controllerBus) {
        this.controllerBus = controllerBus;
    }

    /**
     * Function that returns the optional controller
     *
     * @return Optional controller of the vehicle
     */
    protected Optional<FunctionBlockInterface> getController() {
        return controller;
    }

    /**
     * Function that sets the optional controller
     *
     * @param controller Optional controller of the vehicle
     */
    protected void setController(Optional<FunctionBlockInterface> controller) {
        this.controller = controller;
    }

    /**
     * Function that returns the optional navigation
     *
     * @return Optional navigation of the vehicle
     */
    public Optional<FunctionBlockInterface> getNavigation() {
        return navigation;
    }

    /**
     * Function that sets the optional navigation
     *
     * @param navigation Optional navigation of the vehicle
     */
    public void setNavigation(Optional<FunctionBlockInterface> navigation) {
        this.navigation = navigation;
    }

    /**
     * Function that returns the width of the vehicle
     *
     * @return Width of the vehicle
     */
    public double getWidth() {
        return width;
    }

    /**
     * Function that sets the width of the vehicle
     *
     * @param width New width of the vehicle
     */
    public void setWidth(double width){
        this.width = width;
    }

    /**
     * Function that returns the length of the vehicle
     *
     * @return Length of the vehicle
     */
    public double getLength() {
        return length;
    }

    /**
     * Function that sets the length of the vehicle
     *
     * @param length New length of the vehicle
     */
    public void setLength(double length){
        this.length = length;
    }

    /**
     * Function that returns the height of simulated car
     *
     * @return Height of the vehicle
     */
    public double getHeight() {
        return height;
    }

    /**
     * Function that sets the height of the vehicle
     *
     * @param height New height of the vehicle
     */
    public void setHeight(double height){
        if(vehicleInitialised){
            throw new IllegalStateException("Height can only be set before initialisation.");
        }
        this.height = height;
    }

    /**
     * Function that returns the mass of the vehicle
     *
     * @return Mass of the vehicle
     */
    public double getMass() {
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            return mass;
        }else{
            if(!vehicleInitialised) {
                throw new IllegalStateException("Mass can only be read after initialisation.");
            }else{
                ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
                return  modelicaPhysicalVehicle.getVDM().getValue("m");
            }
        }
    }

    /**
     * Function that sets the mass of the vehicle
     *
     * @param mass New mass of the vehicle
     */
    public void setMass(double mass){
        if(vehicleInitialised){
            throw new IllegalStateException("Mass can only be set before initialisation.");
        }
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            this.mass = mass;
        }else{
            ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
            modelicaPhysicalVehicle.getVDM().setParameter("m", mass);
        }
    }

    /**
     * Function that returns the wheel radius of the vehicle
     *
     * @return Wheel radius of the vehicle
     */
    public double getWheelRadius() {
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            return wheelRadius;
        }else{
            if(!vehicleInitialised) {
                throw new IllegalStateException("Wheel radius can only be read after initialisation.");
            }else{
                ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
                return modelicaPhysicalVehicle.getVDM().getValue("r_nom");
            }
        }
    }

    /**
     * Function that sets the wheel radius of the vehicle
     *
     * @param wheelRadius New wheel radius of the vehicle
     */
    public void setWheelRadius(double wheelRadius){
        if(vehicleInitialised){
            throw new IllegalStateException("Wheel radius can only be set before initialisation.");
        }
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            this.wheelRadius = wheelRadius;
        }else{
            ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
            modelicaPhysicalVehicle.getVDM().setParameter("r_nom", wheelRadius);
        }
    }

    /**
     * Function that returns the distance between left and right wheels of the front axel of the vehicle
     *
     * @return Distance between left and right wheels of the front axel of the vehicle
     */
    double getWheelDistLeftRightFrontSide() {
        if(physicalVehicle instanceof MassPointPhysicalVehicle){
            return wheelDistLeftRightFrontSide;
        }else{
            if(!vehicleInitialised) {
                throw new IllegalStateException("Front axel wheel distance can only be read after initialisation.");
            }else{
                ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
                return modelicaPhysicalVehicle.getVDM().getValue("TW_f");
            }
        }
    }

    /**
     * Function that sets the distance between left and right wheels of the front axel of the vehicle
     *
     * @param wheelDistLeftRightFrontSide New distance between left and right wheels of the front axel of the vehicle
     */
    public void setWheelDistLeftRightFrontSide(double wheelDistLeftRightFrontSide){
        if (vehicleInitialised) {
            throw new IllegalStateException("Front axel wheel distance can only be set before initialisation.");
        }
        if(physicalVehicle instanceof MassPointPhysicalVehicle){
            this.wheelDistLeftRightFrontSide = wheelDistLeftRightFrontSide;
        }else{
            ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
            modelicaPhysicalVehicle.getVDM().setParameter("TW_f", wheelDistLeftRightFrontSide);
        }
    }

    /**
     * Function that returns the distance between left and right wheels of the back axel of the vehicle
     *
     * @return Distance between left and right wheels of the back axel of the vehicle
     */
    double getWheelDistLeftRightBackSide(){
        if(physicalVehicle instanceof MassPointPhysicalVehicle){
            return wheelDistLeftRightBackSide;
        }else{
            if(!vehicleInitialised){
                throw new IllegalStateException("Back axel wheel distance can only be set before initialisation.");
            }else{
                ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
                return modelicaPhysicalVehicle.getVDM().getValue("TW_r");
            }
        }
    }

    /**
     * Function that sets the distance between left and right wheels of the back axel of the vehicle
     *
     * @param wheelDistLeftRightBackSide New distance between left and right wheels of the back axel of the vehicle
     */
    public void setWheelDistLeftRightBackSide(double wheelDistLeftRightBackSide){
        if (vehicleInitialised) {
            throw new IllegalStateException("Position can only be set after initialisation.");
        }
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            this.wheelDistLeftRightBackSide = wheelDistLeftRightBackSide;
        }else{
            ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
            modelicaPhysicalVehicle.getVDM().setParameter("TW_r", wheelDistLeftRightBackSide);
        }
    }

    /**
     * Function that returns the distance between the center of mass and the front axel of the vehicle
     *
     * @return Distance between center of mass and front axel of the vehicle
     */
    double getWheelDistToFront() {
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            return wheelDistToFront;
        }else{
            if(!vehicleInitialised){
                throw new IllegalStateException("Distance from the center of mass to the front axel can only be read after initialisation.");
            }else{
                ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
                return modelicaPhysicalVehicle.getVDM().getValue("L_1");
            }
        }
    }

    /**
     * Function that sets the distance between the center of mass and the front axel of the vehicle
     *
     * @param wheelDistToFront New distance between center of mass and front axel of the vehicle
     */
    public void setWheelDistToFront(double wheelDistToFront){
        if (vehicleInitialised) {
            throw new IllegalStateException("Distance from the center of mass to the front axel can only be set before initialisation.");
        }
        if (physicalVehicle instanceof MassPointPhysicalVehicle) {
            this.wheelDistToFront = wheelDistToFront;
        } else {
            ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
            modelicaPhysicalVehicle.getVDM().setParameter("L_1", wheelDistToFront);
        }
    }

    /**
     * Function that returns the distance between the center of mass and the back axel of the vehicle
     *
     * @return Distance between center of mass and back axel of the vehicle
     */
    double getWheelDistToBack() {
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            return wheelDistToBack;
        }else{
            if(!vehicleInitialised){
                throw new IllegalStateException("Ha"); //tdo error
            }else{
                ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
                return modelicaPhysicalVehicle.getVDM().getValue("L_2");
            }
        }
    }

    /**
     * Function that sets the distance between the center of mass and the back axel of the vehicle
     *
     * @param wheelDistToBack New distance between center of mass and back axel of the vehicle
     */
    public void setWheelDistToBack(double wheelDistToBack){
        if (vehicleInitialised) {
            throw new IllegalStateException("Distance from the center of mass to the back axel can only be set before initialisation.");
        }
        if(physicalVehicle instanceof MassPointPhysicalVehicle) {
            this.wheelDistToBack = wheelDistToBack;
        }else {
            ModelicaPhysicalVehicle modelicaPhysicalVehicle = (ModelicaPhysicalVehicle) physicalVehicle;
            modelicaPhysicalVehicle.getVDM().setParameter("L_2", wheelDistToBack);
        }
    }

    /**
     * Function that returns the last navigation target of the vehicle
     *
     * @return ToDo
     */
    public Optional<IControllerNode> getLastNavigationTarget() {
        return lastNavigationTarget;
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
     * Function that sets the vehicle initialised flag to true
     * Should only be called by initPhysics
     *
     * @param vehicleInitialised New value for the vehicle initialised flag
     */
    public void setVehicleInitialised(boolean vehicleInitialised){
        if(!vehicleInitialised){
            throw new IllegalArgumentException("Vehicle can only be initialised once.");
        }
        this.vehicleInitialised = vehicleInitialised;
    }











    /**
     * Function that exchanges data with the controller
     *
     * @param deltaT Time difference of the last update loop in seconds
     * TODO: Add remaining Actuators
     */
    protected void exchangeDataWithController(double deltaT) {
        // Skip if controller bus or controller are not available
        if (!controllerBus.isPresent() || !controller.isPresent()) {
            return;
        }

        batteryProblem = false;
        
        if (battery != null) {
            
            // check if connection to ChargingStation is established
            if ( battery.get().getChargingStationConnectionStatus() == true ) {
                
                // dummy placeholders for ChargingStation specifications
                double ChargingStationVoltage = 100;
                double ChargingStationAmpere = 1;
                
                // depending on the ChargingStation implementation, 
                // they also can invoke this function themselves
                //      here, for demonstration purposes, we set them to some dummy values
                battery.get().setVoltageChargingStation(ChargingStationVoltage);
                battery.get().setAmpereChargingStation(ChargingStationAmpere);
                // leave charging initiation to ChargingStation, not to the Vehicle
                //battery.get().charge();
            }
            
            // check vehicle type,
            //      set consumption calculation based on the vehicle type
            if (physicalVehicle instanceof MassPointPhysicalVehicle)
                battery.get().setConsumptionMethod(IBattery.ConsumptionMethod.CONSUMPTION_MASS_VELOCITY);
            else
                battery.get().setConsumptionMethod(IBattery.ConsumptionMethod.CONSUMPTION_THROTTLE_GEAR);
            
            try {
                battery.get().discharge();
            }
            catch (IllegalArgumentException e) {
                // this flag would cause motor/throttle to be updated to zero,
                //      exactly at the end of this function
                batteryProblem = true;
            }   
        }

        // Send vehicle data to controller
        if (!constantBusDataSent) {
            controllerBus.get().setData(CONSTANT_NUMBER_OF_GEARS.toString(), 1);
            controllerBus.get().setData(CONSTANT_WHEELBASE.toString(), getWheelDistToFront() + getWheelDistToBack());
            controllerBus.get().setData(CONSTANT_MAXIMUM_TOTAL_VELOCITY.toString(), Vehicle.VEHICLE_DEFAULT_APPROX_MAX_VELOCITY);
            controllerBus.get().setData(CONSTANT_MOTOR_MAX_ACCELERATION.toString(), motor.getActuatorValueMax());
            controllerBus.get().setData(CONSTANT_MOTOR_MIN_ACCELERATION.toString(), motor.getActuatorValueMin());
            controllerBus.get().setData(CONSTANT_BRAKES_MAX_ACCELERATION.toString(), brakesFrontLeft.getActuatorValueMax());
            controllerBus.get().setData(CONSTANT_BRAKES_MIN_ACCELERATION.toString(), brakesFrontLeft.getActuatorValueMin());
            controllerBus.get().setData(CONSTANT_STEERING_MAX_ANGLE.toString(), steering.getActuatorValueMax());
            controllerBus.get().setData(CONSTANT_STEERING_MIN_ANGLE.toString(), steering.getActuatorValueMin());
            controllerBus.get().setData(CONSTANT_TRAJECTORY_ERROR.toString(), 0.0);

            constantBusDataSent = true;
        }

        // Send sensor data: Write values to bus
        for (Sensor sensor : sensorList) {
            // Put data from sensor on the bus
            controllerBus.get().setData(sensor.getType().toString(), sensor.getValue());

            // Special case for weather / surface, for now just constant Asphalt
            if (sensor.getType() == SENSOR_WEATHER) {
                Surface surface = Surface.Asphalt;
                controllerBus.get().setData(SENSOR_CURRENT_SURFACE.toString(), surface);
            }
        }

        //TODO: This logic should be moved to the controller
        Optional<Sensor> streetTypeSensor = getSensorByType(SENSOR_STREETTYPE);
        if (streetTypeSensor.isPresent()) {
            String streetType = (String)(streetTypeSensor.get().getValue());
            double allowedVelocityByStreetType = Double.MAX_VALUE;

            switch(streetType){
                case "MOTORWAY":
                    allowedVelocityByStreetType = (100.0 / 3.6);
                    break;
                case "A_ROAD":
                    allowedVelocityByStreetType = (70.0 / 3.6);
                    break;
                case "STREET":
                    allowedVelocityByStreetType = (50.0 / 3.6);
                    break;
                case "LIVING_STREET":
                    allowedVelocityByStreetType = (30.0 / 3.6);
                    break;
                default:
                    allowedVelocityByStreetType = maxTemporaryAllowedVelocity;
                    break;
            }

            setMaxTemporaryAllowedVelocity(Math.min(getMaxTemporaryAllowedVelocity(), allowedVelocityByStreetType));
        }

        // Set other values on bus that can change during simulation
        controllerBus.get().setData(SIMULATION_DELTA_TIME.toString(), deltaT);
        controllerBus.get().setData(VEHICLE_MAX_TEMPORARY_ALLOWED_VELOCITY.toString(), getMaxTemporaryAllowedVelocity());

        //Give the bus to the mainControlBlock
        controller.get().setInputs(controllerBus.get().getAllData());

        // Call controller to compute new values
        controller.get().execute(deltaT);

        //Pass the data of the mainControlBlock to the bus
        controllerBus.get().setAllData(controller.get().getOutputs());

        // Read new values from bus
        double motorValue = 0;
        double brakeValue = 0;
        double steeringValue = 0;

        Object motorObj = controllerBus.get().getData(BusEntry.ACTUATOR_ENGINE.toString());
        if (motorObj != null){
            motorValue = (Double) motorObj;
        }
        Object brakeObj = controllerBus.get().getData(BusEntry.ACTUATOR_BRAKE.toString());
        if (brakeObj != null){
            brakeValue = (Double) brakeObj;
        }
        Object steeringObj = controllerBus.get().getData(BusEntry.ACTUATOR_STEERING.toString());
        if (steeringObj != null){
            steeringValue = (Double) steeringObj;
        }

        steering.setActuatorValueTarget(steeringValue);

        // Set new values from bus to actuators

        if (physicalVehicle instanceof MassPointPhysicalVehicle) {
            motor.setActuatorValueTarget(motorValue);
            brakesBackLeft.setActuatorValueTarget(brakeValue);
            brakesBackRight.setActuatorValueTarget(brakeValue);
            brakesFrontLeft.setActuatorValueTarget(brakeValue);
            brakesFrontRight.setActuatorValueTarget(brakeValue);

        } else {
            throttle.setActuatorValueTarget(motorValue);
            double brakePressure = brakeValue*brakes.getActuatorValueMax();
            brakes.setActuatorValueTarget(brakePressure);
        }

        //  Check Battery
        checkBattery();
    }

    /**
     * Check Battery state and move to the next Chargingstation if needed
     */
    void checkBattery(){
        if(battery.isPresent()) {
            if (isElectricVehicle() && !gotoCharginstation && battery.get().getBatteryPercentage() <= 20) {
                gotoCharginstation = true;
                try {
                    long nearestcharg = ChargingStationNavigator.getNearestChargingStation(
                            physicalVehicle.getGlobalId(),
                            ChargingStationNavigator.RealVectortoOSMID(this.physicalVehicle.getPosition())
                    );
                    RealVector point3d = ChargingStationNavigator.getPositionOfOsmNode(nearestcharg);
                    navigateTo(new ControllerNode(Geometry.realVector2Point3D(point3d), nearestcharg));
                } catch (Exception e) {
                    e.printStackTrace();
                }
                //      battery discharging failed, cannot accelerate the vehicle
                //      set either motor OR throttle to zero, based on type of the car
                if (batteryProblem) {
                    if (physicalVehicle instanceof MassPointPhysicalVehicle) {
                        motor.setActuatorValueTarget(0.0);
                    } else {
                        throttle.setActuatorValueTarget(0.0);
                    }
                }
            }
        }
    }

    /**
     * Can be called from the Chargingstation
     */
    public void onRechargeReady(){
        gotoCharginstation = false;
        navigateTo(lastdestination);
    }

    /**
     * Function that initiates or updates navigation of the vehicle to a specified point in the map
     * Controller is periodically called such that setting these values in the function here should work without issues
     *
     * @param node Target node for navigation
     */
    public void navigateTo(IControllerNode node) {
        if(!gotoCharginstation) {
            lastdestination = node;
        }
        navigateTo(node, Collections.synchronizedList(new LinkedList<RealVector>()));
    }

    public boolean getGotoChargingStation(){
        return gotoCharginstation;
    }

    /**
     * Function that initiates or updates navigation of the vehicle to a specified point in the map
     * Controller is periodically called such that setting these values in the function here should work without issues
     * Tries to avoid list of coordinates, might not be possible if all ways to target are affected. Then avoiding coordinates is not possible.
     *
     * @param node Target node for navigation
     * @param avoidCoordinates List of coordinates which should be avoided in path finding, if possible
     */
    public void navigateTo(IControllerNode node, List<RealVector> avoidCoordinates) {
        // Check for valid objects
        if (!navigation.isPresent() || !controllerBus.isPresent() || !getSensorByType(SENSOR_GPS_COORDINATES).isPresent()) {
            Log.warning("Vehicle: navigateTo called without valid navigation or controllerBus or GPS sensor");
            return;
        }

        // Set last navigation target
        this.lastNavigationTarget = Optional.of(node);

        // Get current GPS coordinates from sensor
        getSensorByType(SENSOR_GPS_COORDINATES).get().update();
        Object gpsCoordinates = getSensorByType(SENSOR_GPS_COORDINATES).get().getValue();

        // Process navigation target without avoiding coordinates for reference
        Map<String, Object> navigationInputs = new LinkedHashMap<>();
        navigationInputs.put(NavigationEntry.MAP_ADJACENCY_LIST.toString(), WorldModel.getInstance().getControllerMap().getAdjacencies());
        navigationInputs.put(NavigationEntry.CONSTANT_WHEELBASE.toString(), getWheelDistToFront() + getWheelDistToBack());
        navigationInputs.put(NavigationEntry.GPS_COORDINATES.toString(), gpsCoordinates);
        navigationInputs.put(NavigationEntry.TARGET_NODE.toString(), node);
        navigation.get().setInputs(navigationInputs);
        navigation.get().execute(0);

        // Stop processing if trajectory or avoiding coordinate list is empty
        if (navigation.get().getOutputs().get(NavigationEntry.DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()) == null) {
            return;
        }

        List<Vertex> trajectoryWithoutAvoiding = (List<Vertex>)(navigation.get().getOutputs().get(NavigationEntry.DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()));
        if (trajectoryWithoutAvoiding.isEmpty() || avoidCoordinates.isEmpty()) {
            controllerBus.get().setData(NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString(), trajectoryWithoutAvoiding);
            afterTrajectoryUpdate();
            return;
        }

        // Compare distance to final destination to compare quality of trajectories
        RealVector endTarget = new ArrayRealVector(new double[]{node.getPoint().getX(), node.getPoint().getY(), node.getPoint().getZ()});
        double endTargetDistanceWithoutAvoiding = trajectoryWithoutAvoiding.get(trajectoryWithoutAvoiding.size() - 1).getPosition().getDistance(endTarget);

        // Compute trajectory with avoiding coordinates on a copied adjacency list
        ArrayList<IAdjacency> adjacencyFiltered = new ArrayList<>(WorldModel.getInstance().getControllerMap().getAdjacencies());
        ArrayList<IAdjacency> adjacencyRemove = new ArrayList<>();
        Set<Long> filterOsmIds = new HashSet<>();

        // Find OSM IDs with minimal distance to coordinates to be avoided
        for (RealVector pos : avoidCoordinates) {
            double minDistSq = Double.MAX_VALUE;
            long minOsmId = -1L;

            for (IAdjacency adjacency : adjacencyFiltered) {
                RealVector posAdjacency1 = new ArrayRealVector(new double[]{adjacency.getNode1().getPoint().getX(), adjacency.getNode1().getPoint().getY(), adjacency.getNode1().getPoint().getZ()});
                RealVector posAdjacency2 = new ArrayRealVector(new double[]{adjacency.getNode2().getPoint().getX(), adjacency.getNode2().getPoint().getY(), adjacency.getNode2().getPoint().getZ()});

                // Compute square distances manually here, cheaper than distance since no sqrt is needed for minimum
                double distSq1 = (pos.getEntry(0) - posAdjacency1.getEntry(0)) * (pos.getEntry(0) - posAdjacency1.getEntry(0)) + (pos.getEntry(1) - posAdjacency1.getEntry(1)) * (pos.getEntry(1) - posAdjacency1.getEntry(1)) + (pos.getEntry(2) - posAdjacency1.getEntry(2)) * (pos.getEntry(2) - posAdjacency1.getEntry(2));
                double distSq2 = (pos.getEntry(0) - posAdjacency2.getEntry(0)) * (pos.getEntry(0) - posAdjacency2.getEntry(0)) + (pos.getEntry(1) - posAdjacency2.getEntry(1)) * (pos.getEntry(1) - posAdjacency2.getEntry(1)) + (pos.getEntry(2) - posAdjacency2.getEntry(2)) * (pos.getEntry(2) - posAdjacency2.getEntry(2));

                if (distSq1 < minDistSq) {
                    minDistSq = distSq1;
                    minOsmId = adjacency.getNode1().getOsmId();
                }

                if (distSq2 < minDistSq) {
                    minDistSq = distSq2;
                    minOsmId = adjacency.getNode2().getOsmId();
                }
            }

            if (minOsmId > 0) {
                filterOsmIds.add(minOsmId);
            }
        }

        // Find adjacency entries with OSM Ids to be removed
        for (IAdjacency adjacency : adjacencyFiltered) {
            if (filterOsmIds.contains(adjacency.getNode1().getOsmId()) || filterOsmIds.contains(adjacency.getNode2().getOsmId())) {
                adjacencyRemove.add(adjacency);
            }
        }

        // Remove all adjacency entries to be filtered out
        adjacencyFiltered.removeAll(adjacencyRemove);

        // Process navigation target without avoiding coordinates for reference
        Map<String, Object> navigationInputsFiltered = new LinkedHashMap<>();
        navigationInputsFiltered.put(NavigationEntry.MAP_ADJACENCY_LIST.toString(), adjacencyFiltered);
        navigationInputsFiltered.put(NavigationEntry.CONSTANT_WHEELBASE.toString(), getWheelDistToFront() + getWheelDistToBack());
        navigationInputsFiltered.put(NavigationEntry.GPS_COORDINATES.toString(), gpsCoordinates);
        navigationInputsFiltered.put(NavigationEntry.TARGET_NODE.toString(), node);
        navigation.get().setInputs(navigationInputsFiltered);
        navigation.get().execute(0);

        // If trajectory with avoiding is null or empty, just set original result without avoiding
        if (navigation.get().getOutputs().get(NavigationEntry.DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()) == null) {
            controllerBus.get().setData(NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString(), trajectoryWithoutAvoiding);
            afterTrajectoryUpdate();
            return;
        }

        List<Vertex> trajectoryWithAvoiding = (List<Vertex>)(navigation.get().getOutputs().get(NavigationEntry.DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()));
        if (trajectoryWithAvoiding.isEmpty()) {
            controllerBus.get().setData(NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString(), trajectoryWithoutAvoiding);
            afterTrajectoryUpdate();
            return;
        }

        // Compare distance to final destination to compare quality of trajectories
        double endTargetDistanceWithAvoiding = trajectoryWithAvoiding.get(trajectoryWithAvoiding.size() - 1).getPosition().getDistance(endTarget);

        // Check if end target distance with avoiding is roughly as good as without avoiding
        // If yes then set new trajectory with avoiding, otherwise use old one without avoiding
        if (endTargetDistanceWithAvoiding - 5.0 <= endTargetDistanceWithoutAvoiding) {
            controllerBus.get().setData(NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString(), trajectoryWithAvoiding);
            afterTrajectoryUpdate();
            return;
        }

        controllerBus.get().setData(NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString(), trajectoryWithoutAvoiding);
        afterTrajectoryUpdate();
    }

    /**
     * Get current trajectory of the vehicle, if available. Otherwise return empty list.
     *
     * @return Current trajectory of the vehicle, if not available return empty list
     */
    public List<Vertex> getTrajectory() {
        // Check if trajectory is available and return copy if valid
        if (controllerBus.isPresent() && (controllerBus.get().getData(NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()) != null)) {
                ArrayList<Vertex> originalList = (ArrayList<Vertex>)(controllerBus.get().getData(NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE.toString()));
                return new ArrayList<>(originalList);
        }

        // Fallback to empty list
        return new ArrayList<>();
    }

    public boolean isGotoCharginstation() {
        return gotoCharginstation;
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
     * Internal function that is called after an trajectory update was performed
     */
    private void afterTrajectoryUpdate() {
        // Get current trajectory
        List<Vertex> trajectory = getTrajectory();
        if (trajectory.isEmpty()) {
            return;
        }

        // Add intersection node information to each vertex in the trajectory
        Set<OsmNode> intersectionNodes = IntersectionFinder.getInstance().getIntersections();
        for (Vertex vertex : trajectory) {
            for (OsmNode intersectionNode : intersectionNodes) {
                if (vertex.getOsmId() == intersectionNode.getId()) {
                    vertex.setIntersectionNode(true);
                }
            }
        }
    }

    /**
     * Function that updates all sensor data, should be called before data exchange with controller
     */
    public void updateAllSensors() {
        for (Sensor sensor : sensorList) {
            sensor.update();
        }
    }

    /**
     * Overwrite toString() to get a nice output for vehicles
     *
     * @return String that contains all information of vehicles
     */
    @Override
    public String toString() {
        return "Vehicle " + hashCode() + ": length: " + length +
                " , width: " + width +
                " , height: " + height +
                " , maxTemporaryAllowedVelocity: " + maxTemporaryAllowedVelocity +
                " , wheelRadius: " + wheelRadius +
                " , wheelDistLeftRightFrontSide: " + wheelDistLeftRightFrontSide +
                " , wheelDistToFront: " + wheelDistToFront +
                " , wheelDistToBack: " + wheelDistToBack +
                " , mass: " + mass +
                " , constantBusDataSent: " + constantBusDataSent +
                " , motor: " + motor +
                " , brakesFrontLeft: " + brakesFrontLeft +
                " , brakesFrontRight: " + brakesFrontRight +
                " , brakesBackLeft: " + brakesBackLeft+
                " , brakesBackRight: " + brakesBackRight +
                " , steering: " + steering +
                " , sensorList:" + sensorList +
                " , controllerBus:" + controllerBus +
                " , controller:" + controller +
                " , navigation:" + navigation +
                " , cameraImage:" + cameraImage;
    }
}
