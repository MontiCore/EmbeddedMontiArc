/* (c) https://github.com/MontiCore/monticore */
package simulation.vehicle;

import commons.simulation.SimulationLoopExecutable;
import commons.simulation.IPhysicalVehicle;
import commons.simulation.PhysicalObjectType;
import commons.simulation.IdGenerator;
import org.apache.commons.math3.linear.RealVector;
import simulation.environment.object.ChargingStation;
import simulation.environment.util.Chargeable;
import simulation.environment.util.IBattery;
import simulation.util.Log;
import static simulation.vehicle.VehicleActuatorType.*;
import simulation.vehicle.Battery;
import java.util.Optional;

/**
 * Class that represents all physical properties of a vehicle and performs physics computations
 */
public abstract class PhysicalVehicle implements SimulationLoopExecutable, IPhysicalVehicle, Chargeable{

    /** Variables for the IPhysicalVehicle interface */
    /** Type of the physical object */
    PhysicalObjectType physicalObjectType;

    /** Error flag */
    protected boolean error;

    /** Collision flag */
    protected boolean collision;

    /** Unique ID */
    private final long uniqueId = IdGenerator.getSharedInstance().generateUniqueId();


    /** The vehicle */
    protected final Vehicle simulationVehicle;


    /** Internal flags*/
    /** Flag whether the vehicle is fully initialised or not */
    protected boolean physicalVehicleInitialised;


    /**
     * Constructor for a physical vehicle that is standing at its position
     * Use other functions to initiate movement and position updates
     */
    protected PhysicalVehicle() {
        // Set physical object type car
        this.physicalObjectType = PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR;
        // Set error flag
        error = false;
        // Set collision flag
        collision = false;
        // Create default simulation vehicle
        this.simulationVehicle = new Vehicle(this);
        // When created, the physical vehicle is not initialised
        physicalVehicleInitialised = false;
    }

    /**
     * Constructor for a physical vehicle that can be electric
     */
    protected PhysicalVehicle(boolean isElectricVehicle) {
        // Set physical object type car
        this.physicalObjectType = PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR;
        // Set error flag
        error = false;
        // Set collision flag
        collision = false;
        // Create default simulation vehicle
        this.simulationVehicle = new Vehicle(this);
        // Create battery if vehicle is electric
        if (isElectricVehicle) {
            Battery battery = new Battery(simulationVehicle,100,100);
            simulationVehicle.setBattery(battery);
        }
        // When created, the physical vehicle is not initialised
        physicalVehicleInitialised = false;
    }

    /**
     * @return true if it's an EV
     */

    @Override
    public boolean isElectricVehicle(){

        return simulationVehicle.isElectricVehicle();
    }

    /**
     * @return Battery of the vehicle
     */

    @Override
    public Optional<IBattery> getBattery(){
        return simulationVehicle.getBattery();
    }

    /**
     * @return true if vehicle is parked at charging station
     */

    @Override
    public boolean isParkedChargingStation(ChargingStation station){
        return simulationVehicle.isParkedChargingStation(station);
    }

    /**
     * Function that allows vehicle to move on after charging
     */

    @Override
    public void onRechargeReady(){
        simulationVehicle.onRechargeReady();
    }

    /**
     * Function that returns the width of the object
     * @return Width of the object
     */

    @Override
    public double getWidth(){
        return simulationVehicle.getWidth();
    }

    /**
     * Function that sets the width of the object
     * @param width New width of the object
     */
    @Override
    public void setWidth(double width){
        simulationVehicle.setWidth(width);
    }

    /**
     * Function that returns the length of the object
     * @return Length of the object
     */
    @Override
    public double getLength(){
        return simulationVehicle.getLength();
    }

    /**
     * Function that sets the length of the object
     * @param length New length of the object
     */
    @Override
    public void setLength(double length){
        simulationVehicle.setLength(length);
    }

    /**
     * Function that returns the height of the object
     * @return Height of the object
     */
    @Override
    public double getHeight(){
        return simulationVehicle.getHeight();
    }

    /**
     * Function that sets the height of the object
     * @param height New height of the object
     */
    @Override
    public void setHeight(double height){
        if(physicalVehicleInitialised) {
            throw new IllegalStateException("Height can only be set before initialisation.");
        }
        simulationVehicle.setHeight(height);
    }

    /**
     * Function that returns the type of the physical object
     * @return Type of the physical object
     */
    @Override
    public PhysicalObjectType getPhysicalObjectType(){
        return this.physicalObjectType;
    }

    /**
     * Function that returns the error flag
     * @return Error flag of the physical object
     */
    @Override
    public boolean getError(){
        return this.error;
    }

    /**
     * Function that sets the error flag
     * @param error New Error flag of the physical object
     */
    @Override
    public void setError(boolean error){
        this.error = error;
    }

    /**
     * Function that returns the collision flag
     * @return Collision flag of the physical object
     */
    @Override
    public boolean getCollision(){
        return this.collision;
    }

    /**
     * Function that sets the collision flag
     * @param collision New collision flag of the physical object
     */
    @Override
    public void setCollision(boolean collision){
        this.collision = collision;
    }

    /**
     * Function that returns the id of the physical object
     * @return Id of the physical object
     */
    @Override
    public long getId(){
        return this.uniqueId;
    }

    /**
     * Function that returns the wheel radius of the physical vehicle
     * @return Wheel radius of the physical vehicle
     */
    @Override
    public double getWheelRadius(){
        return simulationVehicle.getWheelRadius();
    }

    /**
     * Function that returns the steering angle of the physical vehicle
     * @return Steering angle of the physical vehicle
     */
    @Override
    public double getSteeringAngle(){
        return simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();
    }


    /**
     * Function that requests the called object to update its state for given time difference
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    @Override
    public void executeLoopIteration(long timeDiffMs) {
        if (this.error) {
            Log.finest("PhysicalVehicle: Vehicle collided or had a computational error and will therefore not move anymore, PhysicalVehicle: " + this);
            return;
        }
        Log.finest("PhysicalVehicle: executeLoopIteration - timeDiffMs: " + timeDiffMs + ", PhysicalVehicle at start: " + this);

        simulationVehicle.updateAllSensors();

        final double deltaT = (timeDiffMs / 1000.0);

        // Exchange data with controller
        simulationVehicle.exchangeDataWithController(deltaT);

        // Update vehicle actuators
        if (!this.collision) {
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).update(deltaT);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).update(deltaT);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).update(deltaT);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).update(deltaT);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).update(deltaT);
        }else{
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).setActuatorValueCurrent(0.0);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).setActuatorValueCurrent(0.0);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).setActuatorValueCurrent(0.0);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).setActuatorValueCurrent(0.0);
            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).setActuatorValueCurrent(0.0);
        }

        simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).update(deltaT);
        this.collision = false;

        Log.finest("PhysicalVehicle: executeLoopIteration - timeDiffMs: " + timeDiffMs +  ", PhysicalVehicle at end: " + this);
    }

    /**
     * Function that returns the current simulation vehicle
     * @return Current simulation vehicle object
     */
    public Vehicle getSimulationVehicle() {
        return simulationVehicle;
    }

    /**
     * Function that initialises the physical components and parameters
     * Should only be called by physical vehicle builder
     */
    public abstract void initPhysics();

    /**
     * Function that returns the force that is acting on the vehicle
     * @return Force acting on the vehicle
     */
    public abstract RealVector getForce();

    /**
     * Function that returns the torque that is acting on the vehicle
     * @return Torque acting on the vehicle
     */
    public abstract RealVector getTorque();

    /**
     * Function that returns if the physicalVehicle is initialised
     * @return Value of physicalVehicleInitialised
     */
    public boolean getPhysicalVehicleInitialised() {
        return physicalVehicleInitialised;
    }

}
