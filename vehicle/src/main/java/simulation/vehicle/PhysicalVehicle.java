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

import commons.simulation.SimulationLoopExecutable;
import commons.simulation.IPhysicalVehicle;
import commons.simulation.PhysicalObjectType;
import commons.simulation.IdGenerator;
import org.apache.commons.math3.linear.RealVector;
import simulation.EESimulator.EESimulator;
import simulation.util.Log;
import static simulation.vehicle.VehicleActuatorType.*;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Class that represents all physical properties of a vehicle and performs physics computations
 */
public abstract class PhysicalVehicle implements IPhysicalVehicle {
	
    /** Length of the vehicle in meters */
	protected static final double VEHICLE_DEFAULT_LENGTH = 4.236423828125;

    /** Width of the vehicle in meters */
    protected static final double VEHICLE_DEFAULT_WIDTH = 2.02712567705637776;

    /** Height of the vehicle in meters */
    protected static final double VEHICLE_DEFAULT_HEIGHT = 1.19524474896355328;

    /** Radius of the wheels in meters */
    protected static final double VEHICLE_DEFAULT_WHEEL_RADIUS = 0.3334;

    /** Mass of the vehicle */
    protected static final double VEHICLE_DEFAULT_MASS = 1800.0;

    /** Distance between the left and the right wheels in meters */
    protected static final double VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_FRONT = 1.62025;

    /**Track width rear wheels in meters */
    protected static final double VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_REAR = 1.505;

    /** Distance between front and back wheels in meters */
    protected static final double VEHICLE_DEFAULT_WHEEL_DIST_TO_FRONT = 1.379;

    /** Distance between back wheels and center of mass in meters */
    protected static final double VEHICLE_DEFAULT_WHEEL_DIST_TO_BACK = 1.542;

    /** Variables for the IPhysicalVehicle interface */
    /** Type of the physical object */
    PhysicalObjectType physicalObjectType;

    /** Error flag */
    protected boolean error;

    /** Collision flag */
    protected boolean collision;

    /** Unique ID */
    private final long uniqueId = IdGenerator.getSharedInstance().generateUniqueId();

    /** Dimensions of vehicle in meters */
    protected double width, length, height;

    /** Internal flags*/
    /** Flag whether the vehicle is fully initialised or not */
    protected boolean physicalVehicleInitialised;
    
    protected VehicleActuator steering;
    
    /** The vehicle that this physicalVehicle belongs to */
    private Vehicle vehicle;


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
        // When created, the physical vehicle is not initialised
        physicalVehicleInitialised = false;
        //Instantiate PhysicalVehicle
        this.instantiatePhysicalVehicle();
        // Set width
        this.width = VEHICLE_DEFAULT_WIDTH;
        // Set length
        this.length = VEHICLE_DEFAULT_LENGTH;
        // Set height
        this.height = VEHICLE_DEFAULT_HEIGHT;
        // Set front mass
        this.setMass(VEHICLE_DEFAULT_MASS);
        // Set wheel radius
        this.setWheelRadius(VEHICLE_DEFAULT_WHEEL_RADIUS);
        // Set track
        this.setWheelDistLeftRightFrontSide(VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_FRONT);
        this.setWheelDistLeftRightBackSide(VEHICLE_DEFAULT_WHEEL_TRACK_WIDTH_REAR);
        // Set wheel base
        this.setWheelDistToFront(VEHICLE_DEFAULT_WHEEL_DIST_TO_FRONT);
        this.setWheelDistToBack(VEHICLE_DEFAULT_WHEEL_DIST_TO_BACK);
		 
    }

    /**
     * Function that returns the width of the object
     * @return Width of the object
     */

    @Override
    public double getWidth(){
        return width;
    }

    /**
     * Function that sets the width of the object
     * @param width New width of the object
     */
    @Override
    public void setWidth(double width){
        this.width = width;
    }

    /**
     * Function that returns the length of the object
     * @return Length of the object
     */
    @Override
    public double getLength(){
        return this.length;
    }

    /**
     * Function that sets the length of the object
     * @param length New length of the object
     */
    @Override
    public void setLength(double length){
        this.length = length;
    }

    /**
     * Function that returns the height of the object
     * @return Height of the object
     */
    @Override
    public double getHeight(){
        return this.height;
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
        this.height = height;
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
        this.vehicle.getEEVehicle().setCollision(true);
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
     * Function that returns the steering angle of the physical vehicle
     * @return Steering angle of the physical vehicle
     */
    @Override
    public double getSteeringAngle(){
        return this.steering.getActuatorValueCurrent();
    }
    
    /**
     * Function that returns if the physicalVehicle is initialised
     * @return Value of physicalVehicleInitialised
     */
    public boolean getPhysicalVehicleInitialised() {
        return physicalVehicleInitialised;
    }
    
    /**
     * Function that returns the vehicle this PhyiscalVehicle belongs to
     * @return Reference to the vehicle this PhysicalVehicle belongs to
     */
    public Vehicle getVehicle() {
    	return this.vehicle;
    }
    
    /**
     * Function that sets the vehicle this PhyiscalVehicle belongs to
     * @param Reference to the vehicle this PhysicalVehicle 
     */
    protected void setVehicle(Vehicle vehicle) {
    	if(this.vehicle != null) {
			throw new IllegalStateException("Vehicle can only be set once.");
    	}
    	if(physicalVehicleInitialised) {
			throw new IllegalStateException("Vehicle can only be set before initialisation.");
    	}
		this.vehicle = vehicle;
    }
    
	protected List<VehicleActuator> initializeActuators(VehicleActuatorType[] actuatorTypes) {
		List<VehicleActuator> actuators = new ArrayList<VehicleActuator>(actuatorTypes.length);
		for(VehicleActuatorType type : actuatorTypes) {
	        Optional<VehicleActuator> actuator = vehicle.getEEVehicle().getActuator(type);
			if(actuator.isPresent()) {
	        	actuators.add(actuator.get());
	        }
		}
		if(actuatorTypes.length > actuators.size()) {
			throw new IllegalStateException("EEVehicle does not contain all necessary actuators");
		}
		return actuators;
	}
    
//    /**
//     * Function that requests the called object to update its state for given time difference
//     * @param timeDiff Difference in time
//     */
//    @Override
//    public void executeLoopIteration(Duration timeDiff) {
//        if (this.error) {
//            Log.finest("PhysicalVehicle: Vehicle collided or had a computational error and will therefore not move anymore, PhysicalVehicle: " + this);
//            return;
//        }
//        Log.finest("PhysicalVehicle: executeLoopIteration - timeDiff: " + timeDiff + ", PhysicalVehicle at start: " + this);
//
//        final double deltaT = (timeDiff.toMillis() / 1000.0);
//
//        // Exchange data with controller
//        simulationVehicle.exchangeDataWithController(deltaT);
//
//        // Update vehicle actuators
//        if (!this.collision) {
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).update(deltaT);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).update(deltaT);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).update(deltaT);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).update(deltaT);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).update(deltaT);
//        }else{
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).setActuatorValueCurrent(0.0);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).setActuatorValueCurrent(0.0);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).setActuatorValueCurrent(0.0);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).setActuatorValueCurrent(0.0);
//            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).setActuatorValueCurrent(0.0);
//        }
//
//        simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).update(deltaT);
//        this.collision = false;
//
//        Log.finest("PhysicalVehicle: executeLoopIteration - timeDiff: " + timeDiff +  ", PhysicalVehicle at end: " + this);
//    }
	
    /**
* Function that returns the current vehicle actuators
*
* @param type Type of the vehicle actuator to get
* @return Current vehicle actuator object for the type, otherwise null
*/
	public VehicleActuator getVehicleActuator(VehicleActuatorType type) {
		if(type == VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING) {
			return steering;
		}
		return null;
	}

    /**
     * Function that initialises the actuators
     * Should only be called by vehicle
     */
	protected void initializeActuators() {
		if(steering != null) {
            throw new IllegalStateException("Actuators can only be initialised once.");
		}
		Optional<VehicleActuator> steering = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        if(steering.isPresent()) {
        	this.steering = steering.get();
        }
        else {
        	throw new IllegalStateException("EEVehicle does not have the necessary actuators for the pyhiscal vehicle");
        }
	}
	
	/**
	 * Function that instantiates the parts of the vehicle that are dependent on the implementation
	 * but need to be present before values can be set. (e.g. VehicleDynamicsModel)
	 */
	public abstract void instantiatePhysicalVehicle();
	
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
     * Function that returns the mass of the vehicle
     *
     * @return Mass of the vehicle
     */
     public abstract double getMass();

    /**
     * Function that sets the mass of the vehicle
     *
     * @param mass New mass of the vehicle
     */
    public abstract void setMass(double mass);

    /**
     * Function that returns the wheel radius of the vehicle
     *
     * @return Wheel radius of the vehicle
     */
    public abstract double getWheelRadius();

    /**
     * Function that sets the wheel radius of the vehicle
     *
     * @param wheelRadius New wheel radius of the vehicle
     */
    public abstract void setWheelRadius(double wheelRadius);

    /**
     * Function that returns the distance between left and right wheels of the front axel of the vehicle
     *
     * @return Distance between left and right wheels of the front axel of the vehicle
     */
    public abstract double getWheelDistLeftRightFrontSide();

    /**
     * Function that sets the distance between left and right wheels of the front axel of the vehicle
     *
     * @param wheelDistLeftRightFrontSide New distance between left and right wheels of the front axel of the vehicle
     */
    public abstract void setWheelDistLeftRightFrontSide(double wheelDistLeftRightFrontSide);

    /**
     * Function that returns the distance between left and right wheels of the back axel of the vehicle
     *
     * @return Distance between left and right wheels of the back axel of the vehicle
     */
    public abstract double getWheelDistLeftRightBackSide();

    /**
     * Function that sets the distance between left and right wheels of the back axel of the vehicle
     *
     * @param wheelDistLeftRightBackSide New distance between left and right wheels of the back axel of the vehicle
     */
    public abstract void setWheelDistLeftRightBackSide(double wheelDistLeftRightBackSide);

    /**
     * Function that returns the distance between the center of mass and the front axel of the vehicle
     *
     * @return Distance between center of mass and front axel of the vehicle
     */
    public abstract double getWheelDistToFront();

    /**
     * Function that sets the distance between the center of mass and the front axel of the vehicle
     *
     * @param wheelDistToFront New distance between center of mass and front axel of the vehicle
     */
    public abstract void setWheelDistToFront(double wheelDistToFront);

    /**
     * Function that returns the distance between the center of mass and the back axel of the vehicle
     *
     * @return Distance between center of mass and back axel of the vehicle
     */
    public abstract double getWheelDistToBack();

    /**
     * Function that sets the distance between the center of mass and the back axel of the vehicle
     *
     * @param wheelDistToBack New distance between center of mass and back axel of the vehicle
     */
    public abstract void setWheelDistToBack(double wheelDistToBack);



}