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

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.*;
import simulation.EESimulator.EESimulator;
import simulation.environment.WorldModel;
import simulation.environment.visualisationadapter.implementation.Street2D;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.EnvStreet;
import simulation.util.MathHelper;

import java.time.Duration;
import java.util.*;

import static simulation.vehicle.VehicleActuatorType.*;
import static simulation.vehicle.VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING;

/**
 * Class that represents a simulation vehicle with physical properties and interactions implemented according to the Vehicle Dynamics Model for Driving Simulators
 */
public class ModelicaPhysicalVehicle extends PhysicalVehicle{

    /** Variables for the IPhysicalVehicle interface */
    /** Position vector of the center of mass */
    private RealVector position;

    /** Rotation matrix around the center of mass */
    private RealMatrix rotation;

    /** Force vector acting on the center of mass */
    private RealVector force;

    /** Vector pointing from the center of mass position to the center of geometry position in the local coordinate system */
    private RealVector geometryPositionOffset;

    /** Current rotation around the local z axis */
    private double yawAngle;

    /** VehicleDynamicsModel used for modelica physics */
    private VehicleDynamicsModel vehicleDynamicsModel;

    /** Coordinate system rotation */
    public static final RealMatrix coordinateRotation = new BlockRealMatrix(new Rotation(
            RotationOrder.XYZ,
            RotationConvention.VECTOR_OPERATOR,
            0.0, 0.0, Math.PI / 2).getMatrix());

    /**
     * Constructor for an uninitialised physical vehicle
     */
    public ModelicaPhysicalVehicle(){
        super();

        vehicleDynamicsModel = new VehicleDynamicsModel();

        // Before initialisation
        // the center of mass position is at the origin
        // no rotation
        // the center of geometry position is at the origin
        this.position = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        this.rotation = coordinateRotation.copy();
        this.geometryPositionOffset = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
    }

    /**
     * Function that returns a copy of the center of mass position vector
     * @return Position vector of the center of mass
     */
    @Override
    public RealVector getPosition(){
        return  position.copy();
    }

    /**
     * Function that sets the center of mass position vector
     * @param position New position vector of the center of mass
     */
    @Override
    public void setPosition(RealVector position){
        if(!physicalVehicleInitialised) {
            throw new IllegalStateException("Position can only be set after initialisation.");
        }
        this.position = position.copy();
    }

    /**
     * Function that returns a copy of the rotation matrix around the center of mass
     * @return Rotation matrix around the center of mass
     */
    @Override
    public RealMatrix getRotation(){
        return coordinateRotation.transpose().multiply(this.rotation);
    }

    /**
     * Function that sets the rotation matrix around the center of mass
     * @param rotation New rotation matrix around the center of mass
     */
    @Override
    public void setRotation(RealMatrix rotation){
        if(!physicalVehicleInitialised) {
            throw new IllegalStateException("Rotation can only be set after initialisation.");
        }
        this.rotation = coordinateRotation.multiply(rotation.copy());
        // Get angles to set yaw_angle
        Rotation rot = new Rotation(rotation.getData(), 0.00000001);
        double[] angles = rot.getAngles(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR);
        yawAngle = angles[2] - Math.PI / 2;
    }

    /**
     * Function that returns a copy of the velocity vector of the center of mass
     * @return Velocity vector of the center of mass
     */
    @Override
    public RealVector getVelocity(){
        // Get current velocity
        RealVector localVelocity = new ArrayRealVector(new double[]{
                vehicleDynamicsModel.getValue("v_x"),
                vehicleDynamicsModel.getValue("v_y"),
                vehicleDynamicsModel.getValue("v_z")});
        // Return in global coordinates
        return rotation.operate(localVelocity);
    }

    /**
     * Function that sets the velocity vector of the center of mass
     * @param velocity New velocity vector of the center of mass
     */
    @Override
    public void setVelocity(RealVector velocity){
        if(physicalVehicleInitialised){
            throw new IllegalStateException("Velocity can only be set before initialisation.");
        }else{
            throw new UnsupportedOperationException("Setting the velocity before initialisation is done by the builder.");
        }
    }

    /**
     * Function that returns a copy of the angular velocity vector around the center of mass
     * @return Angular velocity vector around the center of mass
     */
    @Override
    public RealVector getAngularVelocity(){
        // Get current angular velocity
        RealVector localAngularVelocity = new ArrayRealVector(new double[]{0.0, 0.0,
                vehicleDynamicsModel.getValue("omega_z")});
        // Return in global coordinates
        return rotation.operate(localAngularVelocity);
    }

    /**
     * Function that sets the angular velocity vector around the center of mass
     * @param angularVelocity New angular velocity around of the center of mass
     */
    @Override
    public void setAngularVelocity(RealVector angularVelocity){
        if(physicalVehicleInitialised){
            throw new IllegalStateException("Angular velocity can only be set before initialisation.");
        }
        throw new UnsupportedOperationException("Setting the velocity before initialisation is done by the builder.");
    }

    /**
     * Function that adds an external force acting on the center of mass
     * @param force Force vector that acts on the center of mass
     */
    @Override
    public void addForce(RealVector force){
        this.force = this.force.add(force);
    }

    /**
     * Function that add an external torque acting around the center of mass
     * @param torque Torque vector that acts around the center of mass
     */
    public void addTorque(RealVector torque){
        //TODO: Expand the model to accept external torques
        throw new UnsupportedOperationException("External torques are currently not supported.");
    }

    /**
     * Function that returns the mass of the object
     * @return Mass of the physical object
     */
    @Override
    public double getMass(){
        return simulationVehicle.getMass();
    }

    /**
     * Function that sets the mass of the object
     * @param mass New mass of the physical object
     */
    @Override
    public void setMass(double mass){
        if(physicalVehicleInitialised) {
            throw new IllegalStateException("Mass can only be set before initialisation.");
        }
        simulationVehicle.setMass(mass);
    }

    /**
     * Function that returns a copy of center of geometry position vector
     * @return Position vector of the center of geometry
     */
    @Override
    public RealVector getGeometryPosition(){
        return position.add(getGeometryPositionOffset());
    }

    /**
     * Function that sets the center of geometry position vector.
     * @param geometryPosition New position vector of the center of geometry
     */
    @Override
    public void setGeometryPosition(RealVector geometryPosition){
        // Uses setter to check initialisation
        setPosition(geometryPosition.add(getGeometryPositionOffset().mapMultiply(-1.0)));

    }

    /**
     * Function that returns a copy of the vector pointing from the center of mass position to the center of geometry position
     * @return Offset vector of the center of mass position to the center of geometry position
     */
    @Override
    public RealVector getGeometryPositionOffset(){
        return this.rotation.operate(geometryPositionOffset).copy();
    }

    /**
     * Function that sets the vector pointing from the center of mass position to the center of geometry position
     * @param geometryPositionOffset New offset vector of the center of mass position to the center of geometry position
     */
    @Override
    public void setGeometryPositionOffset(RealVector geometryPositionOffset){
        throw new UnsupportedOperationException("Geometry position offset is determined by the chassis rotation.");
    }

    /**
     * Function that returns a list of pairs of 3D coordinates, indicating a vector on the edges of the physical object
     * @return List of pairs of 3D points, indicating a vector on the edges of the physical object
     */
    @Override
    @Deprecated
    public List<Map.Entry<RealVector, RealVector>> getBoundaryVectors(){
        //TODO: Function is unnecessary with three dimensional collision detection
        // Build relative vectors between vertices
        RealVector relVectorBackFront = new ArrayRealVector(new double[] {getLength(), 0.0, 0.0});
        RealVector relVectorLeftRight = new ArrayRealVector(new double[] {0.0, -getWidth(), 0.0});
        RealVector relVectorBottomTop = new ArrayRealVector(new double[] {0.0, 0.0, getHeight()});

        // Rotate relative vectors
        relVectorBackFront = getRotation().operate(relVectorBackFront);
        relVectorLeftRight = getRotation().operate(relVectorLeftRight);
        relVectorBottomTop = getRotation().operate(relVectorBottomTop);

        // From center coordinate, compute to bottom left vertex of box
        RealVector absBackLeft = getGeometryPosition();
        absBackLeft = absBackLeft.add(relVectorBackFront.mapMultiply(-0.5));
        absBackLeft = absBackLeft.add(relVectorLeftRight.mapMultiply(-0.5));
        absBackLeft = absBackLeft.add(relVectorBottomTop.mapMultiply(-0.5));

        // Compute absolute vectors
        RealVector backLeft = absBackLeft.copy();
        RealVector backRight = absBackLeft.add(relVectorLeftRight);
        RealVector frontLeft = absBackLeft.add(relVectorBackFront);
        RealVector frontRight = absBackLeft.add(relVectorLeftRight).add(relVectorBackFront);

        // Put vectors in list and return
        // Create map entries and insert them into list
        // Ordering is important here
        List<Map.Entry<RealVector, RealVector>> boundaryVectors = new LinkedList<>();
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(backLeft, backRight));
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(backRight, frontRight));
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontRight, frontLeft));
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontLeft, backLeft));
        return boundaryVectors;
    }

    /**
     * Function that computes one step of the physical behaviour of the object
     * @param deltaTime Duration of the current simulation step
     */
    @Override
    public void computePhysics(Duration deltaTime){
        if (!this.getError()) {
            // Calculate input values
            // Get values from VDM
            double z = vehicleDynamicsModel.getValue("z");

            // Reset vehicle on surface and calculate slope and bank
            RealVector roadPlane = position.add(rotation.operate(new ArrayRealVector(new double[]{0.0, 0.0, -z})));
            putOnSurface(roadPlane.getEntry(0), roadPlane.getEntry(1), yawAngle);

            // Do calculation steps with maximum step size as long as possible
            long currentDeltaTms = 0;
            int stepSizems = 2;
            while(currentDeltaTms + stepSizems <= deltaTime.toMillis()){
                doCalculationStep(stepSizems);
                currentDeltaTms = currentDeltaTms + stepSizems;
            }

            // Do a calculation step with partial step size to fill the gap
            long partialStepSize = deltaTime.toMillis() - currentDeltaTms;
            if(partialStepSize > 0) {
                doCalculationStep(partialStepSize);
            }

            // Update the rotation and position
            z = vehicleDynamicsModel.getValue("z");
            roadPlane = position.add(rotation.operate(new ArrayRealVector(new double[]{0.0, 0.0, -z})));
            putOnSurface(roadPlane.getEntry(0), roadPlane.getEntry(1), yawAngle);
        }
        // Reset forces
        force = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
    }

    /**
     * Function that sets the position of the center of mass and the rotation of the object, in order to place the object on the surface of the world.
     * given a x, y coordinate and a z rotation
     * @param posX X component of the position of the physical object
     * @param posY Y component of the position of the physical object
     * @param rotZ Z component of the rotation of the physical object
     */
    @Override
    public void putOnSurface(double posX, double posY, double rotZ){
        //Get values from VDM
        double z = vehicleDynamicsModel.getValue("z");
        double L_1 = vehicleDynamicsModel.getValue("L_1");
        double L_2 = vehicleDynamicsModel.getValue("L_2");
        double TW_f = vehicleDynamicsModel.getValue("TW_r");
        double TW_r = vehicleDynamicsModel.getValue("TW_r");

        //set on ground only rotated around z axis
        double ground = WorldModel.getInstance().getGround(posX, posY, 0.0).doubleValue();
        double posZ = ground + z;
        position = new ArrayRealVector(new double[]{posX, posY, posZ});

        //rotate around z axis
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, rotZ);
        rotation = coordinateRotation.multiply(new BlockRealMatrix(rot.getMatrix())).copy();

        //Get local positions
        RealVector frontPositionLocal = new ArrayRealVector(new double[]{L_1, 0.0, -z});
        RealVector backPositionLocal = new ArrayRealVector(new double[]{-L_2, 0.0, -z});
        RealVector leftPositionLocal = new ArrayRealVector(new double[]{0.0, (TW_f / 2 + TW_r / 2) / 2, -z});
        RealVector rightPositionLocal = new ArrayRealVector(new double[]{0.0, -(TW_f / 2 + TW_r / 2) / 2, -z});

        RealVector backToFrontLocal = new ArrayRealVector(new double[]{L_1 + L_2, 0.0, 0.0});

        //Get global positions only rotated around z axis
        RealVector frontPosition = position.add(rotation.operate(frontPositionLocal));
        RealVector backPosition = position.add(rotation.operate(backPositionLocal));
        RealVector leftPosition = position.add(rotation.operate(leftPositionLocal));
        RealVector rightPosition = position.add(rotation.operate(rightPositionLocal));

        //Get ground values for rotated wheel positions
        double frontGround = WorldModel.getInstance().getGround(frontPosition.getEntry(0), frontPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();
        double backGround = WorldModel.getInstance().getGround(backPosition.getEntry(0), backPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();
        double leftGround = WorldModel.getInstance().getGround(leftPosition.getEntry(0), leftPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();
        double rightGround = WorldModel.getInstance().getGround(rightPosition.getEntry(0), rightPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();

        //Store new ground value in global wheel position
        frontPosition.setEntry(2, frontGround);
        backPosition.setEntry(2, backGround);
        leftPosition.setEntry(2, leftGround);
        rightPosition.setEntry(2, rightGround);

        //Compute relative vectors
        RealVector backToFront = frontPosition.subtract(backPosition);
        RealVector rightToLeft = leftPosition.subtract(rightPosition);
        RealVector roadPlaneNorm = MathHelper.crossProduct(backToFront, rightToLeft);

        //Compute angles between relative vectors and X-Y-Plane
        RealVector xyPlaneNorm = new ArrayRealVector(new double[]{0.0, 0.0, 1.0});
        double backToFrontAngle = (Math.PI / 2) - MathHelper.angle(xyPlaneNorm, backToFront);
        double rightToLeftAngle = (Math.PI / 2) - MathHelper.angle(xyPlaneNorm, rightToLeft);

        if(physicalVehicleInitialised) {
            vehicleDynamicsModel.setInput("slope", backToFrontAngle);
            vehicleDynamicsModel.setInput("bank", rightToLeftAngle);
        }

        //The resulting rotation should transform the XY plane norm to the roadPlaneNorm
        //and the backToFrontLocal to the BackToFront

        Rotation finalRot = new Rotation(MathHelper.realTo3D(xyPlaneNorm),
                MathHelper.realTo3D(backToFrontLocal),
                MathHelper.realTo3D(roadPlaneNorm),
                MathHelper.realTo3D(backToFront));
        rotation = new BlockRealMatrix(finalRot.getMatrix());
        yawAngle = rotZ;

        //The rotation is occurring around the center of the road plane, so the position has to be shifted
        RealVector roadPlaneCenter = position.add(new ArrayRealVector(new double[]{0.0, 0.0, -z}));
        RealVector roadPlaneCenterToPositionLocal = new ArrayRealVector(new double[]{0.0, 0.0, z});
        position = roadPlaneCenter.add(rotation.operate(roadPlaneCenterToPositionLocal));
    }

    /**
     * Function that returns a copy of the position vector of the center of the front right wheel
     * @return Position vector of the center of the front right wheel
     */
    @Override
    public RealVector getFrontRightWheelGeometryPosition(){
        //Get values from VDM
        double L_1 = vehicleDynamicsModel.getValue(("L_1"));
        double TW_f = vehicleDynamicsModel.getValue("TW_f");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //Calculate localPosition and return global position
        RealVector localPosition = new ArrayRealVector(new double[]{L_1, -TW_f / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that returns a copy of the position vector of the center of the front left wheel
     * @return Position vector of the center of the front left wheel
     */
    @Override
    public RealVector getFrontLeftWheelGeometryPosition(){
        //Get values from VDM
        double L_1 = vehicleDynamicsModel.getValue(("L_1"));
        double TW_f = vehicleDynamicsModel.getValue("TW_f");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //Calculate localPosition and return global position
        RealVector localPosition = new ArrayRealVector(new double[]{L_1, TW_f / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that returns a copy of the position vector of the center of the back right wheel
     * @return Position vector of the center of the back right wheel
     */
    @Override
    public RealVector getBackRightWheelGeometryPosition(){
        //Get values from VDM
        double L_2 = vehicleDynamicsModel.getValue(("L_2"));
        double TW_r = vehicleDynamicsModel.getValue("TW_r");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //Calculate localPosition and return global position
        RealVector localPosition = new ArrayRealVector(new double[]{-L_2, -TW_r / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that returns a copy of the position vector of the center of the back left wheel
     * @return Position vector of the center of the back left wheel
     */
    @Override
    public RealVector getBackLeftWheelGeometryPosition(){
        //Get values from VDM
        double L_2 = vehicleDynamicsModel.getValue(("L_2"));
        double TW_r = vehicleDynamicsModel.getValue("TW_r");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //Calculate localPosition and return global position
        RealVector localPosition = new ArrayRealVector(new double[]{-L_2, TW_r / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that initialises the physics computations when the physicalVehicle is created
     * Should only be called by builder
     */
    @Override
    public void initPhysics() {
        if(physicalVehicleInitialised){
            throw new IllegalStateException("Physical Vehicle can only be initialised once.");
        }
        // Set parameters for the VDM
        vehicleDynamicsModel.setParameter("rho_air", PhysicsEngine.AIR_DENSITY);
        vehicleDynamicsModel.setParameter("g", - PhysicsEngine.GRAVITY_EARTH);

        // Initialise the modelica components
        vehicleDynamicsModel.initialise();

        //Shift position and geometryPositionOffset
        double z = vehicleDynamicsModel.getValue("z");
        geometryPositionOffset = new ArrayRealVector(new double[]{0.0, 0.0, simulationVehicle.getHeight() / 2 - z});
        position = geometryPositionOffset.mapMultiply(-1.0);

        //Initialise remaining variables
        force = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        yawAngle = 0.0;

        physicalVehicleInitialised = true;
        simulationVehicle.setVehicleInitialised(true);
    }

    /**
     * Function that returns the force that is acting on the vehicle
     * @return Force acting on the vehicle
     */
    @Override
    public RealVector getForce(){
        return force.copy();
    }

    /**
     * Function that returns the torque that is acting on the vehicle
     * @return Torque acting on the vehicle
     */
    @Override
    public RealVector getTorque(){
        //TODO: Expand the model to accept external torques
        throw new UnsupportedOperationException("External torques are currently not supported.");
    }

    /**
     * Function that returns the VDM
     * Should only be called by the builder and vehicle
     * @return The VDM of the vehicle
     */
    public VehicleDynamicsModel getVDM(){
        return vehicleDynamicsModel;
    }

    /**
     * Function that does a calculation step
     * @param deltaTms Length of the calculation step in milliseconds
     */
    private void doCalculationStep(long deltaTms){
        // Calculate input values
        // Get values from VDM
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        double m = vehicleDynamicsModel.getValue("m");
        double omega_wheel_1 = vehicleDynamicsModel.getValue("omega_wheel_1");
        double omega_wheel_2 = vehicleDynamicsModel.getValue("omega_wheel_2");
        double omega_wheel_3 = vehicleDynamicsModel.getValue("omega_wheel_3");
        double omega_wheel_4 = vehicleDynamicsModel.getValue("omega_wheel_4");

        // Get motor acceleration and convert it in torque
        double motorAcceleration = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).getActuatorValueCurrent();
        double motorForce = m * motorAcceleration;
        double motorTorque = r_nom * motorForce;
        vehicleDynamicsModel.setInput("tau_D_1", motorTorque / 4);
        vehicleDynamicsModel.setInput("tau_D_2", motorTorque / 4);
        vehicleDynamicsModel.setInput("tau_D_3", motorTorque / 4);
        vehicleDynamicsModel.setInput("tau_D_4", motorTorque / 4);

        // Get brake acceleration and convert it in torque
        double brakeAcceleration1 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).getActuatorValueCurrent();
        double brakeAcceleration2 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).getActuatorValueCurrent();
        double brakeAcceleration3 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).getActuatorValueCurrent();
        double brakeAcceleration4 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).getActuatorValueCurrent();
        double brakeForce1 = (m/4) * brakeAcceleration1;
        double brakeForce2 = (m/4) * brakeAcceleration2;
        double brakeForce3 = (m/4) * brakeAcceleration3;
        double brakeForce4 = (m/4) * brakeAcceleration4;
        double brakeTorque1 = r_nom * brakeForce1;
        double brakeTorque2 = r_nom * brakeForce2;
        double brakeTorque3 = r_nom * brakeForce3;
        double brakeTorque4 = r_nom * brakeForce4;
        vehicleDynamicsModel.setInput("tau_B_1", brakeTorque1 * Math.tanh(omega_wheel_1));
        vehicleDynamicsModel.setInput("tau_B_2", brakeTorque2 * Math.tanh(omega_wheel_2));
        vehicleDynamicsModel.setInput("tau_B_3", brakeTorque3 * Math.tanh(omega_wheel_3));
        vehicleDynamicsModel.setInput("tau_B_4", brakeTorque4 * Math.tanh(omega_wheel_4));

        // Get steering angle
        double steeringAngle = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();
        vehicleDynamicsModel.setInput("delta_1", - steeringAngle);
        vehicleDynamicsModel.setInput("delta_2", - steeringAngle);

        // Express the force vector in local coordinates
        RealVector localForce = rotation.transpose().operate(force);
        vehicleDynamicsModel.setInput("F_ext_x", localForce.getEntry(0));
        vehicleDynamicsModel.setInput("F_ext_y", localForce.getEntry(1));

        // Take the wheel positions and get the frictions coefficients
        vehicleDynamicsModel.setInput("mu_1", PhysicsEngine.calcFrictionCoefficient(getFrontLeftWheelGeometryPosition()));
        vehicleDynamicsModel.setInput("mu_2", PhysicsEngine.calcFrictionCoefficient(getFrontRightWheelGeometryPosition()));
        vehicleDynamicsModel.setInput("mu_3", PhysicsEngine.calcFrictionCoefficient(getBackLeftWheelGeometryPosition()));
        vehicleDynamicsModel.setInput("mu_4", PhysicsEngine.calcFrictionCoefficient(getBackRightWheelGeometryPosition()));

        // Store z coordinate for interpolation later
        double oldZ = vehicleDynamicsModel.getValue("z");

        // Do a computation step
        double deltaT = deltaTms / 1000.0;
        vehicleDynamicsModel.doStep(deltaT);

        // Integrate over model output
        // Integrate over the yaw rotation rate
        double omega_z = vehicleDynamicsModel.getValue("omega_z");
        yawAngle = yawAngle + omega_z * deltaT;

        // Integrate over the velocity
        double v_x = vehicleDynamicsModel.getValue("v_x");
        double v_y = vehicleDynamicsModel.getValue("v_y");
        double v_z = vehicleDynamicsModel.getValue("v_z");
        RealVector localVelocity = new ArrayRealVector(new double[]{v_x, v_y, v_z});
        RealVector velocity = rotation.operate(localVelocity);
        position = position.add(velocity.mapMultiply(deltaT));

        // Update geometryPositionOffset
        double z = vehicleDynamicsModel.getValue("z");
        RealVector deltaZ = new ArrayRealVector(new double[]{0.0, 0.0, oldZ - z});
        geometryPositionOffset = geometryPositionOffset.add(deltaZ);

        // Set velocity to zero when braking if very near to zero
        if(velocity.getNorm() <= 0.1 && brakeTorque1 > 0 && motorTorque == 0){
            // Set brake input to zero
            vehicleDynamicsModel.setInput("tau_B_1", 0.0);
            vehicleDynamicsModel.setInput("tau_B_2", 0.0);
            vehicleDynamicsModel.setInput("tau_B_3", 0.0);
            vehicleDynamicsModel.setInput("tau_B_4", 0.0);
            // Set chassis states to zero
            vehicleDynamicsModel.setInput("omega_wheel_1", 0.0);
            vehicleDynamicsModel.setInput("omega_wheel_2", 0.0);
            vehicleDynamicsModel.setInput("omega_wheel_3", 0.0);
            vehicleDynamicsModel.setInput("omega_wheel_4", 0.0);
            vehicleDynamicsModel.setInput("v_x", 0.0);
            vehicleDynamicsModel.setInput("v_y", 0.0);
            vehicleDynamicsModel.setInput("omega_z", 0.0);
            vehicleDynamicsModel.setInput("roll_angle", 0.0);
            vehicleDynamicsModel.setInput("omega_x", 0.0);
            vehicleDynamicsModel.setInput("pitch_angle", 0.0);
            vehicleDynamicsModel.setInput("omega_y", 0.0);
            // Set tires states to zero
            vehicleDynamicsModel.setInput("F_x_1", 0.0);
            vehicleDynamicsModel.setInput("F_x_2", 0.0);
            vehicleDynamicsModel.setInput("F_x_3", 0.0);
            vehicleDynamicsModel.setInput("F_x_4", 0.0);
            vehicleDynamicsModel.setInput("F_y_1", 0.0);
            vehicleDynamicsModel.setInput("F_y_2", 0.0);
            vehicleDynamicsModel.setInput("F_y_3", 0.0);
            vehicleDynamicsModel.setInput("F_y_4", 0.0);
        }
    }

    /**
     * Overwrite toString() to get a nice output for ModelicaPhysicalVehicles
     * @return String that contains all information of ModelicaPhysicalVehicles
     */
    @Override
    public String toString() {
        return  "PhysicalVehicle " + getId() +
                (physicalVehicleInitialised ? " , geometryPos: " + getGeometryPosition() : "") +
                (physicalVehicleInitialised ? " , position: " + position : "") +
                (physicalVehicleInitialised ? " , velocity: " + getVelocity() : "") +
                (physicalVehicleInitialised ? " , force: " + force : "") +
                (physicalVehicleInitialised ? " , rotation: " + rotation : "") +
                (physicalVehicleInitialised ? " , yawAngle: " + yawAngle : "") +
                (physicalVehicleInitialised ? " , physicalObjectType: " + physicalObjectType : "") +
                " , collision: " + collision +
                " , error: " + error +
                " , physicalVehicleInitialised: " + physicalVehicleInitialised +
                " , simulationVehicle: " + simulationVehicle;
    }
}