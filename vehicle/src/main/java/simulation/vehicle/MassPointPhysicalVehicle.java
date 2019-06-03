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
import simulation.environment.WorldModel;
import simulation.util.Log;
import simulation.util.MathHelper;
import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import static simulation.vehicle.MassPointType.*;
import static simulation.vehicle.PhysicsEngine.GRAVITY_EARTH;
import static simulation.vehicle.VehicleActuatorType.*;

/**
 * Class that represents a simulation vehicle with physical properties and interactions implemented according to the euler loop physics model
 */
public class MassPointPhysicalVehicle extends PhysicalVehicle {

    /** Attributes used for massPoint physics */
    /** x_cm bar of formula */
    private RealVector localPosition;

    /** x_cm of formula */
    private RealVector position;

    /** A of formula */
    private RealMatrix rotation;

    /** v_cm of formula */
    private RealVector velocity;

    /** omega of formula */
    private RealVector angularVelocity;

    /** L of formula */
    private RealVector angularMomentum;

    /** F of formula */
    private RealVector force;

    /** tau of formula */
    private RealVector torque;

    /** I bar ^-1 of formula */
    private RealMatrix localInertiaInverse;

    /** I ^-1 of formula */
    private RealMatrix inertiaInverse;

    /** Average tire pressure for car wheels */
    public static final double VEHICLE_DEFAULT_TIRE_PRESSURE = 2.5;

    /** Average car air drag coefficient*/
    public static final double AIR_DRAG_CAR = 0.3;


    /** Variables for the IPhysicalVehicle interface */
    /** Vector pointing from the center of mass position to the center of geometry position in the local coordinate system */
    private RealVector geometryPositionOffset;


    /** Components used for massPoint physics */
    /** Mass points of the car */
    private MassPoint[] massPoints = new MassPoint[4];


    /** Attributes used for massPoint physics */
    /** Angular velocity the car should have after initialisation */
    private RealVector targetAngularVelocity;


    /**
     * Constructor for an uninitialised physical vehicle
     */
    public MassPointPhysicalVehicle(){
        super();
        // Before initialisation
        // the center of mass position is at the center of the bottom side of the vehicle
        // no rotation
        this.position = new ArrayRealVector(new double[]{0.0, 0.0, - simulationVehicle.getHeight()/2});
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0);
        this.rotation = new BlockRealMatrix(rot.getMatrix());
        velocity = new ArrayRealVector(3);
        angularMomentum = new ArrayRealVector(3);

        // the center of geometry position is at the origin
        this.geometryPositionOffset = new ArrayRealVector(new double[]{0.0, 0.0, simulationVehicle.getHeight()/2});

        targetAngularVelocity = new ArrayRealVector(3);
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
            throw new IllegalStateException("Position can only be set after initialisation");
        }
        this.position = position.copy();
        // Recalculate mass point values that are based on the position
        calcMassPointCenterDiff();
        calcMassPointPosition();
        calcMassPointVelocity();
    }

    /**
     * Function that returns a copy of the rotation matrix around the center of mass
     * @return Rotation matrix around the center of mass
     */
    @Override
    public RealMatrix getRotation(){
        return this.rotation.copy();
    }

    /**
     * Function that sets the rotation matrix around the center of mass
     * @param rotation New rotation matrix around the center of mass
     */
    @Override
    public void setRotation(RealMatrix rotation){
        if(!physicalVehicleInitialised) {
            throw new IllegalStateException("Rotation can only be set after initialisation");
        }
        // Calculate the rotation to reach given rotation
        RealMatrix rotationDiff = rotation.multiply(this.rotation.transpose());
        rotationDiff = MathHelper.matrixReOrthonormalize(rotationDiff);

        // Apply rotation to all values
        this.rotation = rotationDiff.multiply(this.rotation);
        this.velocity = rotationDiff.operate(this.velocity);
        this.angularVelocity = rotationDiff.operate(this.angularVelocity);
        this.angularMomentum = rotationDiff.operate(this.angularMomentum);
        this.force = rotationDiff.operate(this.force);
        this.torque = rotationDiff.operate(this.torque);
        this.inertiaInverse = rotation.multiply(localInertiaInverse).multiply(rotation.transpose());

        // Recalculate the mass point values that are based on the rotation
        calcMassPointCenterDiff();
        calcMassPointPosition();
        calcMassPointVelocity();
    }

    /**
     * Function that returns a copy of the velocity vector of the center of mass
     * @return Velocity vector of the center of mass
     */
    @Override
    public RealVector getVelocity(){
        return this.velocity.copy();
    }

    /**
     * Function that sets the velocity vector of the center of mass
     * @param velocity New velocity vector of the center of mass
     */
    @Override
    public void setVelocity(RealVector velocity){
        if(physicalVehicleInitialised){
            throw new IllegalStateException("Velocity can only be set before the initialisation");
        }
        this.velocity  = velocity.copy();
    }

    /**
     * Function that returns a copy of the angular velocity vector around the center of mass
     * @return Angular velocity vector around the center of mass
     */
    @Override
    public RealVector getAngularVelocity(){
        return angularVelocity.copy();
    }

    /**
     * Function that sets the angular velocity vector around the center of mass
     * @param angularVelocity New angular velocity around of the center of mass
     */
    @Override
    public void setAngularVelocity(RealVector angularVelocity){
        if(physicalVehicleInitialised){
            throw new IllegalStateException("Angular velocity can only be set before the initialisation");
        }
        // In reality the angular momentum has to be set, but that is not possible before initialisation
        this.targetAngularVelocity = angularVelocity.copy();
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
    @Override
    public void addTorque(RealVector torque){
        this.torque = this.torque.add(torque);
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
            throw new IllegalStateException("Mass can only be set before the initialisation");
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
        // Uses setter to update mass point information and check initialisation
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
        throw new UnsupportedOperationException("Geometry position offset is determined by the mass point configuration.");
    }

    /**
     * Function that returns a list of pairs of 3D coordinates, indicating a vector on the edges of the physical object
     * @return List of pairs of 3D points, indicating a vector on the edges of the physical object
     */
    @Override
    public List<Map.Entry<RealVector, RealVector>> getBoundaryVectors(){
        //TODO: Function is unnecessary with three dimensional collision detection
        // Build relative vectors between vertices
        RealVector relVectorBackFront = new ArrayRealVector(new double[] {0.0, getLength(), 0.0});
        RealVector relVectorLeftRight = new ArrayRealVector(new double[] {getWidth(), 0.0 , 0.0});
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
     * @param deltaTms Duration of the current simulation step in milliseconds
     */
    @Override
    public void computePhysics(long deltaTms){
        if(!physicalVehicleInitialised){
            throw new IllegalStateException("Physical vehicle has to be initialised before physical computations");
        }
        double deltaT = deltaTms / 1000.0;

        if (!this.getError()) {
            // Calculate forces
            calcMassPointForces(deltaT);

            // Perform loop computations
            calcTorque();
            calcForce();
            calcPosition(deltaT);
            calcVelocity(deltaT);
            calcRotationMatrix(deltaT);
            calcAngularMomentum(deltaT);
            calcInertiaInverse();
            calcAngularVelocity();
            calcMassPointCenterDiff();
            calcMassPointPosition();
            calcMassPointVelocity();
        }

        // Reset forces and torques
        for(MassPoint mp : massPoints) {
            mp.resetForce();
        }
        force = new ArrayRealVector(3);
        torque = new ArrayRealVector(3);
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
        // Set vehicle on ground
        double groundZ = WorldModel.getInstance().getGround(posX, posY, getGeometryPosition().getEntry(2)).doubleValue();
        RealVector newPosition = new ArrayRealVector(new double[]{posX, posY, groundZ + getWheelRadius()});
        setPosition(newPosition);

        // Create rotation for Z, needed to get the correct ground values of wheel mass points
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, rotZ);
        RealMatrix newRotation = new BlockRealMatrix(rot.getMatrix());

        // Compute rotated positions of wheel mass points with new Z rotation
        RealVector frontLeft = getPosition().add(newRotation.operate(massPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getLocalCenterDiff()));
        RealVector frontRight = getPosition().add(newRotation.operate(massPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getLocalCenterDiff()));
        RealVector backLeft = getPosition().add(newRotation.operate(massPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getLocalCenterDiff()));
        RealVector backRight = getPosition().add(newRotation.operate(massPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getLocalCenterDiff()));

        // Get all ground values for new mass point X and Y coordinates
        double frontLeftGroundZ = WorldModel.getInstance().getGround(frontLeft.getEntry(0), frontLeft.getEntry(1), frontLeft.getEntry(2)).doubleValue();
        double frontRightGroundZ = WorldModel.getInstance().getGround(frontRight.getEntry(0), frontRight.getEntry(1), frontRight.getEntry(2)).doubleValue();
        double backLeftGroundZ = WorldModel.getInstance().getGround(backLeft.getEntry(0), backLeft.getEntry(1), backLeft.getEntry(2)).doubleValue();
        double backRightGroundZ = WorldModel.getInstance().getGround(backRight.getEntry(0), backRight.getEntry(1), backRight.getEntry(2)).doubleValue();

        // Store elevated ground values in all vectors
        frontLeft.setEntry(2, frontLeftGroundZ + getWheelRadius());
        frontRight.setEntry(2, frontRightGroundZ + getWheelRadius());
        backLeft.setEntry(2, backLeftGroundZ + getWheelRadius());
        backRight.setEntry(2, backRightGroundZ + getWheelRadius());

        // Compute relative vectors to estimate angles for rotations around X and Y axis
        RealVector backFrontLeftSide = frontLeft.subtract(backLeft);
        RealVector backFrontRightSide = frontRight.subtract(backRight);
        RealVector leftRightFrontSide = frontRight.subtract(frontLeft);
        RealVector leftRightBackSide = backRight.subtract(backLeft);

        // Compute all estimation angles between Z plane and relative vectors
        RealVector planeXYNormVector = new ArrayRealVector(new double[] {0.0, 0.0, 1.0});
        double angleBackFrontLeftSide = 0.0;
        double angleBackFrontRightSide = 0.0;
        double angleLeftRightFrontSide = 0.0;
        double angleLeftRightBackSide = 0.0;

        double normBackFrontLeftSide = backFrontLeftSide.getNorm() * planeXYNormVector.getNorm();
        if (normBackFrontLeftSide != 0.0) {
            double dotProduct = backFrontLeftSide.dotProduct(planeXYNormVector);
            double sinAngle = Math.abs(dotProduct) / normBackFrontLeftSide;
            angleBackFrontLeftSide = Math.asin(sinAngle);
            angleBackFrontLeftSide = (dotProduct < 0.0 ? -angleBackFrontLeftSide : angleBackFrontLeftSide);
        }

        double normBackFrontRightSide = backFrontRightSide.getNorm() * planeXYNormVector.getNorm();
        if (normBackFrontRightSide != 0.0) {
            double dotProduct = backFrontRightSide.dotProduct(planeXYNormVector);
            double sinAngle = Math.abs(dotProduct) / normBackFrontRightSide;
            angleBackFrontRightSide = Math.asin(sinAngle);
            angleBackFrontRightSide = (dotProduct < 0.0 ? -angleBackFrontRightSide : angleBackFrontRightSide);
        }

        double normLeftRightFrontSide = leftRightFrontSide.getNorm() * planeXYNormVector.getNorm();
        if (normLeftRightFrontSide != 0.0) {
            double dotProduct = leftRightFrontSide.dotProduct(planeXYNormVector);
            double sinAngle = Math.abs(dotProduct) / normLeftRightFrontSide;
            angleLeftRightFrontSide = Math.asin(sinAngle);
            angleLeftRightFrontSide = (dotProduct > 0.0 ? -angleLeftRightFrontSide : angleLeftRightFrontSide);
        }

        double normLeftRightBackSide = leftRightBackSide.getNorm() * planeXYNormVector.getNorm();
        if (normLeftRightBackSide != 0.0) {
            double dotProduct = leftRightBackSide.dotProduct(planeXYNormVector);
            double sinAngle = Math.abs(dotProduct) / normLeftRightBackSide;
            angleLeftRightBackSide = Math.asin(sinAngle);
            angleLeftRightBackSide = (dotProduct > 0.0 ? -angleLeftRightBackSide : angleLeftRightBackSide);
        }

        // From vector angles compute and set optimal rotation values based on ground levels
        double rotX = 0.5 * (angleBackFrontLeftSide + angleBackFrontRightSide);
        double rotY = 0.5 * (angleLeftRightFrontSide + angleLeftRightBackSide);

        // Set optimal angles
        rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, rotX, rotY, rotZ);
        newRotation = new BlockRealMatrix(rot.getMatrix());
        setRotation(newRotation);
    }

    /**
     * Function that returns a copy of the position vector of the center of the front right wheel
     * @return Position vector of the center of the front right wheel
     */
    @Override
    public RealVector getFrontRightWheelGeometryPosition(){
        return massPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPosition();
    }

    /**
     * Function that returns a copy of the position vector of the center of the front left wheel
     * @return Position vector of the center of the front left wheel
     */
    @Override
    public RealVector getFrontLeftWheelGeometryPosition(){
        return massPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPosition();
    }

    /**
     * Function that returns a copy of the position vector of the center of the back right wheel
     * @return Position vector of the center of the back right wheel
     */
    @Override
    public RealVector getBackRightWheelGeometryPosition(){
        return massPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getPosition();
    }

    /**
     * Function that returns a copy of the position vector of the center of the back left wheel
     * @return Position vector of the center of the back left wheel
     */
    @Override
    public RealVector getBackLeftWheelGeometryPosition(){
        return massPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPosition();
    }

    /**
     * Function that initialises the physics computations when the physicalVehicle is created
     * Should only be called by builder
     */
    @Override
    public void initPhysics() {
        if(physicalVehicleInitialised) {
            throw new IllegalStateException("Physical vehicle is already initialised");
        }
        // Reset geometryPositionOffset
        this.geometryPositionOffset = new ArrayRealVector(new double[]{0.0, 0.0, simulationVehicle.getHeight()/2});

        // Create Mass Points
        createMassPoints(
                simulationVehicle.getMass(),
                simulationVehicle.getWheelDistLeftRightFrontSide(),
                simulationVehicle.getWheelDistLeftRightBackSide(),
                simulationVehicle.getWheelDistToFront(),
                simulationVehicle.getWheelDistToBack());

        //Initialise values
        initLocalPosition();
        initMassPointLocalCenterDiff();
        initLocalInertiaInverse();
        calcInertiaInverse();

        // Set angular momentum so that the target angular velocity is achieved
        angularMomentum = MathHelper.matrixInvert(inertiaInverse).operate(targetAngularVelocity);

        // Set angular velocity
        calcAngularVelocity();

        // Initialise remaining mass point values
        calcMassPointCenterDiff();
        calcMassPointPosition();
        calcMassPointVelocity();

        // Rest is set to zero
        force = new ArrayRealVector(3);
        torque = new ArrayRealVector(3);

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
        return torque.copy();
    }

    /**
     * Function that returns the mass points
     * Should only be called for tests
     * @return The mass points of the vehicle
     */
    public MassPoint[] getMassPoints(){
        return massPoints;
    }

    /**
     * Function that creates the mass points with local position and mass
     * Should only be called by initMassPointPhysics
     *
     * @param mass Mass of the vehicle
     * @param wheelDistLeftRightFrontSide Distance between left and right wheels at front axel
     * @param wheelDistLeftRightBackSide Distance between left and right wheels at back axel
     * @param wheelDistToFront Distance between center of mass and front axel
     * @param wheelDistToBack Distance between center of mass and back axel
     */
    private void createMassPoints(double mass, double wheelDistLeftRightFrontSide, double wheelDistLeftRightBackSide, double wheelDistToFront, double wheelDistToBack) {
        // Calculate mass point positions
        RealVector localPositionFrontLeft = new ArrayRealVector(new double[]{-(wheelDistLeftRightFrontSide / 2), wheelDistToFront, 0.0});
        RealVector localPositionFrontRight = new ArrayRealVector(new double[]{(wheelDistLeftRightFrontSide / 2), wheelDistToFront, 0.0});
        RealVector localPositionBackLeft = new ArrayRealVector(new double[]{-(wheelDistLeftRightBackSide / 2), -wheelDistToBack, 0.0});
        RealVector localPositionBackRight = new ArrayRealVector(new double[]{(wheelDistLeftRightBackSide / 2), -wheelDistToBack, 0.0});

        // Calculate mass distribution
        double massFront = mass / ((wheelDistToFront/wheelDistToBack)+1);
        double massBack = mass / ((wheelDistToBack/wheelDistToFront)+1);

        massPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_FRONT_LEFT, localPositionFrontLeft, (massFront / 2));
        massPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_FRONT_RIGHT, localPositionFrontRight, (massFront / 2));
        massPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_BACK_LEFT, localPositionBackLeft, (massBack / 2));
        massPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_BACK_RIGHT, localPositionBackRight, (massBack / 2));

        massPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
        massPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
        massPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
        massPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
    }

    /**
     * Function that computes the center of mass position in the local coordinate system
     * Based on mass and local positions of the mass point
     * Should only be called by initMassPointPhysics
     */
    private void initLocalPosition() {
        RealVector result = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        for (MassPoint mp : massPoints) {
            result = result.add(mp.getLocalPosition().mapMultiplyToSelf(mp.getMass()));
        }
        result = result.mapDivideToSelf(getMass());
        localPosition = result;
    }

    /**
     * Function that computes the localCenterDiff values for all mass points
     * Based on local position of the mass point and local position of the physical vehicle
     * Should only be called by initMassPointPhysics
     */
    private void initMassPointLocalCenterDiff() {
        for (MassPoint mp : massPoints) {
            mp.setLocalCenterDiff(mp.getLocalPosition().subtract(localPosition));
        }
    }

    /**
     * Function that computes the localInertiaInverse of the physical vehicle
     * Based on mass and localCenterDiff of all mass points
     * Should only be called by initMassPointPhysics
     */
    private void initLocalInertiaInverse(){
        RealMatrix result = MatrixUtils.createRealMatrix(3, 3);
        for (MassPoint mp : massPoints) {
            RealMatrix matrixMassPoint = MathHelper.vectorToCrossProductMatrix(mp.getLocalCenterDiff()).power(2).scalarMultiply(-mp.getMass());
            result = result.add(matrixMassPoint);
        }
        localInertiaInverse = MathHelper.matrixInvert(result);
    }

    /**
     * Function that calculates the torque for the physicalVehicle
     * Based on current forces and center differences of vehicles mass points
     */
    private void calcTorque(){
        RealVector result = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        for (MassPoint mp : massPoints) {
            result = result.add(MathHelper.crossProduct(mp.getCenterDiff(), mp.getForce()));
        }
        torque = result;
    }

    /**
     * Function that calculates the force for the vehicle
     * Based on current forces of vehicles mass points
     */
    private void calcForce(){
        RealVector result = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        for (MassPoint mp : massPoints) {
            result = result.add(mp.getForce());
        }
        force = force.add(result);
        Double forceNorm = force.getNorm();
        if (forceNorm.isInfinite() || forceNorm.isNaN() || forceNorm > 1.0E10) {
            setError(true);
        }
    }

    /**
     * Function that calculates the position vector for the physicalVehicle
     * Based on current physicalVehicles velocity and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcPosition(double deltaT){
        position = position.add(velocity.mapMultiply(deltaT));
    }

    /**
     * Function that calculates the velocity and acceleration vector for the physicalVehicle
     * Based on current physicalVehicles force and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcVelocity(double deltaT){
        RealVector zeroVector = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        double threshold = 0.0001;
        velocity = velocity.add(force.mapMultiply(deltaT).mapDivide(getMass()));
        if(MathHelper.vectorEquals(velocity, zeroVector, threshold)){
            velocity = zeroVector;
        }
    }

    /**
     * Function that calculates the rotationMatrix for the physicalVehicle
     * Based on current physicalVehicles angularVelocity and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcRotationMatrix(double deltaT){
        rotation = rotation.add((MathHelper.vectorToCrossProductMatrix(angularVelocity).multiply(rotation)).scalarMultiply(deltaT));
        rotation = MathHelper.matrixReOrthonormalize(rotation);
    }

    /**
     * Function that calculates the angularMomentum for the physicalVehicle
     * Based on current physicalVehicles torque and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcAngularMomentum(double deltaT){
        angularMomentum = angularMomentum.add(torque.mapMultiply(deltaT));
    }

    /**
     * Function that calculates the inertiaInverse matrix for physicalVehicle
     * Based on current physicalVehicles localInertiaInverse and rotationMatrix
     */
    private void calcInertiaInverse(){
        inertiaInverse = rotation.multiply(localInertiaInverse).multiply(rotation.transpose());
    }

    /**
     * Function that calculates the angularVelocity vector for the physicalVehicle
     * Based on current physicalVehicles inertiaInverse and angularMomentum
     */
    private void calcAngularVelocity(){
        angularVelocity = inertiaInverse.operate(angularMomentum);
    }

    /**
     * Recalculates the r_i after one integration step
     */
    private void calcMassPointCenterDiff(){
        for(MassPoint massPoint : massPoints){
            massPoint.setCenterDiff(this.rotation.operate(massPoint.getLocalCenterDiff()));
        }
    }

    /**
     * Calculates the positions of the mass points
     */
    private void calcMassPointPosition(){
        for(MassPoint massPoint : massPoints){
            massPoint.setPosition(this.position.add(massPoint.getCenterDiff()));

            RealVector massPointPosition = massPoint.getPosition();
            double groundZ = WorldModel.getInstance().getGround(massPointPosition.getEntry(0), massPointPosition.getEntry(1), massPointPosition.getEntry(2)).doubleValue();
            massPoint.setGroundZ(groundZ);
            double limitZ = groundZ + simulationVehicle.getWheelRadius();

            // If mass point position goes way below ground position + wheel radius, then set computational error
            if (massPointPosition.getEntry(2) < (limitZ - 0.5 * simulationVehicle.getWheelRadius())) {
                //TODO: Build in check if physical vehicle is part of a simulation and then do not set the error if the vehicle is not part of a simulation;
            }
        }
    }

    /**
     * Function that calculates the velocity vector for all mass points
     */
    private void calcMassPointVelocity(){
        for(MassPoint massPoint : massPoints){
            massPoint.setVelocity(this.velocity.add(MathHelper.crossProduct(this.angularVelocity, massPoint.getCenterDiff())));
            RealVector zeroVector = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            double threshold = 0.0000000000001;
            if (MathHelper.vectorEquals(massPoint.getVelocity(), zeroVector, threshold)) {
                massPoint.setVelocity(zeroVector);
            }
        }
    }

    /**
     * Function that calculates the forces for each mass point of the vehicle
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcMassPointForces(double deltaT){
        // Iterate over wheel mass points
        for (MassPoint mp : massPoints) {

            // Force result of wheel mass point
            RealVector forceResult = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

            // Force: Acceleration force F = mass * acceleration
            RealVector forceAcceleration = calcAccelerationForce(mp, deltaT);
            forceResult = forceResult.add(forceAcceleration);

            Double forceAccelerationNorm = forceAcceleration.getNorm();
            if (forceAccelerationNorm.isInfinite() || forceAccelerationNorm.isNaN() || forceAccelerationNorm > 1.0E10) {
                Log.warning("Large forceAcceleration: " + forceAcceleration + " in MassPoint: " + mp + " for PhysicalVehicle: " + this);
            }

            // Force: Brake force F = mass * acceleration
            // Consider amount of acceleration, do not cause negative acceleration due to brakes
            RealVector forceBrake = calcBrakeForce(mp, deltaT);
            forceResult = forceResult.add(forceBrake);

            Double forceBrakeNorm = forceBrake.getNorm();
            if (forceBrakeNorm.isInfinite() || forceBrakeNorm.isNaN() || forceBrakeNorm > 1.0E10) {
                Log.warning("Large forceBrake: " + forceBrake + " in MassPoint: " + mp + " for PhysicalVehicle: " + this);
            }

            // Forces: Gravity, road friction, downhill force
            RealVector forcesRelatedToGravity = calcGravityRelatedForces(mp, deltaT);
            forceResult = forceResult.add(forcesRelatedToGravity);

            Double forcesRelatedToGravityNorm = forcesRelatedToGravity.getNorm();
            if (forcesRelatedToGravityNorm.isInfinite() || forcesRelatedToGravityNorm.isNaN() || forcesRelatedToGravityNorm > 1.0E10) {
                Log.warning("Large forcesRelatedToGravity: " + forcesRelatedToGravityNorm + " in MassPoint: " + mp + " for PhysicalVehicle: " + this);
            }

            // Force: Centripetal force Fc = mass * acceleration centrifugal = mass * (angularVelocity x (angularVelocity x radiusVector))
            RealVector forceCentripetal = calcCentripetalForce(mp);
            forceResult = forceResult.add(forceCentripetal);

            Double forceCentripetalNorm = forceCentripetal.getNorm();
            if (forceCentripetalNorm.isInfinite() || forceCentripetalNorm.isNaN() || forceCentripetalNorm > 1.0E10) {
                Log.warning("Large forceCentripetal: " + forceCentripetal + " in MassPoint: " + mp + " for PhysicalVehicle: " + this);
            }

            // Force: Air friction Fa = -0.5 * air density * velocity^2 * drag coefficient * area hit by wind
            RealVector forceAirFriction = calcAirFrictionForce(mp);
            forceResult = forceResult.add(forceAirFriction);

            Double forceAirFrictionNorm = forceAirFriction.getNorm();
            if (forceAirFrictionNorm.isInfinite() || forceAirFrictionNorm.isNaN() || forceAirFrictionNorm > 1.0E10) {
                Log.warning("Large forceAirFriction: " + forceAirFriction + " in MassPoint: " + mp + " for PhysicalVehicle: " + this);
            }

            // Set force to mass point
            mp.addForce(forceResult);
        }
    }

    /**
     * Function that calculates the acceleration force for a given mass point
     *
     * @param mp MassPoint for which force should be computed
     * @param deltaT Time difference to previous step in seconds
     * @return RealVector that represents the force
     */
    private RealVector calcAccelerationForce(MassPoint mp, double deltaT){
        // Check wheels with ground contact
        double groundContact = calcGroundContact(mp, deltaT);

        RealVector forceAcceleration = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        double countWheelGroundContact = 0.0;
        for (MassPoint mpTmp : massPoints) {
            if (calcGroundContact(mpTmp, deltaT) >= 0.5) {
                countWheelGroundContact = countWheelGroundContact + 1.0;
            }
        }

        if (mp.getType().ordinal() > MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal() || groundContact == 0.0 || countWheelGroundContact == 0.0) {
            return forceAcceleration;
        }

        // Compute motor acceleration values shared among wheels with ground contact
        double accelerationPerWheel = (4.0 * getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).getActuatorValueCurrent() / countWheelGroundContact);

        RealVector vehicleOrientation = getRotation().operate(new ArrayRealVector(new double[] {0.0, 1.0, 0.0}));
        double steeringAngle = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();

        // Scale force down when near zero velocity to avoid permanent positive / negative changes
        double velocityNorm = mp.getVelocity().getNorm();
        double brakeValueActuatorFrontLeft = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).getActuatorValueCurrent();
        double brakeValueActuatorFrontRight = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).getActuatorValueCurrent();
        double brakeValueActuatorBackLeft = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).getActuatorValueCurrent();
        double brakeValueActuatorBackRight = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).getActuatorValueCurrent();
        double brakeValueActuator = (brakeValueActuatorFrontLeft + brakeValueActuatorFrontRight + brakeValueActuatorBackLeft + brakeValueActuatorBackRight) /4;
        if (velocityNorm >= 0.0 && velocityNorm < 0.35 && brakeValueActuator >= Math.abs(accelerationPerWheel)) {
            accelerationPerWheel = 0.0;
        }

        // Force: Motor acceleration, F = mass * acceleration
        forceAcceleration = vehicleOrientation.mapMultiply(mp.getMass() * accelerationPerWheel);

        // Front wheels: Consider steering
        if (mp.getType().ordinal() == MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal() || mp.getType().ordinal() == MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()) {
            Rotation steerRotZ = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, -steeringAngle);
            RealMatrix steerRotZMatrix = new BlockRealMatrix(steerRotZ.getMatrix());
            forceAcceleration = steerRotZMatrix.operate(forceAcceleration);
        }

        return forceAcceleration.mapMultiply(groundContact);
    }

    /**
     * Function that calculates the brake force for a given mass point
     *
     * @param mp MassPoint for which force should be computed
     * @param deltaT Time difference to previous step in seconds
     * @return RealVector that represents the force
     */
    private RealVector calcBrakeForce(MassPoint mp, double deltaT){
        // Check wheels with ground contact
        double groundContact = calcGroundContact(mp, deltaT);

        RealVector forceBrake = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        if (mp.getType().ordinal() > MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal() || groundContact == 0.0) {
            return forceBrake;
        }
        // Individual brake force for each wheel
        double brakeValueActuator = 0.0;
        switch (mp.getType()) {
            case MASS_POINT_TYPE_WHEEL_FRONT_LEFT:
                brakeValueActuator = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).getActuatorValueCurrent();
                break;
            case MASS_POINT_TYPE_WHEEL_FRONT_RIGHT:
                brakeValueActuator = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).getActuatorValueCurrent();
                break;
            case MASS_POINT_TYPE_WHEEL_BACK_LEFT:
                brakeValueActuator = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).getActuatorValueCurrent();
                break;
            case MASS_POINT_TYPE_WHEEL_BACK_RIGHT:
                brakeValueActuator = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).getActuatorValueCurrent();
                break;
        }

        // Brakes work against mass point velocity with amount of acceleration
        forceBrake = mp.getVelocity().mapMultiply(-1.0);
        double velocityNorm = mp.getVelocity().getNorm();

        if (velocityNorm > 0.0) {
            forceBrake = forceBrake.mapDivide(velocityNorm);
        }

        // Scale force down when near zero velocity to avoid permanent positive / negative changes
        double brakeAmount = brakeValueActuator;
        if (velocityNorm >= 0.0 && velocityNorm < 0.35) {
            brakeAmount = velocityNorm * brakeValueActuator;
        }

        // Force: Brake force F = mass * acceleration
        // Consider amount of acceleration, do not cause negative acceleration due to brakes
        forceBrake = forceBrake.mapMultiply(mp.getMass() * brakeAmount);

        return forceBrake.mapMultiply(groundContact);
    }

    /**
     * Function that calculates the sum of all forces that are caused by gravity for a given mass point
     *
     * @param mp MassPoint for which force should be computed
     * @param deltaT Time difference to previous step in seconds
     * @return RealVector that represents the force
     */
    private RealVector calcGravityRelatedForces(MassPoint mp, double deltaT){
        // Check wheels with ground contact
        double groundContact = calcGroundContact(mp, deltaT);

        RealVector forcesGravityAll = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        RealVector forcesGravityGround = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        RealVector forcesGravityFalling = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        // Skip invalid mass points to avoid out of array bounds accesses
        if (mp.getType().ordinal() > MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()) {
            return forcesGravityAll;
        }

        // Force: Gravity force Fg = mass * gravity constant
        RealVector forceGravity = new ArrayRealVector(new double[] {0.0, 0.0, mp.getMass() * GRAVITY_EARTH});

        // Compute forces related to ground contact
        if (groundContact > 0.0) {

            // Add gravity force
            forcesGravityGround = forcesGravityGround.add(forceGravity);

            // Determine angle information of vehicle
            RealVector backFront1 = (massPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPosition().subtract(massPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPosition()));
            RealVector backFront2 = (massPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPosition().subtract(massPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getPosition()));
            RealVector leftRight1 = (massPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPosition().subtract(massPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPosition()));
            RealVector leftRight2 = (massPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getPosition().subtract(massPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPosition()));

            // Compute average vectors
            RealVector vectorBackFront = backFront1.add(backFront2).mapMultiply(0.5);
            RealVector vectorLeftRight = leftRight1.add(leftRight2).mapMultiply(0.5);

            // Force: Normal force Fn = mass * gravity constant (with correct angles)
            RealVector forceNormal = MathHelper.crossProduct(vectorBackFront, vectorLeftRight);

            // Ensure that normal force points upwards
            if (forceNormal.getEntry(2) < 0.0) {
                forceNormal = forceNormal.mapMultiply(-1.0);
            }

            // Normalize vector
            double forceNormalNorm = forceNormal.getNorm();
            if (forceNormalNorm != 0.0) {
                forceNormal = forceNormal.mapDivide(forceNormalNorm);
            }

            // Set correct length of vector and add it to final result
            double forceNormalAmount = Math.abs(mp.getMass() * GRAVITY_EARTH);
            forceNormal = forceNormal.mapMultiply(forceNormalAmount);
            forcesGravityGround = forcesGravityGround.add(forceNormal);

            // Split normal force vector to get fractions in backFront and leftRight directions
            // Compute angle between normal force and z plane, with this angle compute x, y normal force component vector in z plane
            double angleNormalZ = 0.0;
            RealVector planeZVector = new ArrayRealVector(new double[] {0.0, 0.0, 1.0});

            if (forceNormal.getNorm() != 0.0 && planeZVector.getNorm() != 0.0) {
                angleNormalZ = Math.acos(forceNormal.cosine(planeZVector));
            }

            RealVector componentXYVector = new ArrayRealVector(new double[] {forceNormal.getEntry(0), forceNormal.getEntry(1), 1.0});
            double normComponentXYVector = componentXYVector.getNorm();

            if (normComponentXYVector > 0.0) {
                componentXYVector = componentXYVector.mapDivide(normComponentXYVector);
            }

            // Rotate x, y normal force component vector to origin, then split based on axis
            componentXYVector = componentXYVector.mapMultiply(angleNormalZ * forceNormal.getNorm());
            componentXYVector = rotation.transpose().operate(componentXYVector);

            double fractionNormalLeftRight = 0.0;
            double fractionNormalBackFront = 0.0;
            double fractionNormalLengths = Math.abs(componentXYVector.getEntry(0)) + Math.abs(componentXYVector.getEntry(1));

            if (fractionNormalLengths != 0.0) {
                fractionNormalBackFront = 2.0 * (Math.abs(componentXYVector.getEntry(1)) / fractionNormalLengths);
                fractionNormalLeftRight = 2.0 * (Math.abs(componentXYVector.getEntry(0)) / fractionNormalLengths);
            } else {
                fractionNormalBackFront = 1.0;
                fractionNormalLeftRight = 1.0;
            }

            // Compute amounts of normal forces in different directions
            double forceNormalLengthLeftRight = fractionNormalLeftRight * forceNormal.getNorm();
            double forceNormalLengthBackFront = fractionNormalBackFront * forceNormal.getNorm();

            // Split velocity vector
            RealVector mpVelocityWheels = new ArrayRealVector(new double[] {0.0, 1.0, 0.0});
            RealVector mpVelocityWheelsOrthogonal = new ArrayRealVector(new double[] {1.0, 0.0, 0.0});
            double steeringAngle = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();

            // For rotation matrix it holds transpose(matrix) = inverse(matrix)
            RealVector mpVelocityOrigin = rotation.transpose().operate(mp.getVelocity());

            // Front wheels: Consider steering
            if (mp.getType().ordinal() == MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal() || mp.getType().ordinal() == MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()) {
                Rotation steerRotZ = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, steeringAngle);
                RealMatrix steerRotZMatrix = new BlockRealMatrix(steerRotZ.getMatrix());
                mpVelocityOrigin = steerRotZMatrix.operate(mpVelocityOrigin);
            }

            // Fractions of velocity vector in x and y directions
            double fractionWheels = 0.0;
            double fractionWheelsOrthogonal = 0.0;
            double fractionWheelsLengths = Math.abs(mpVelocityOrigin.getEntry(0)) + Math.abs(mpVelocityOrigin.getEntry(1));

            if (fractionWheelsLengths != 0.0) {
                fractionWheels = (Math.abs(mpVelocityOrigin.getEntry(1)) / fractionWheelsLengths);
                fractionWheelsOrthogonal = (Math.abs(mpVelocityOrigin.getEntry(0)) / fractionWheelsLengths);
            }

            mpVelocityWheels.setEntry(1, mpVelocityOrigin.getEntry(1));
            mpVelocityWheels.setEntry(2, fractionWheels * mpVelocityOrigin.getEntry(2));
            mpVelocityWheelsOrthogonal.setEntry(0, mpVelocityOrigin.getEntry(0));
            mpVelocityWheelsOrthogonal.setEntry(2, fractionWheelsOrthogonal * mpVelocityOrigin.getEntry(2));

            // Front wheels: Consider steering
            if (mp.getType().ordinal() == MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal() || mp.getType().ordinal() == MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()) {
                Rotation steerRotZ = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, -steeringAngle);
                RealMatrix steerRotZMatrix = new BlockRealMatrix(steerRotZ.getMatrix());
                mpVelocityWheels = steerRotZMatrix.operate(mpVelocityWheels);
                mpVelocityWheelsOrthogonal = steerRotZMatrix.operate(mpVelocityWheelsOrthogonal);
            }

            mpVelocityWheels = rotation.operate(mpVelocityWheels);
            mpVelocityWheelsOrthogonal = rotation.operate(mpVelocityWheelsOrthogonal);

            // Road friction: Back-Front: Rolling resistance in the direction of the wheels
            RealVector forceRoadFrictionBackFront = mpVelocityWheels.mapMultiply(-1.0);
            double forceRoadFrictionBackFrontNorm = forceRoadFrictionBackFront.getNorm();
            double pressure = (mp.getPressure() > 0.0 ? mp.getPressure() : VEHICLE_DEFAULT_TIRE_PRESSURE);
            double rollingCoefficient = PhysicsEngine.calcRollingResistance(getPosition(), pressure, forceRoadFrictionBackFrontNorm);

            if (forceRoadFrictionBackFrontNorm > 0.0) {
                forceRoadFrictionBackFront = forceRoadFrictionBackFront.mapDivide(forceRoadFrictionBackFrontNorm);
            } else {
                // If there is no velocity, there should not be any rolling coefficient
                rollingCoefficient = 0.0;
            }

            // Scale force down when near zero velocity to avoid permanent positive / negative changes
            if (forceRoadFrictionBackFrontNorm >= 0.0 && forceRoadFrictionBackFrontNorm < 0.35) {
                rollingCoefficient = forceRoadFrictionBackFrontNorm * rollingCoefficient;
            }

            forceRoadFrictionBackFront = forceRoadFrictionBackFront.mapMultiply(rollingCoefficient * forceNormalLengthBackFront);
            forcesGravityGround = forcesGravityGround.add(forceRoadFrictionBackFront);

            // Road friction: Left-Right: Resistance against wheels moving sideways
            RealVector forceRoadFrictionLeftRight = mpVelocityWheelsOrthogonal.mapMultiply(-1.0);
            double forceRoadFrictionLeftRightNorm = forceRoadFrictionLeftRight.getNorm();

            if (forceRoadFrictionLeftRightNorm > 0.0) {
                forceRoadFrictionLeftRight = forceRoadFrictionLeftRight.mapDivide(forceRoadFrictionLeftRightNorm);
            }

            double forceRoadFrictionLeftRightAmount = PhysicsEngine.calcFrictionCoefficient(getPosition()) * forceNormalLengthLeftRight;

            // Scale force down when near zero velocity to avoid permanent positive / negative changes
            if (forceRoadFrictionLeftRightNorm >= 0.0 && forceRoadFrictionLeftRightNorm < 0.35) {
                forceRoadFrictionLeftRightAmount = forceRoadFrictionLeftRightNorm * forceRoadFrictionLeftRightAmount;
            }

            forceRoadFrictionLeftRight = forceRoadFrictionLeftRight.mapMultiply(forceRoadFrictionLeftRightAmount);
            forcesGravityGround = forcesGravityGround.add(forceRoadFrictionLeftRight);
            forcesGravityGround = forcesGravityGround.mapMultiply(groundContact);
        }

        // No ground contact, mass point is falling
        if (groundContact < 1.0) {
            // Add gravity force
            forcesGravityFalling = forcesGravityFalling.add(forceGravity);
            forcesGravityFalling = forcesGravityFalling.mapMultiply(1.0 - groundContact);

            // Impact impulse J = mass * delta velocity = F_average * delta time (for change)
            if (groundContact > 0.0) {
                double velocityZ = (1.0 - groundContact) * deltaT * GRAVITY_EARTH + mp.getVelocity().getEntry(2);

                if (velocityZ < 0.0) {
                    double impactImpulse = mp.getMass() * Math.abs(velocityZ);
                    double forceImpactAverageAmount = impactImpulse / 0.01;
                    RealVector forceImpactAverage = new ArrayRealVector(new double[] {0.0, 0.0, 1.0});
                    forceImpactAverage = forceImpactAverage.mapMultiply(forceImpactAverageAmount);
                    forcesGravityFalling = forcesGravityFalling.add(forceImpactAverage);
                }
            }
        }

        forcesGravityAll = forcesGravityGround.add(forcesGravityFalling);

        return forcesGravityAll;
    }

    /**
     * Function that calculates the centripetal force for a given mass point
     *
     * @param mp MassPoint for which force should be computed
     * @return RealVector that represents the force
     */
    private RealVector calcCentripetalForce(MassPoint mp){
        // Force: Centripetal force Fc = mass * acceleration centrifugal = mass * (angularVelocity x (angularVelocity x radiusVector))
        RealVector forceCentripetal = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        double steeringAngle = getSimulationVehicle().getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();

        // Do not compute centripetal force for very small steering angles
        // Values very close to 0 are equivalent to a nearly infinite turning radius, leading to wrong force results
        // 0.02 rad = 1.1 deg means that turning radius is limited to a value of about 150 meters
        if (Math.abs(steeringAngle) < 0.02) {
            return forceCentripetal;
        }

        double curveRadiusSin = Math.sin(steeringAngle);

        if (curveRadiusSin != 0.0) {
            double wheelDistFrontBack = getSimulationVehicle().getWheelDistToFront() + getSimulationVehicle().getWheelDistToBack();
            double curveRadiusVectorLength = (wheelDistFrontBack / curveRadiusSin);
            RealVector curveRadiusVector = (massPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPosition().subtract(massPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPosition()));

            if (curveRadiusVector.getNorm() > 0.0) {
                curveRadiusVector = curveRadiusVector.mapDivide(curveRadiusVector.getNorm());
            }

            curveRadiusVector = curveRadiusVector.mapMultiply(curveRadiusVectorLength);
            forceCentripetal = MathHelper.crossProduct(angularVelocity, MathHelper.crossProduct(angularVelocity, curveRadiusVector)).mapMultiply(mp.getMass());
        }
        return forceCentripetal;
    }

    /**
     * Function that calculates the air friction force for a given mass point
     *
     * @param mp MassPoint for which force should be computed
     * @return RealVector that represents the force
     */
    private RealVector calcAirFrictionForce(MassPoint mp){
        // Force: Air friction Fa = -0.5 * air density * velocity^2 * drag coefficient * area hit by wind
        RealVector forceAirFriction = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        RealVector mpVelocity = mp.getVelocity();

        if (mpVelocity.getNorm() == 0.0) {
            return forceAirFriction;
        }

        double areaX = 0.25 * getSimulationVehicle().getHeight() * getSimulationVehicle().getLength();
        double areaY = 0.25 * getSimulationVehicle().getHeight() * getSimulationVehicle().getWidth();
        double areaZ = 0.25 * getSimulationVehicle().getLength() * getSimulationVehicle().getWidth();

        // For a rotation matrix it holds: inverse(matrix) = transpose(matrix)
        // Rotate velocityOrientation back to global coordinate system axis to match up with area values
        RealVector velocityOrigin = rotation.transpose().operate(mpVelocity);

        // Fractions of area values for each axis according to velocity vector orientation and with no more vehicle rotation
        areaX = areaX * (velocityOrigin.getEntry(0) / velocityOrigin.getL1Norm());
        areaY = areaY * (velocityOrigin.getEntry(1) / velocityOrigin.getL1Norm());
        areaZ = areaZ * (velocityOrigin.getEntry(2) / velocityOrigin.getL1Norm());

        // Sum of all fractions for area values yields correct approximation of total area for air resistance
        double area = Math.abs(areaX) + Math.abs(areaY) + Math.abs(areaZ);

        // Scalar for air friction force computations
        //TODO: Let the physical vehicle look up the ground type and not only the weather
        double scalarCoefficient = -0.5 * PhysicsEngine.AIR_DENSITY * MassPointPhysicalVehicle.AIR_DRAG_CAR * area;

        // Final force computation, preserve direction that we need for computations in the 3D space
        RealVector direction = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        direction.setEntry(0, (mpVelocity.getEntry(0) < 0.0 ? -1.0 : 1.0));
        direction.setEntry(1, (mpVelocity.getEntry(1) < 0.0 ? -1.0 : 1.0));
        direction.setEntry(2, (mpVelocity.getEntry(2) < 0.0 ? -1.0 : 1.0));
        forceAirFriction = mpVelocity.ebeMultiply(mpVelocity).ebeMultiply(direction).mapMultiply(scalarCoefficient);
        return forceAirFriction;
    }

    /**
     * Function that calculates the fraction between 0.0 and 1.0 describing how much ground contact
     * the mass point will probably have in the current time step to adjust forces, this is just an approximation
     *
     * @param mp MassPoint for which ground fraction should be computed
     * @param deltaT Time difference to previous step in seconds
     * @return Fraction of ground contact in current time step between 0.0 and 1.0
     */
    private double calcGroundContact(MassPoint mp, double deltaT) {

        double groundFraction = 0.0;

        double velocityZ = mp.getVelocity().getEntry(2);
        double accelerationZ;

        double groundZ = mp.getGroundZ();
        double limitZ = groundZ + getSimulationVehicle().getWheelRadius();
        double groundDistance = (mp.getPosition().getEntry(2) - limitZ);

        if (groundDistance > 1.0E-8) {
            accelerationZ = GRAVITY_EARTH;
        } else {
            return 1.0;
        }

        double zDistance = 0.5 * accelerationZ * deltaT * deltaT + velocityZ * deltaT;

        if (groundDistance > 0.0 && groundDistance + zDistance < 0.0) {
            groundFraction = 1.0 - (groundDistance / Math.abs(zDistance));
        }

        return groundFraction;
    }

    /**
     * Overwrite toString() to get a nice output for MassPointPhysicalVehicles
     * @return String that contains all information of MassPointPhysicalVehicles
     */
    @Override
    public String toString() {
        return  "PhysicalVehicle " + getId() +
                (physicalVehicleInitialised ? " , geometryPos: " + getGeometryPosition() : "") +
                (physicalVehicleInitialised ? " , localPosition: " + localPosition : "") +
                " , position: " + position +
                (physicalVehicleInitialised ? " , velocity: " + velocity : "") +
                (physicalVehicleInitialised ? " , force: " + force : "") +
                (physicalVehicleInitialised ? " , localInertiaInverse: " + localInertiaInverse : "") +
                (physicalVehicleInitialised ? " , inertiaInverse: " + inertiaInverse : "") +
                (physicalVehicleInitialised ? " , rotation: " + rotation : "") +
                (physicalVehicleInitialised ? " , angularVelocity: " + angularVelocity : "") +
                (physicalVehicleInitialised ? " , angularMomentum: " + angularMomentum : "") +
                (physicalVehicleInitialised ? " , torque: " + torque : "") +
                (physicalVehicleInitialised ? " , physicalObjectType: " + physicalObjectType : "") +
                " , collision: " + collision +
                " , error: " + error +
                " , physicalVehicleInitialised: " + physicalVehicleInitialised +
                (physicalVehicleInitialised ? " , massPoints[0]: " + massPoints[0] : "") +
                (physicalVehicleInitialised ? " , massPoints[1]: " + massPoints[1] : "") +
                (physicalVehicleInitialised ? " , massPoints[2]: " + massPoints[2] : "") +
                (physicalVehicleInitialised ? " , massPoints[3]: " + massPoints[3] : "") +
                " , simulationVehicle: " + simulationVehicle;
    }
}