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

    /** Variables for the IPhysicalVehicle interface */
    /** Vector pointing from the center of mass position to the center of geometry position in the local coordinate system */
    private RealVector geometryPositionOffset;


    /** Attributes used for massPoint physics */
    /** x_cm bar of formula */
    private RealVector localPosition;

    /** x_cm of formula */
    private RealVector position;

    /** v_cm / x_cm dot of formula */
    private RealVector velocity;

    /** x_cm dot dot of formula */
    private RealVector acceleration;

    /** F of formula */
    private RealVector force;

    /** I bar ^-1 of formula */
    private RealMatrix localInertiaInverse;

    /** I ^-1 of formula */
    private RealMatrix inertiaInverse;

    /** A of formula */
    private RealMatrix rotation;

    /** omega of formula */
    private RealVector angularVelocity;

    /** L of formula */
    private RealVector angularMomentum;

    /** tau of formula */
    private RealVector angularMomentumDeriv;

    /** Average tire pressure for car wheels */
    public static final double VEHICLE_DEFAULT_TIRE_PRESSURE = 2.5;


    /** Components used for massPoint physics */
    /** Mass points of the car */
    private MassPoint[] wheelMassPoints = new MassPoint[4];


    /**
     * Constructor for an uninitialized physical vehicle
     */
    public MassPointPhysicalVehicle(){
        super();
        // Before the initialisation the center of geometry position is at the origin
        // Sets origin of the local coordinate system to the center of the bottom side of the vehicle
        position = new ArrayRealVector(new double[]{0.0, 0.0, simulationVehicle.getHeight() * -0.5});
        // Set geometry position offset, works because the center of geometry position is at the origin
        geometryPositionOffset = position.mapMultiply(-1.0);

        Log.finest("PhysicalVehicle: Constructor - PhysicalVehicle constructed: " + this);
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
        if(physicalVehicleInitialized) {
            this.position = position.copy();
            // Recalculate the mass point values that are based on the position
            calcMassPointCenterDiff();
            calcMassPointPosition();
            calcMassPointVelocity();
        }
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
        if(physicalVehicleInitialized) {
            this.rotation = rotation.copy();
            // Recalculate the mass point values that are based on the rotation
            calcMassPointCenterDiff();
            calcMassPointPosition();
            calcMassPointVelocity();
        }
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
        // this.velocity = velocity.copy();
        // toDo why
    }

    /**
     * Function that returns a copy of the acceleration vector of the center of mass
     * @return Acceleration vector of the center of mass
     */
    @Override
    public RealVector getAcceleration(){
        return this.acceleration.copy();
    }

    /**
     * Function that sets the accelerations vector of the center of mass
     * @param acceleration New acceleration vector of the center of mass
     */
    @Override
    public void setAcceleration(RealVector acceleration){
        // this.acceleration = acceleration.copy();
        // toDo why
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
        // Uses setter to update mass point information
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
        // toDo why not
    }

    /**
     * Function that sets the error flag
     * @param error New Error flag of the physical object
     */
    @Override
    public void setError(boolean error){
        Log.warning("PhysicalVehicle: setError - error: " + error + ", PhysicalVehicle at start: " + this);
        this.error = error;

        if (error) {
            this.velocity = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            this.acceleration = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            this.angularVelocity = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            this.angularMomentum = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            this.angularMomentumDeriv = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            for(MassPoint point : wheelMassPoints){
                point.setVelocity(new ArrayRealVector(new double[] {0.0, 0.0, 0.0}));
                point.setForce(new ArrayRealVector(new double[] {0.0, 0.0, 0.0}));
            }
        }

        Log.warning("PhysicalVehicle: setError - error: " + error + ", PhysicalVehicle at end: " + this);
    }

    /**
     * Function that returns a list of pairs of 3D coordinates, indicating a vector on the edges of the physical object
     * @return List of pairs of 3D points, indicating a vector on the edges of the physical object
     */
    @Override
    public List<Map.Entry<RealVector, RealVector>> getBoundaryVectors(){
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
        double deltaT = deltaTms / 1000.0;
        if (!this.getError()) {
            // Calculate forces
            calcMassPointForces(deltaT);

            // Perform loop computations
            calcAngularMomentumDeriv();
            calcForce();
            calcPosition(deltaT);
            calcVelocityAndAcceleration(deltaT);
            calcRotationMatrix(deltaT);
            calcAngularMomentum(deltaT);
            calcInertiaInverse();
            calcAngularVelocity();
            calcMassPointCenterDiff();
            calcMassPointPosition();
            calcMassPointVelocity();
        }
        // Reset forces
        for(MassPoint mp : wheelMassPoints) {
            mp.setForce(new ArrayRealVector(new double[] {0.0, 0.0, 0.0})) ;
        }
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
        // Set vehicle on ground
        double groundZ = WorldModel.getInstance().getGround(posX, posY, getGeometryPosition().getEntry(2)).doubleValue();
        RealVector newPosition = new ArrayRealVector(new double[]{posX, posY, groundZ + getWheelRadius()});
        setPosition(newPosition);

        // Create rotation for Z, needed to get the correct ground values of wheel mass points
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, rotZ);
        RealMatrix newRotation = new BlockRealMatrix(rot.getMatrix());

        // Compute rotated positions of wheel mass points with new Z rotation
        RealVector frontLeft = getPosition().add(newRotation.operate(wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getLocalCenterDiff()));
        RealVector frontRight = getPosition().add(newRotation.operate(wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getLocalCenterDiff()));
        RealVector backLeft = getPosition().add(newRotation.operate(wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getLocalCenterDiff()));
        RealVector backRight = getPosition().add(newRotation.operate(wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getLocalCenterDiff()));

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


    /*====================*/

    /**
     * Marked as deprecated in favour of getGeometryPosition
     * Function that returns a vector with the x, y and z coordinates of the object
     * This refers to the center position of the geometry object (i.e. NOT mass point position)
     * @return Vector with x, y, z coordinates of the object center
     */
    @Override
    @Deprecated
    public RealVector getGeometryPos(){
        return  getGeometryPosition();
    }

    /**
     * Marked as deprecated in favour of getRotation
     * Function that returns a matrix with the rotation of the object
     * @return Matrix with the rotation of the object
     */
    @Override
    @Deprecated
    public RealMatrix getGeometryRot(){
        return getRotation();
    }

    /**
     * Function that returns a copy of the position vector of the center of the front right wheel
     * @return Position vector of the center of the front right wheel
     */
    @Override
    public RealVector getFrontRightWheelGeometryPosition(){
        return wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPos();
    }

    /**
     * Function that returns a copy of the position vector of the center of the front left wheel
     * @return Position vector of the center of the front left wheel
     */
    @Override
    public RealVector getFrontLeftWheelGeometryPosition(){
        return wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPos();
    }

    /**
     * Function that returns a copy of the position vector of the center of the back right wheel
     * @return Position vector of the center of the back right wheel
     */
    @Override
    public RealVector getBackRightWheelGeometryPosition(){
        return wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getPos();
    }

    /**
     * Function that returns a copy of the position vector of the center of the back left wheel
     * @return Position vector of the center of the back left wheel
     */
    @Override
    public RealVector getBackLeftWheelGeometryPosition(){
        return wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPos();
    }

    /**
     * Function that requests the called object to update its state for given time difference
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    @Override
    public void executeLoopIteration(long timeDiffMs) {

        simulationVehicle.updateAllSensors();

        if (!this.error) {
            Log.finest("PhysicalVehicle: executeLoopIteration - timeDiffMs: " + timeDiffMs + ", PhysicalVehicle at start: " + this);

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
                // TODO: This logic should be moved to the controller!
                try {
                    simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).setActuatorValueCurrent(0.0);
                    simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).setActuatorValueCurrent(0.0);
                    simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).setActuatorValueCurrent(0.0);
                    simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).setActuatorValueCurrent(0.0);
                    simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).setActuatorValueCurrent(0.0);
                }
                catch (Exception e){
                    e.printStackTrace();
                }
            }

            simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).update(deltaT);
            this.collision = false;

            Log.finest("PhysicalVehicle: executeLoopIteration - timeDiffMs: " + timeDiffMs +  ", PhysicalVehicle at end: " + this);
        } else {
            Log.finest("PhysicalVehicle: Vehicle collided or had a computational error and will therefore not move anymore, PhysicalVehicle: " + this);
        }
    }

    /**
     * Function that initializes the massPoint physics computations when the physicalVehicle is created
     * Should only be called by builder
     */
    @Override
    public void initPhysics() {
        if(!physicalVehicleInitialized) {
            // Resets origin of the local coordinate system to the center of the bottom side of the vehicle
            // Works, because between construction and initialisation the position and rotation of the physicalVehicle have not changed
            position = new ArrayRealVector(new double[]{0.0, 0.0, simulationVehicle.getHeight() * -0.5});
            // Set geometry position offset, works because the center of geometry position is at the origin
            geometryPositionOffset = position.mapMultiply(-1.0);

            // Create Mass Points
            createMassPoints(
                    simulationVehicle.getMassFront(),
                    simulationVehicle.getMassBack(),
                    simulationVehicle.getWheelDistLeftRight(),
                    simulationVehicle.getWheelDistFrontBack());

            // Initialize vectors and matrices
            localPosition = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
            velocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
            acceleration = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
            force = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
            localInertiaInverse = MatrixUtils.createRealIdentityMatrix(3);
            inertiaInverse = MatrixUtils.createRealIdentityMatrix(3);
            Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0);
            rotation = new BlockRealMatrix(rot.getMatrix());
            angularVelocity = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
            angularMomentum = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
            angularMomentumDeriv = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});

            // Store old center of mass position
            RealVector oldCenterOfMassPosition = localPosition.copy();
            // Calculate and set correct center of mass position
            initLocalPosition();
            // Calculate change of center of mass position
            RealVector centerOfMassShift = oldCenterOfMassPosition.subtract(localPosition);
            // Shift local coordinate system to put center of mass position at the origin
            localPosition = localPosition.add(centerOfMassShift);
            geometryPositionOffset = geometryPositionOffset.add(centerOfMassShift);
            RealVector leftFrontWheelLocalPosition = wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getLocalPos();
            RealVector rightFrontWheelLocalPosition = wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getLocalPos();
            RealVector leftBackWheelLocalPosition = wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getLocalPos();
            RealVector rightBackWheelLocalPosition = wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getLocalPos();
            leftFrontWheelLocalPosition = leftFrontWheelLocalPosition.add(centerOfMassShift);
            rightFrontWheelLocalPosition = rightFrontWheelLocalPosition.add(centerOfMassShift);
            leftBackWheelLocalPosition = leftBackWheelLocalPosition.add(centerOfMassShift);
            rightBackWheelLocalPosition = rightBackWheelLocalPosition.add(centerOfMassShift);
            wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].setLocalPos(leftFrontWheelLocalPosition);
            wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].setLocalPos(rightFrontWheelLocalPosition);
            wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].setLocalPos(leftBackWheelLocalPosition);
            wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].setLocalPos(rightBackWheelLocalPosition);
            position = position.add(rotation.operate(centerOfMassShift));

            // Initialize remaining values
            initMassPointLocalCenterDiff();
            initLocalInertiaInverse();
            calcInertiaInverse();
            calcAngularVelocity();

            // Initialize remaining mass point values
            calcMassPointCenterDiff();
            calcMassPointPosition();
            calcMassPointVelocity();

            physicalVehicleInitialized = true;
            simulationVehicle.setVehicleInitialized(true);
        }
    }

    /**
     * Function that creates the mass points with local position and mass
     * Should only be called by initMassPointPhysics
     *
     * @param massFront          Sum of mass for both front wheels
     * @param massBack           Sum of mass for both back wheels
     * @param wheelDistLeftRight Distance between left and right wheels
     * @param wheelDistFrontBack Distance between front and back wheels
     */
    private void createMassPoints(double massFront, double massBack,  double wheelDistLeftRight, double wheelDistFrontBack) {
        RealVector localPositionFrontLeft = new ArrayRealVector(new double[]{-(wheelDistLeftRight / 2), (wheelDistFrontBack / 2), 0.0});
        RealVector localPositionFrontRight = new ArrayRealVector(new double[]{(wheelDistLeftRight / 2), (wheelDistFrontBack / 2), 0.0});
        RealVector localPositionBackLeft = new ArrayRealVector(new double[]{-(wheelDistLeftRight / 2), -(wheelDistFrontBack / 2), 0.0});
        RealVector localPositionBackRight = new ArrayRealVector(new double[]{(wheelDistLeftRight / 2), -(wheelDistFrontBack / 2), 0.0});
        RealVector zeroVector = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        //RealVector gravitationVectorFrontWheel = new ArrayRealVector(new double[]{0.0, 0.0, (massFront / 2) * -9.81});
        //RealVector gravitationVectorBackWheel = new ArrayRealVector(new double[]{0.0, 0.0, (massBack / 2) * -9.81});

        wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_FRONT_LEFT, localPositionFrontLeft, zeroVector, zeroVector, zeroVector, zeroVector, zeroVector, (massFront / 2));
        wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_FRONT_RIGHT, localPositionFrontRight, zeroVector, zeroVector, zeroVector, zeroVector, zeroVector, (massFront / 2));
        wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_BACK_LEFT, localPositionBackLeft, zeroVector, zeroVector, zeroVector, zeroVector, zeroVector, (massBack / 2));
        wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()] = new MassPoint(MASS_POINT_TYPE_WHEEL_BACK_RIGHT, localPositionBackRight, zeroVector, zeroVector, zeroVector, zeroVector, zeroVector, (massBack / 2));

        wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
        wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
        wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
        wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].setPressure(VEHICLE_DEFAULT_TIRE_PRESSURE);
    }

    /**
     * Function that computes the center of mass position in the local coordinate system
     * Based on mass and local positions of the mass point
     * Should only be called by initMassPointPhysics
     */
    private void initLocalPosition() {
        Log.finest("PhysicalVehicle: initLocalPos - PhysicalVehicle at start: " + this);
        RealVector result = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        for (MassPoint mp : wheelMassPoints) {
            result = result.add(mp.getLocalPos().mapMultiplyToSelf(mp.getMass()));
        }

        localPosition = result.mapDivideToSelf(getMass());
        Log.finest("PhysicalVehicle: initLocalPos - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that computes the localCenterDiff values for all mass points
     * Based on local position of the mass point and local position of the physical vehicle
     * Should only be called by initMassPointPhysics
     */
    private void initMassPointLocalCenterDiff() {
        Log.finest("PhysicalVehicle: initMassPointLocalCenterDiff - PhysicalVehicle at start: " + this);

        for (MassPoint mp : wheelMassPoints) {
            mp.setLocalCenterDiff(mp.getLocalPos().subtract(localPosition));
        }

        Log.finest("PhysicalVehicle: initMassPointLocalCenterDiff - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that computes the localInertiaInverse of the physical vehicle
     * Based on mass and localCenterDiff of all mass points
     * Should only be called by initMassPointPhysics
     */
    private void initLocalInertiaInverse(){
        Log.finest("PhysicalVehicle: initLocalInertiaInverse - PhysicalVehicle at start: " + this);
        try {
            // Matrix of dimension 3x3 with zero values
            RealMatrix result = MatrixUtils.createRealMatrix(3, 3);

            // Add up values from mass points
            for (MassPoint mp : wheelMassPoints) {
                RealMatrix matrixMassPoint = MathHelper.vector3DToCrossProductMatrix(mp.getLocalCenterDiff()).power(2).scalarMultiply(-mp.getMass());
                result = result.add(matrixMassPoint);
            }

            // Compute inverse
            localInertiaInverse = MathHelper.matrixInvert(result);

        } catch (Exception e) {
            Log.severe("PhysicalVehicle: initLocalInertiaInverse - Could not calculate local inertia inverse. Cross product matrix or matrix inversion failed.");
            e.printStackTrace();
        }
        Log.finest("PhysicalVehicle: initLocalInertiaInverse - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the angularMomentumDeriv for the physicalVehicle
     * Based on current forces and center differences of vehicles mass points
     */
    private void calcAngularMomentumDeriv(){
        Log.finest("PhysicalVehicle: calcAngularMomentumDeriv - PhysicalVehicle at start: " + this);
        RealVector result = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        for (MassPoint mp : wheelMassPoints) {
            try {
                result = result.add(MathHelper.vector3DCrossProduct(mp.getCenterDiff(), mp.getForce()));
            }
            catch (Exception e) {
                Log.severe("PhysicalVehicle: calcAngularMomentumDeriv - Exception:" + e.toString());
                e.printStackTrace();
            }
        }

        angularMomentumDeriv = result;
        Log.finest("PhysicalVehicle: calcAngularMomentumDeriv - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the force for the vehicle
     * Based on current forces of vehicles mass points
     */
    private void calcForce(){
        Log.finest("PhysicalVehicle: calcForce - PhysicalVehicle at start: " + this);
        RealVector result = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

        for (MassPoint mp : wheelMassPoints) {
            result = result.add(mp.getForce());
        }

        force = force.add(result);
        Double forceNorm = force.getNorm();

        // Set computational error if forces are way too high to keep vehicle in stable state
        if (forceNorm.isInfinite() || forceNorm.isNaN() || forceNorm > 1.0E10) {
            setError(true);
        }

        Log.finest("PhysicalVehicle: calcForce - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the position vector for the physicalVehicle
     * Based on current physicalVehicles velocity and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcPosition(double deltaT){
        Log.finest("PhysicalVehicle: calcPosition - Input time: " + deltaT + ", PhysicalVehicle at start: " + this);
        position = position.add(velocity.mapMultiply(deltaT));
        Log.finest("PhysicalVehicle: calcPosition - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the velocity and acceleration vector for the physicalVehicle
     * Based on current physicalVehicles force and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcVelocityAndAcceleration(double deltaT){
        Log.finest("PhysicalVehicle: calcVelocityAndAcceleration - Input time: " + deltaT + ", PhysicalVehicle at start: " + this);

        acceleration = force.mapDivide(getMass());

        RealVector zeroVector = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        double threshold = 0.0000000000001;
        if(MathHelper.vectorEquals(acceleration, zeroVector, threshold)){
            acceleration = zeroVector;
        }

        velocity = velocity.add(acceleration.mapMultiply(deltaT));

        if(MathHelper.vectorEquals(velocity, zeroVector, threshold)){
            velocity = zeroVector;
        }

        Log.finest("PhysicalVehicle: calcVelocityAndAcceleration - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the rotationMatrix for the physicalVehicle
     * Based on current physicalVehicles angularVelocity and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcRotationMatrix(double deltaT){
        Log.finest("PhysicalVehicle: calcRotationMatrix - Input time: " + deltaT + ", PhysicalVehicle at start: " + this);

        try {
            rotation = rotation.add((MathHelper.vector3DToCrossProductMatrix(angularVelocity).multiply(rotation)).scalarMultiply(deltaT));
        } catch (Exception e) {
            Log.severe("PhysicalVehicle: calcRotationMatrix - Exception:" + e.toString());
            e.printStackTrace();
        }

        // Always orthonormalize matrix after computations to avoid numerical issues
        rotation = MathHelper.matrix3DOrthonormalize(rotation);

        Log.finest("PhysicalVehicle: calcRotationMatrix - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the angularMomentum for the physicalVehicle
     * Based on current physicalVehicles angularMomentumDeriv and time input
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcAngularMomentum(double deltaT){
        Log.finest("PhysicalVehicle: calcAngularMomentum - Input time: " + deltaT + ", PhysicalVehicle at start: " + this);
        angularMomentum = angularMomentum.add(angularMomentumDeriv.mapMultiply(deltaT));
        Log.finest("PhysicalVehicle: calcAngularMomentum - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the inertiaInverse matrix for physicalVehicle
     * Based on current physicalVehicles localInertiaInverse and rotationMatrix
     */
    private void calcInertiaInverse(){
        Log.finest("PhysicalVehicle: calcInertiaInverse - PhysicalVehicle at start: " + this);
        inertiaInverse = rotation.multiply(localInertiaInverse).multiply(rotation.transpose());
        Log.finest("PhysicalVehicle: calcInertiaInverse - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the angularVelocity vector for the physicalVehicle
     * Based on current physicalVehicles inertiaInverse and angularMomentum
     */
    private void calcAngularVelocity(){
        Log.finest("PhysicalVehicle: calcAngularVelocity - PhysicalVehicle at start: " + this);
        angularVelocity = inertiaInverse.operate(angularMomentum);
        Log.finest("PhysicalVehicle: calcAngularVelocity - PhysicalVehicle at end: " + this);
    }

    /**
     * Recalculates the r_i after one integration step
     */
    private void calcMassPointCenterDiff(){
        Log.finest("PhysicalVehicle: calcMassPointCenterDiff - PhysicalVehicle at start: " + this);
        for(MassPoint massPoint : wheelMassPoints){
            massPoint.setCenterDiff(this.rotation.operate(massPoint.getLocalCenterDiff()));
        }
        Log.finest("PhysicalVehicle: calcMassPointCenterDiff - PhysicalVehicle at end: " + this);
    }

    /**
     * Calculates the positions of the mass points
     */
    private void calcMassPointPosition(){
        Log.finest("PhysicalVehicle: calcMassPointPosition - PhysicalVehicle at start: " + this);
        for(MassPoint massPoint : wheelMassPoints){
            massPoint.setPos(this.position.add(massPoint.getCenterDiff()));

            RealVector massPointPosition = massPoint.getPos();
            double groundZ = WorldModel.getInstance().getGround(massPointPosition.getEntry(0), massPointPosition.getEntry(1), massPointPosition.getEntry(2)).doubleValue();
            massPoint.setGroundZ(groundZ);
            double limitZ = groundZ + simulationVehicle.getWheelRadius();

            // If mass point position goes way below ground position + wheel radius, then set computational error
            if (massPointPosition.getEntry(2) < (limitZ - 0.5 * simulationVehicle.getWheelRadius())) {
                // setError(true);
            }
        }
        Log.finest("PhysicalVehicle: calcMassPointPosition - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the velocity vector for all mass points
     */
    private void calcMassPointVelocity(){
        Log.finest("PhysicalVehicle: calcMassPointVelocity - PhysicalVehicle at start: " + this);
        for(MassPoint massPoint : wheelMassPoints){
            try {
                massPoint.setVelocity(this.velocity.add(MathHelper.vector3DCrossProduct(this.angularVelocity, massPoint.getCenterDiff())));
            } catch (Exception e) {
                e.printStackTrace();
            }

            RealVector zeroVector = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            double threshold = 0.0000000000001;
            if (MathHelper.vectorEquals(massPoint.getVelocity(), zeroVector, threshold)) {
                massPoint.setVelocity(zeroVector);
            }
        }
        Log.finest("PhysicalVehicle: calcMassPointVelocity - PhysicalVehicle at end: " + this);
    }

    /**
     * Function that calculates the forces for each mass point of the vehicle
     *
     * @param deltaT Time difference to previous step in seconds
     */
    private void calcMassPointForces(double deltaT){
        Log.finest("PhysicsEngine: calcMassPointForces - PhysicalVehicle at start: " + this);

        // Iterate over wheel mass points
        for (MassPoint mp : wheelMassPoints) {

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
            mp.setForce(mp.getForce().add(forceResult));
        }

        Log.finest("PhysicsEngine: calcMassPointForces - PhysicalVehicle at end: " + this);
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
        for (MassPoint mpTmp : wheelMassPoints) {
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
            RealVector backFront1 = (wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPos().subtract(wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPos()));
            RealVector backFront2 = (wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPos().subtract(wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getPos()));
            RealVector leftRight1 = (wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPos().subtract(wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPos()));
            RealVector leftRight2 = (wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_RIGHT.ordinal()].getPos().subtract(wheelMassPoints[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPos()));

            // Compute average vectors
            RealVector vectorBackFront = backFront1.add(backFront2).mapMultiply(0.5);
            RealVector vectorLeftRight = leftRight1.add(leftRight2).mapMultiply(0.5);

            // Force: Normal force Fn = mass * gravity constant (with correct angles)
            RealVector forceNormal = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});

            try {
                forceNormal = MathHelper.vector3DCrossProduct(vectorBackFront, vectorLeftRight);
            } catch (Exception e) {
                e.printStackTrace();
            }

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
            double rollingCoefficient = 0.005 + (1 / pressure) * (0.01 + 0.0095 * (forceRoadFrictionBackFrontNorm * 3.6 / 100) * (forceRoadFrictionBackFrontNorm * 3.6 / 100));

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

            double forceRoadFrictionLeftRightAmount = ((WorldModel.getInstance().isItRaining()) ? PhysicsEngine.ROAD_FRICTION_WET : PhysicsEngine.ROAD_FRICTION_DRY) * forceNormalLengthLeftRight;

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
            double wheelDistFrontBack = getSimulationVehicle().getWheelDistFrontBack();
            double curveRadiusVectorLength = (wheelDistFrontBack / curveRadiusSin);
            RealVector curveRadiusVector = (wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPos().subtract(wheelMassPoints[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPos()));

            if (curveRadiusVector.getNorm() > 0.0) {
                curveRadiusVector = curveRadiusVector.mapDivide(curveRadiusVector.getNorm());
            }

            curveRadiusVector = curveRadiusVector.mapMultiply(curveRadiusVectorLength);

            try {
                forceCentripetal = MathHelper.vector3DCrossProduct(angularVelocity, MathHelper.vector3DCrossProduct(angularVelocity, curveRadiusVector)).mapMultiply(mp.getMass());
            }
            catch (Exception e) {
                e.printStackTrace();
            }
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

        RealVector velocity = mp.getVelocity();

        if (velocity.getNorm() == 0.0) {
            return forceAirFriction;
        }

        double areaX = 0.25 * getSimulationVehicle().getHeight() * getSimulationVehicle().getLength();
        double areaY = 0.25 * getSimulationVehicle().getHeight() * getSimulationVehicle().getWidth();
        double areaZ = 0.25 * getSimulationVehicle().getLength() * getSimulationVehicle().getWidth();

        // For a rotation matrix it holds: inverse(matrix) = transpose(matrix)
        // Rotate velocityOrientation back to global coordinate system axis to match up with area values
        RealVector velocityOrigin = rotation.transpose().operate(velocity);

        // Fractions of area values for each axis according to velocity vector orientation and with no more vehicle rotation
        areaX = areaX * (velocityOrigin.getEntry(0) / velocityOrigin.getL1Norm());
        areaY = areaY * (velocityOrigin.getEntry(1) / velocityOrigin.getL1Norm());
        areaZ = areaZ * (velocityOrigin.getEntry(2) / velocityOrigin.getL1Norm());

        // Sum of all fractions for area values yields correct approximation of total area for air resistance
        double area = Math.abs(areaX) + Math.abs(areaY) + Math.abs(areaZ);

        // Scalar for air friction force computations
        double scalarCoefficient = -0.5 * PhysicsEngine.AIR_DENSITY * PhysicsEngine.AIR_DRAG_CAR * area;

        // Final force computation, preserve direction that we need for computations in the 3D space
        RealVector direction = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
        direction.setEntry(0, (velocity.getEntry(0) < 0.0 ? -1.0 : 1.0));
        direction.setEntry(1, (velocity.getEntry(1) < 0.0 ? -1.0 : 1.0));
        direction.setEntry(2, (velocity.getEntry(2) < 0.0 ? -1.0 : 1.0));
        forceAirFriction = velocity.ebeMultiply(velocity).ebeMultiply(direction).mapMultiply(scalarCoefficient);
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
        double groundDistance = (mp.getPos().getEntry(2) - limitZ);

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
     * Function that returns the four wheel mass points of the vehicle
     *
     * @return Four wheel mass points of the vehicle
     */
    public MassPoint[] getWheelMassPoints() {
        return wheelMassPoints;
    }

    /**
     * Function that returns a vector with the x, y and z coordinates of the object
     * This refers to the center position of the geometry object (i.e. NOT mass point position)
     *
     * @return Vector with x, y, z coordinates of the object center

    @Override
    public RealVector getGeometryPos() {
        RealVector relVectorBottomTop = new ArrayRealVector(new double[] {0.0, 0.0, getHeight()});
        relVectorBottomTop = getGeometryRot().operate(relVectorBottomTop);
        relVectorBottomTop = relVectorBottomTop.mapMultiply(0.5);

        RealVector relVectorBackFront = (simulationVehicle.getWheelMassPoints()[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPos().subtract(simulationVehicle.getWheelMassPoints()[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPos()));
        RealVector relVectorLeftRight = (simulationVehicle.getWheelMassPoints()[MASS_POINT_TYPE_WHEEL_FRONT_RIGHT.ordinal()].getPos().subtract(simulationVehicle.getWheelMassPoints()[MASS_POINT_TYPE_WHEEL_FRONT_LEFT.ordinal()].getPos()));
        relVectorBackFront = relVectorBackFront.mapMultiply(0.5);
        relVectorLeftRight = relVectorLeftRight.mapMultiply(0.5);

        RealVector absGeometryCenterPos = simulationVehicle.getWheelMassPoints()[MASS_POINT_TYPE_WHEEL_BACK_LEFT.ordinal()].getPos();
        absGeometryCenterPos = absGeometryCenterPos.add(relVectorBackFront);
        absGeometryCenterPos = absGeometryCenterPos.add(relVectorLeftRight);
        absGeometryCenterPos = absGeometryCenterPos.add(relVectorBottomTop);
        return absGeometryCenterPos;
    }
    */

    /**
     * Overwrite toString() to get a nice output for physicalVehicles
     * @return String that contains all information of physicalVehicles
     */
    @Override
    public String toString() {
        return  "PhysicalVehicle " + getId() +
                (physicalVehicleInitialized ? " , geometryPos: " + getGeometryPosition() : "") +
                (physicalVehicleInitialized ? " , localPosition: " + localPosition : "") +
                " , position: " + position +
                (physicalVehicleInitialized ? " , velocity: " + velocity : "") +
                (physicalVehicleInitialized ? " , acceleration: " + acceleration : "") +
                (physicalVehicleInitialized ? " , force: " + force : "") +
                (physicalVehicleInitialized ? " , localInertiaInverse: " + localInertiaInverse : "") +
                (physicalVehicleInitialized ? " , inertiaInverse: " + inertiaInverse : "") +
                (physicalVehicleInitialized ? " , rotation: " + rotation : "") +
                (physicalVehicleInitialized ? " , angularVelocity: " + angularVelocity : "") +
                (physicalVehicleInitialized ? " , angularMomentum: " + angularMomentum : "") +
                (physicalVehicleInitialized ? " , angularMomentumDeriv: " + angularMomentumDeriv : "") +
                (physicalVehicleInitialized ? " , physicalObjectType: " + physicalObjectType : "") +
                " , collision: " + collision +
                " , error: " + error +
                " , physicalVehicleInitialized: " + physicalVehicleInitialized +
                (physicalVehicleInitialized ? " , wheelMassPoints[0]: " + wheelMassPoints[0] : "") +
                (physicalVehicleInitialized ? " , wheelMassPoints[1]: " + wheelMassPoints[1] : "") +
                (physicalVehicleInitialized ? " , wheelMassPoints[2]: " + wheelMassPoints[2] : "") +
                (physicalVehicleInitialized ? " , wheelMassPoints[3]: " + wheelMassPoints[3] : "") +
                " , simulationVehicle: " + simulationVehicle;
    }
    public RealVector getForce(){
        return force.copy();
    }
    public RealVector getAngularVelocity(){
        return angularVelocity.copy();
    }
}
