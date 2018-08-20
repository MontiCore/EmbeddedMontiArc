package simulation.vehicle;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.*;
import simulation.environment.WorldModel;
import simulation.util.Log;

import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

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


    /** Attributes used for medelica physics */
    /** Default step size in milliseconds */
    private int stepSizems = 2;

    /** Current rotation around the local z axis */
    private double yaw_angle;


    /** Components used for modelica physics */
    /** InputFilter of the vehicle */
    private InputFilter inputFilter;

    /** Chassis of the vehicle */
    private Chassis chassis;

    /** Suspension of the vehicle */
    private Suspension suspension;

    /** Tires of the vehicle */
    private Tires tires;


    /** Coordinate system rotation */
    public static final RealMatrix coordinateRotation = new BlockRealMatrix(new Rotation(
            RotationOrder.XYZ,
            RotationConvention.VECTOR_OPERATOR,
            0.0, 0.0, Math.PI / 2).getMatrix());

    /**
     * Constructor for an uninitialized physical vehicle
     */
    public ModelicaPhysicalVehicle(){
        super();
        inputFilter = new InputFilter();
        chassis = new Chassis();
        suspension = new Suspension();
        tires = new Tires();

        //Before initialisation the center of mass position is the same as the center of geometry position and at the origin
        this.position = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
        this.geometryPositionOffset = new ArrayRealVector(new double[]{0.0, 0.0, 0.0});

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
        }
    }

    /**
     * Function that returns a copy of the velocity vector of the center of mass
     * @return Velocity vector of the center of mass
     */
    @Override
    public RealVector getVelocity(){
        //return this.velocity.copy();
        return new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
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
        //return this.acceleration.copy();
        return new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
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
            this.force = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            //todo set FDU values to zero
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
     * @param deltaTms Duration of the current simulation step in milliseconds
     */
    @Override
    public void computePhysics(long deltaTms){
        if (!this.getError()) {
            // Calculate input values
            // Get values from FDUs
            double z = chassis.getSimulator().read("z").asDouble();
            double r_nom = chassis.getSimulator().read("r_nom").asDouble();
            double m = chassis.getSimulator().read("m").asDouble();
            // Get current road surface position
            RealVector roadPlane = position.add(rotation.operate(new ArrayRealVector(new double[]{0.0, 0.0, -z})));
            // Reset vehicle on surface
            putOnSurface(roadPlane.getEntry(0), roadPlane.getEntry(1), yaw_angle);
            // Get motor acceleration and convert it in torque
            double motorAcceleration = 4 * simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_MOTOR).getActuatorValueCurrent();
            double motorForce = m * motorAcceleration;
            double motorTorque = r_nom * motorForce;
            System.out.println(motorTorque);
            // Assume equal 4 wheel drive
            chassis.getSimulator().write("tau_D_1").with(motorTorque / 4);
            chassis.getSimulator().write("tau_D_2").with(motorTorque / 4);
            chassis.getSimulator().write("tau_D_3").with(motorTorque / 4);
            chassis.getSimulator().write("tau_D_4").with(motorTorque / 4);
            // Get brake acceleration and convert it in torque
            double brakeAcceleration1 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).getActuatorValueCurrent();
            double brakeAcceleration2 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).getActuatorValueCurrent();
            double brakeAcceleration3 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).getActuatorValueCurrent();
            double brakeAcceleration4 = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).getActuatorValueCurrent();
            double brakeForce1 = m * brakeAcceleration1;
            double brakeForce2 = m * brakeAcceleration2;
            double brakeForce3 = m * brakeAcceleration3;
            double brakeForce4 = m * brakeAcceleration4;
            double brakeTorque1 = r_nom * brakeForce1;
            double brakeTorque2 = r_nom * brakeForce2;
            double brakeTorque3 = r_nom * brakeForce3;
            double brakeTorque4 = r_nom * brakeForce4;
            chassis.getSimulator().write("tau_B_1").with(brakeTorque1);
            chassis.getSimulator().write("tau_B_2").with(brakeTorque2);
            chassis.getSimulator().write("tau_B_3").with(brakeTorque3);
            chassis.getSimulator().write("tau_B_4").with(brakeTorque4);
            // Get steering angle
            double steeringAngle = simulationVehicle.getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();
            chassis.getSimulator().write("delta_1").with(steeringAngle);
            chassis.getSimulator().write("delta_2").with(steeringAngle);
            tires.getSimulator().write("delta_1").with(steeringAngle);
            tires.getSimulator().write("delta_2").with(steeringAngle);
            // Express the force vector in local coordinates
            RealVector localForce = rotation.transpose().operate(force);
            chassis.getSimulator().write("F_ext_x").with(localForce.getEntry(0));
            chassis.getSimulator().write("F_ext_y").with(localForce.getEntry(1));
            // todo Take the wheel positions and get the frictions coefficients

            // Exchange values
            exchangeValues();
            long currentDeltaTms = 0;
            // Do steps with maximum step size as long as possible
            while(currentDeltaTms + stepSizems <= deltaTms){
                if(inputFilter.isTerminated() || chassis.isTerminated() || suspension.isTerminated() || tires.isTerminated()){
                    //todo Set up new FMUs and transfer values
                }
                doCalculationStep(stepSizems);
                currentDeltaTms = currentDeltaTms + stepSizems;
            }
            // Do a step with partial step size to fill the gap
            long partialStepSize = deltaTms - currentDeltaTms;
            if(partialStepSize > 0){
                if(inputFilter.isTerminated() || chassis.isTerminated() || suspension.isTerminated() || tires.isTerminated()){
                    //todo Set up new FMUs and transfer values
                }
                doCalculationStep(partialStepSize);
                currentDeltaTms = currentDeltaTms + partialStepSize;
            }
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
        //Get values from FDUs
        double z = chassis.getSimulator().read("z").asDouble();
        double L_1 = chassis.getSimulator().read("L_1").asDouble();
        double L_2 = chassis.getSimulator().read("L_2").asDouble();
        double TW_f = chassis.getSimulator().read("TW_r").asDouble();
        double TW_r = chassis.getSimulator().read("TW_r").asDouble();

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

        Vector3D backToFrontLocal = new Vector3D(L_1 + L_2, 0.0, 0.0);

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
        Vector3D backToFront = realTo3D(frontPosition.subtract(backPosition));
        Vector3D rightToLeft = realTo3D(leftPosition.subtract(rightPosition));
        Vector3D roadPlaneNorm = Vector3D.crossProduct(backToFront, rightToLeft);

        //Compute angles between relative vectors and X-Y-Plane
        Vector3D XYPlaneNorm = new Vector3D(0.0, 0.0, 1.0);
        double backToFrontAngle = (Math.PI / 2) - Vector3D.angle(XYPlaneNorm, backToFront);
        double rightToLeftAngle = (Math.PI / 2) - Vector3D.angle(XYPlaneNorm, rightToLeft);

        if(physicalVehicleInitialized) {
            inputFilter.getSimulator().write("slope").with(backToFrontAngle);
            inputFilter.getSimulator().write("bank").with(rightToLeftAngle);
        }

        //The resulting rotation should transform the XY plane norm to the roadPlaneNorm
        //and the backToFrontLocal to the BackToFront

        Rotation finalRot = new Rotation(XYPlaneNorm, backToFrontLocal, roadPlaneNorm, backToFront);
        rotation = new BlockRealMatrix(finalRot.getMatrix());
        yaw_angle = rotZ;

        //The rotation is occurring around the center of the road plane, so the position has to be shifted
        RealVector roadPlaneCenter = position.add(new ArrayRealVector(new double[]{0.0, 0.0, -z}));
        RealVector roadPlaneCenterToPositionLocal = new ArrayRealVector(new double[]{0.0, 0.0, z});
        position = roadPlaneCenter.add(rotation.operate(roadPlaneCenterToPositionLocal));
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
        //Get values from FDUs
        double L_1 = chassis.getSimulator().read(("L_1")).asDouble();
        double TW_f = chassis.getSimulator().read("TW_f").asDouble();
        double z = chassis.getSimulator().read("z").asDouble();
        double r_nom = chassis.getSimulator().read("r_nom").asDouble();
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
        //Get values from FDUs
        double L_1 = chassis.getSimulator().read(("L_1")).asDouble();
        double TW_f = chassis.getSimulator().read("TW_f").asDouble();
        double z = chassis.getSimulator().read("z").asDouble();
        double r_nom = chassis.getSimulator().read("r_nom").asDouble();
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
        //Get values from FDUs
        double L_2 = chassis.getSimulator().read(("L_2")).asDouble();
        double TW_r = chassis.getSimulator().read("TW_r").asDouble();
        double z = chassis.getSimulator().read("z").asDouble();
        double r_nom = chassis.getSimulator().read("r_nom").asDouble();
        //Calculate localPosition and return global position
        RealVector localPosition = new ArrayRealVector(new double[]{L_2, -TW_r / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that returns a copy of the position vector of the center of the back left wheel
     * @return Position vector of the center of the back left wheel
     */
    @Override
    public RealVector getBackLeftWheelGeometryPosition(){
        //Get values from FDUs
        double L_2 = chassis.getSimulator().read(("L_2")).asDouble();
        double TW_r = chassis.getSimulator().read("TW_r").asDouble();
        double z = chassis.getSimulator().read("z").asDouble();
        double r_nom = chassis.getSimulator().read("r_nom").asDouble();
        //Calculate localPosition and return global position
        RealVector localPosition = new ArrayRealVector(new double[]{L_2, TW_r / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
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
            // Initialize the modelica components
            inputFilter.initialize(0.0, 10.0);
            chassis.initialize(0.0, 10.0);
            suspension.initialize(0.0, 10.0);
            tires.initialize(0.0, 10.0);

            //The input values for a stationary car with no rotation and external influences are already encoded in the FMUs

            //Exchange values
            exchangeValues();

            //Shift position and geometryPositionOffset
            double z = chassis.getSimulator().read("z").asDouble();
            geometryPositionOffset = new ArrayRealVector(new double[]{0.0, 0.0, simulationVehicle.getHeight() / 2 - z});
            position = geometryPositionOffset.mapMultiply(-1.0);

            //Initialize remaining variables
            force = new ArrayRealVector(new double[] {0.0, 0.0, 0.0});
            rotation = coordinateRotation.copy();
            yaw_angle = 0.0;

            //Overwrite parameters in vehicle with the FDU values
            simulationVehicle.setMass(chassis.getSimulator().read("m").asDouble());
            simulationVehicle.setWheelDistLeftRightFrontSide(chassis.getSimulator().read("TW_f").asDouble());
            simulationVehicle.setWheelDistLeftRightBackSide(chassis.getSimulator().read("TW_r").asDouble());
            simulationVehicle.setWheelDistFrontBack(
                    chassis.getSimulator().read("L_1").asDouble()+
                    chassis.getSimulator().read("L_2").asDouble());
            simulationVehicle.setWheelRadius(chassis.getSimulator().read("r_nom").asDouble());

            physicalVehicleInitialized = true;
            simulationVehicle.setVehicleInitialized(true);
        }
    }

    /**
     * Function that exchanges values of the modelica components
     */
    private void exchangeValues(){
        chassis.getSimulator().write("slope_d").with(inputFilter.getSimulator().read("slope_d").asDouble());
        chassis.getSimulator().write("bank_d").with(inputFilter.getSimulator().read("bank_d").asDouble());
        suspension.getSimulator().write("slope_d").with(inputFilter.getSimulator().read("slope_d").asDouble());
        suspension.getSimulator().write("bank_d").with(inputFilter.getSimulator().read("bank_d").asDouble());
        suspension.getSimulator().write("a_y").with(chassis.getSimulator().read("a_y").asDouble());
        suspension.getSimulator().write("pitch_angle").with(chassis.getSimulator().read("pitch_angle").asDouble());
        suspension.getSimulator().write("omega_y").with(chassis.getSimulator().read("omega_y").asDouble());
        tires.getSimulator().write("v_s_1").with(chassis.getSimulator().read("v_s_1").asDouble());
        tires.getSimulator().write("v_s_2").with(chassis.getSimulator().read("v_s_2").asDouble());
        tires.getSimulator().write("v_s_3").with(chassis.getSimulator().read("v_s_3").asDouble());
        tires.getSimulator().write("v_s_4").with(chassis.getSimulator().read("v_s_4").asDouble());
        tires.getSimulator().write("v_x_1").with(chassis.getSimulator().read("v_x_1").asDouble());
        tires.getSimulator().write("v_x_2").with(chassis.getSimulator().read("v_x_2").asDouble());
        tires.getSimulator().write("v_x_3").with(chassis.getSimulator().read("v_x_3").asDouble());
        tires.getSimulator().write("v_x_4").with(chassis.getSimulator().read("v_x_4").asDouble());
        tires.getSimulator().write("v_y_1").with(chassis.getSimulator().read("v_y_1").asDouble());
        tires.getSimulator().write("v_y_2").with(chassis.getSimulator().read("v_y_2").asDouble());
        tires.getSimulator().write("v_y_3").with(chassis.getSimulator().read("v_y_3").asDouble());
        tires.getSimulator().write("v_y_4").with(chassis.getSimulator().read("v_y_4").asDouble());
        chassis.getSimulator().write("d_roll").with(suspension.getSimulator().read("d_roll").asDouble());
        chassis.getSimulator().write("d_pitch").with(suspension.getSimulator().read("d_pitch").asDouble());
        chassis.getSimulator().write("K_roll_f").with(suspension.getSimulator().read("K_roll_f").asDouble());
        chassis.getSimulator().write("K_roll_r").with(suspension.getSimulator().read("K_roll_r").asDouble());
        chassis.getSimulator().write("D_roll_f").with(suspension.getSimulator().read("D_roll_f").asDouble());
        chassis.getSimulator().write("D_roll_r").with(suspension.getSimulator().read("D_roll_r").asDouble());
        chassis.getSimulator().write("D_pitch").with(suspension.getSimulator().read("D_pitch").asDouble());
        chassis.getSimulator().write("K_pitch").with(suspension.getSimulator().read("K_pitch").asDouble());
        tires.getSimulator().write("F_z_1").with(suspension.getSimulator().read("F_z_1").asDouble());
        tires.getSimulator().write("F_z_2").with(suspension.getSimulator().read("F_z_2").asDouble());
        tires.getSimulator().write("F_z_3").with(suspension.getSimulator().read("F_z_3").asDouble());
        tires.getSimulator().write("F_z_4").with(suspension.getSimulator().read("F_z_4").asDouble());
        chassis.getSimulator().write("F_x_1").with(tires.getSimulator().read("F_x_1").asDouble());
        chassis.getSimulator().write("F_x_2").with(tires.getSimulator().read("F_x_2").asDouble());
        chassis.getSimulator().write("F_x_3").with(tires.getSimulator().read("F_x_3").asDouble());
        chassis.getSimulator().write("F_x_4").with(tires.getSimulator().read("F_x_4").asDouble());
        chassis.getSimulator().write("F_y_1").with(tires.getSimulator().read("F_y_1").asDouble());
        chassis.getSimulator().write("F_y_2").with(tires.getSimulator().read("F_y_2").asDouble());
        chassis.getSimulator().write("F_y_3").with(tires.getSimulator().read("F_y_3").asDouble());
        chassis.getSimulator().write("F_y_4").with(tires.getSimulator().read("F_y_4").asDouble());
    }

    /**
     * Function that does a calculation step
     * @param deltaTms Length of the calculation step in milliseconds
     */
    private void doCalculationStep(long deltaTms){
        // Store z coordinate for interpolation later
        double oldZ = chassis.getSimulator().read("z").asDouble();

        // Do a computation step
        double deltaT = deltaTms / 1000.0;
        inputFilter.doStep(deltaT);
        chassis.doStep(deltaT);
        suspension.doStep(deltaT);
        tires.doStep(deltaT);

        // Exchange values
        exchangeValues();

        //Integrate over model output
        double omega_z = chassis.getSimulator().read("omega_z").asDouble();
        yaw_angle = yaw_angle + omega_z * deltaT;
        double v_x = chassis.getSimulator().read("v_x").asDouble();
        double v_y = chassis.getSimulator().read("v_y").asDouble();
        double z = chassis.getSimulator().read("z").asDouble();
        double v_z = (z - oldZ)/deltaT;
        RealVector localVelocity = new ArrayRealVector(new double[]{v_x, v_y, v_z});
        RealVector velocity = rotation.operate(localVelocity);
        position = position.add(velocity.mapMultiply(deltaT));
        RealVector deltaZ = new ArrayRealVector(new double[]{0.0, 0.0, oldZ - z});
        geometryPositionOffset = geometryPositionOffset.add(deltaZ);
    }

    /**
     * Function that takes a RealVector with 3 components and gives a Vector3D
     * @param vector RealVector to be turned into a Vector3D
     * @return Vector3D according to the RealVector vector
     */
    private Vector3D realTo3D(RealVector vector){
        if(vector.getDimension() == 3) {
            return new Vector3D(vector.getEntry(0), vector.getEntry(1), vector.getEntry(2));
        }else{
            Log.warning("Input vector " + vector + " has not the right amount of components!");
            return new Vector3D(0.0, 0.0, 0.0);
        }
    }

    /**
     * Overwrite toString() to get a nice output for ModelicaPhysicalVehicles
     * @return String that contains all information of ModelicaPhysicalVehicles
     */
    @Override
    public String toString() {
        return  "PhysicalVehicle " + getId() +
                (physicalVehicleInitialized ? " , geometryPos: " + getGeometryPosition() : "") +
                (physicalVehicleInitialized ? " , position: " + position : "") +
                //(physicalVehicleInitialized ? " , velocity: " + velocity : "") +
                //(physicalVehicleInitialized ? " , acceleration: " + acceleration : "") +
                (physicalVehicleInitialized ? " , force: " + force : "") +
                (physicalVehicleInitialized ? " , rotation: " + rotation : "") +
                (physicalVehicleInitialized ? " , physicalObjectType: " + physicalObjectType : "") +
                " , collision: " + collision +
                " , error: " + error +
                " , physicalVehicleInitialized: " + physicalVehicleInitialized +
                " , simulationVehicle: " + simulationVehicle;
    }

    public RealVector getForce(){
        return force.copy();
    }
    public RealVector getAngularVelocity(){
        //return angularVelocity.copy();
        return new ArrayRealVector(new double[]{0.0, 0.0, 0.0});
    }
}
