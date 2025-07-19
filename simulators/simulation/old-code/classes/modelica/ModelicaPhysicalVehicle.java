/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle.modelica;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.*;
import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.util.Log;
import de.rwth.montisim.simulation.util.MathHelper;
import simulation.vehicle.PhysicalVehicle;
import simulation.vehicle.PhysicsEngine;
import simulation.vehicle.VehicleActuator;
import simulation.vehicle.VehicleActuatorType;

import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.lang.Math;
import java.time.Duration;

import static simulation.vehicle.VehicleActuatorType.*;
import static simulation.vehicle.VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING;

/**
 * Class that represents a simulation vehicle with physical properties and
 * interactions implemented according to the Vehicle Dynamics Model for Driving
 * Simulators
 */
public class ModelicaPhysicalVehicle extends PhysicalVehicle {

    /** Variables for the IPhysicalVehicle interface */

    /** Position vector of the center of mass */
    protected Vec3 position;

    /** Rotation matrix around the center of mass */
    protected RealMatrix rotation;

    /** Force vector acting on the center of mass */
    private Vec3 force;

    /**
     * Vector pointing from the center of mass position to the center of geometry
     * position in the local coordinate system
     */
    private Vec3 geometryPositionOffset;

    /** Current rotation around the local z axis */
    private double yawAngle;

    private double old_sw_angle = 0.0;

    /** VehicleDynamicsModel used for modelica physics */
    private VehicleDynamicsModel vehicleDynamicsModel;

    /** Coordinate system rotation */
    public static final RealMatrix coordinateRotation = new BlockRealMatrix(
            new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, Math.PI / 2).getMatrix());


    /** Actuators of vehicle */
    private VehicleActuator gear;
    private VehicleActuator clutch;
    private VehicleActuator brakes;
    private VehicleActuator throttle;

    /**
     * Constructor for an uninitialised physical vehicle
     */
    public ModelicaPhysicalVehicle() {
        super();

        // Before initialisation
        // the center of mass position is at the origin
        // no rotation
        // the center of geometry position is at the origin
        this.position = new Vec3(new double[]{0.0, 0.0, 0.0});
        this.rotation = coordinateRotation.copy();
        this.geometryPositionOffset = new Vec3(new double[]{0.0, 0.0, 0.0});
    }

    /**
     * Function that returns a copy of the center of mass position vector
     *
     * @return Position vector of the center of mass
     */
    @Override
    public Vec3 getPosition() {
        return position.copy();
    }

    /**
     * Function that sets the center of mass position vector
     *
     * @param position New position vector of the center of mass
     */
    @Override
    public void setPosition(Vec3 position) {
        if (!physicalVehicleInitialised) {
            throw new IllegalStateException("Position can only be set after initialisation.");
        }
        this.position = position.copy();
    }

    /**
     * Function that returns a copy of the rotation matrix around the center of mass
     *
     * @return Rotation matrix around the center of mass
     */
    @Override
    public RealMatrix getRotation() {
        return coordinateRotation.transpose().multiply(this.rotation);
    }

    /**
     * Function that sets the rotation matrix around the center of mass
     *
     * @param rotation New rotation matrix around the center of mass
     */
    @Override
    public void setRotation(RealMatrix rotation) {
        if (!physicalVehicleInitialised) {
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
     *
     * @return Velocity vector of the center of mass
     */
    @Override
    public Vec3 getVelocity() {
        // Get current velocity
        Vec3 localVelocity = new Vec3(new double[]{vehicleDynamicsModel.getValue("v_x"),
                vehicleDynamicsModel.getValue("v_y"), vehicleDynamicsModel.getValue("v_z")});
        // Return in global coordinates
        return rotation.operate(localVelocity);
    }

    /**
     * Function that sets the velocity vector of the center of mass
     *
     * @param velocity New velocity vector of the center of mass
     */
    @Override
    public void setVelocity(Vec3 velocity) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Velocity can only be set before initialisation.");
        }
        throw new UnsupportedOperationException("Setting the velocity before initialisation is done by the builder.");
    }

    /**
     * Function that returns a copy of the angular velocity vector around the center
     * of mass
     *
     * @return Angular velocity vector around the center of mass
     */
    @Override
    public Vec3 getAngularVelocity() {
        // Get current angular velocity
        Vec3 localAngularVelocity = new Vec3(
                new double[]{0.0, 0.0, vehicleDynamicsModel.getValue("omega_z")});
        // Return in global coordinates
        return rotation.operate(localAngularVelocity);
    }

    /**
     * Function that sets the angular velocity vector around the center of mass
     *
     * @param angularVelocity New angular velocity around of the center of mass
     */
    @Override
    public void setAngularVelocity(Vec3 angularVelocity) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Angular velocity can only be set before initialisation.");
        }
        getVDM().setParameter("omega_z_0", angularVelocity.getEntry(2));
    }

    /**
     * Function that adds an external force acting on the center of mass
     *
     * @param force Force vector that acts on the center of mass
     */
    @Override
    public void addForce(Vec3 force) {
        this.force = this.force.add(force);
    }

    /**
     * Function that add an external torque acting around the center of mass
     *
     * @param torque Torque vector that acts around the center of mass
     */
    public void addTorque(Vec3 torque) {
        // TODO: Expand the model to accept external torques
        throw new UnsupportedOperationException("External torques are currently not supported.");
    }

    /**
     * Function that returns the mass of the vehicle
     *
     * @return Mass of the vehicle
     */
    @Override
    public double getMass() {
        if (!physicalVehicleInitialised) {
            throw new IllegalStateException("Mass can only be read after initialisation.");
        }
        return this.getVDM().getValue("m");
    }

    /**
     * Function that sets the mass of the vehicle
     *
     * @param mass New mass of the vehicle
     */
    @Override
    public void setMass(double mass) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Mass can only be set before initialisation.");
        }
        this.getVDM().setParameter("m", mass);
    }

    /**
     * Function that returns the wheel radius of the vehicle
     *
     * @return Wheel radius of the vehicle
     */
    @Override
    public double getWheelRadius() {
        if (!physicalVehicleInitialised) {
            throw new IllegalStateException("Wheel radius can only be read after initialisation.");
        }
        return this.getVDM().getValue("r_nom");
    }

    /**
     * Function that sets the wheel radius of the vehicle
     *
     * @param wheelRadius New wheel radius of the vehicle
     */
    @Override
    public void setWheelRadius(double wheelRadius) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Wheel radius can only be set before initialisation.");
        }
        this.getVDM().setParameter("r_nom", wheelRadius);
    }

    /**
     * Function that returns the distance between left and right wheels of the front
     * axel of the vehicle
     *
     * @return Distance between left and right wheels of the front axel of the
     *         vehicle
     */
    @Override
    public double getWheelDistLeftRightFrontSide() {
        if (!physicalVehicleInitialised) {
            throw new IllegalStateException("Front axel wheel distance can only be read after initialisation.");
        }
        return this.getVDM().getValue("TW_f");
    }

    /**
     * Function that sets the distance between left and right wheels of the front
     * axel of the vehicle
     *
     * @param wheelDistLeftRightFrontSide New distance between left and right wheels
     *                                    of the front axel of the vehicle
     */
    @Override
    public void setWheelDistLeftRightFrontSide(double wheelDistLeftRightFrontSide) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Front axel wheel distance can only be set before initialisation.");
        }
        this.getVDM().setParameter("TW_f", wheelDistLeftRightFrontSide);
    }

    /**
     * Function that returns the distance between left and right wheels of the back
     * axel of the vehicle
     *
     * @return Distance between left and right wheels of the back axel of the
     *         vehicle
     */
    @Override
    public double getWheelDistLeftRightBackSide() {
        if (!physicalVehicleInitialised) {
            throw new IllegalStateException("Back axel wheel distance can only be set before initialisation.");
        }
        return this.getVDM().getValue("TW_r");
    }

    /**
     * Function that sets the distance between left and right wheels of the back
     * axel of the vehicle
     *
     * @param wheelDistLeftRightBackSide New distance between left and right wheels
     *                                   of the back axel of the vehicle
     */
    @Override
    public void setWheelDistLeftRightBackSide(double wheelDistLeftRightBackSide) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Position can only be set after initialisation.");
        }
        this.getVDM().setParameter("TW_r", wheelDistLeftRightBackSide);
    }

    /**
     * Function that returns the distance between the center of mass and the front
     * axel of the vehicle
     *
     * @return Distance between center of mass and front axel of the vehicle
     */
    @Override
    public double getWheelDistToFront() {
        if (!physicalVehicleInitialised) {
            throw new IllegalStateException(
                    "Distance from the center of mass to the front axel can only be read after initialisation.");
        }
        return this.getVDM().getValue("L_1");
    }

    /**
     * Function that sets the distance between the center of mass and the front axel
     * of the vehicle
     *
     * @param wheelDistToFront New distance between center of mass and front axel of
     *                         the vehicle
     */
    @Override
    public void setWheelDistToFront(double wheelDistToFront) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException(
                    "Distance from the center of mass to the front axel can only be set before initialisation.");
        }
        this.getVDM().setParameter("L_1", wheelDistToFront);
    }

    /**
     * Function that returns the distance between the center of mass and the back
     * axel of the vehicle
     *
     * @return Distance between center of mass and back axel of the vehicle
     */
    @Override
    public double getWheelDistToBack() {
        if (!physicalVehicleInitialised) {
            throw new IllegalStateException(
                    "Distance from the center of mass to the back axel can only be set before initialisation.");
        }
        return this.getVDM().getValue("L_2");
    }


    /**
     * Function that sets the distance between the center of mass and the back axel of the vehicle
     *
     * @param wheelDistToBack New distance between center of mass and back axel of the vehicle
     */
    @Override
    public void setWheelDistToBack(double wheelDistToBack) {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Distance from the center of mass to the back axel can only be read after initialisation.");
        }
        this.getVDM().setParameter("L_2", wheelDistToBack);
    }

    /**
     * Function that returns a copy of center of geometry position vector
     * @return Position vector of the center of geometry
     */
    @Override
    public Vec3 getGeometryPosition() {
        return position.add(getGeometryPositionOffset());
    }

    /**
     * Function that sets the center of geometry position vector.
     * @param geometryPosition New position vector of the center of geometry
     */
    @Override
    public void setGeometryPosition(Vec3 geometryPosition) {
        // Uses setter to check initialisation
        setPosition(geometryPosition.add(getGeometryPositionOffset().mapMultiply(-1.0)));

    }

    /**
     * Function that returns a copy of the vector pointing from the center of mass position to the center of geometry position
     * @return Offset vector of the center of mass position to the center of geometry position
     */
    @Override
    public Vec3 getGeometryPositionOffset() {
        return this.rotation.operate(geometryPositionOffset).copy();
    }

    /**
     * Function that sets the vector pointing from the center of mass position to the center of geometry position
     * @param geometryPositionOffset New offset vector of the center of mass position to the center of geometry position
     */
    @Override
    public void setGeometryPositionOffset(Vec3 geometryPositionOffset) {
        throw new UnsupportedOperationException("Geometry position offset is determined by the chassis rotation.");
    }

    /**
     * Function that returns a list of pairs of 3D coordinates, indicating a vector on the edges of the physical object
     * @return List of pairs of 3D points, indicating a vector on the edges of the physical object
     */
    @Override
    public List<Map.Entry<Vec3, Vec3>> getBoundaryVectors() {
        //TODO: Function is unnecessary with three dimensional collision detection
        // Build relative vectors between vertices
        Vec3 relVectorBackFront = new Vec3(new double[]{getLength(), 0.0, 0.0});
        Vec3 relVectorLeftRight = new Vec3(new double[]{0.0, -getWidth(), 0.0});
        Vec3 relVectorBottomTop = new Vec3(new double[]{0.0, 0.0, getHeight()});

        // Rotate relative vectors
        relVectorBackFront = getRotation().operate(relVectorBackFront);
        relVectorLeftRight = getRotation().operate(relVectorLeftRight);
        relVectorBottomTop = getRotation().operate(relVectorBottomTop);

        // From center coordinate, compute to bottom left vertex of box
        Vec3 absBackLeft = getGeometryPosition();
        absBackLeft = absBackLeft.add(relVectorBackFront.mapMultiply(-0.5));
        absBackLeft = absBackLeft.add(relVectorLeftRight.mapMultiply(-0.5));
        absBackLeft = absBackLeft.add(relVectorBottomTop.mapMultiply(-0.5));

        // Compute absolute vectors
        Vec3 backLeft = absBackLeft.copy();
        Vec3 backRight = absBackLeft.add(relVectorLeftRight);
        Vec3 frontLeft = absBackLeft.add(relVectorBackFront);
        Vec3 frontRight = absBackLeft.add(relVectorLeftRight).add(relVectorBackFront);

        // Put vectors in list and return
        // Create map entries and insert them into list
        // Ordering is important here
        List<Map.Entry<Vec3, Vec3>> boundaryVectors = new LinkedList<>();
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
    public void computePhysics(Duration deltaTime) {
        long deltaTms = deltaTime.toMillis();
        if (!this.getError()) {
            // Calculate input values
            // Get values from VDM
            double z = vehicleDynamicsModel.getValue("z");

            // Reset vehicle on surface and calculate slope and bank
            Vec3 roadPlane = position.add(rotation.operate(new Vec3(new double[]{0.0, 0.0, -z})));
            putOnSurface(roadPlane.getEntry(0), roadPlane.getEntry(1), yawAngle);
            this.clutch.setActuatorValueCurrent(0);
            this.clutch.setActuatorValueTarget(0);
            this.gear.setActuatorValueCurrent(1);
            this.gear.setActuatorValueTarget(1);

            // Do calculation steps with maximum step size as long as possible
            long currentDeltaTms = 0;
            int stepSizems = 2;
            while (currentDeltaTms + stepSizems <= deltaTms) {
                doCalculationStep(stepSizems);
                currentDeltaTms = currentDeltaTms + stepSizems;
            }

            // Do a calculation step with partial step size to fill the gap
            long partialStepSize = deltaTms - currentDeltaTms;
            if (partialStepSize > 0) {
                doCalculationStep(partialStepSize);
            }

            // Update the rotation and position
            z = vehicleDynamicsModel.getValue("z");
            roadPlane = position.add(rotation.operate(new Vec3(new double[]{0.0, 0.0, -z})));
            putOnSurface(roadPlane.getEntry(0), roadPlane.getEntry(1), yawAngle);
        }
        // Reset forces
        force = new Vec3(new double[]{0.0, 0.0, 0.0});

    }

    /**
     * Function that sets the position of the center of mass and the rotation of the object, in order to place the object on the surface of the world.
     * given a x, y coordinate and a z rotation
     * @param posX X component of the position of the physical object
     * @param posY Y component of the position of the physical object
     * @param rotZ Z component of the rotation of the physical object
     */
    @Override
    public void putOnSurface(double posX, double posY, double rotZ) {
        //Get values from VDM
        double z = vehicleDynamicsModel.getValue("z");
        double L_1 = vehicleDynamicsModel.getValue("L_1");
        double L_2 = vehicleDynamicsModel.getValue("L_2");
        double TW_f = vehicleDynamicsModel.getValue("TW_r");
        double TW_r = vehicleDynamicsModel.getValue("TW_r");

        //set on ground only rotated around z axis
        double ground = World.getInstance().getGround(posX, posY, 0.0).doubleValue();
        double posZ = ground + z;
        position = new Vec3(new double[]{posX, posY, posZ});

        //rotate around z axis
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, rotZ);
        rotation = coordinateRotation.multiply(new BlockRealMatrix(rot.getMatrix())).copy();

        //Get local positions
        Vec3 frontPositionLocal = new Vec3(new double[]{L_1, 0.0, -z});
        Vec3 backPositionLocal = new Vec3(new double[]{-L_2, 0.0, -z});
        Vec3 leftPositionLocal = new Vec3(new double[]{0.0, (TW_f / 2 + TW_r / 2) / 2, -z});
        Vec3 rightPositionLocal = new Vec3(new double[]{0.0, -(TW_f / 2 + TW_r / 2) / 2, -z});

        Vec3 backToFrontLocal = new Vec3(new double[]{L_1 + L_2, 0.0, 0.0});

        //Get global positions only rotated around z axis
        Vec3 frontPosition = position.add(rotation.operate(frontPositionLocal));
        Vec3 backPosition = position.add(rotation.operate(backPositionLocal));
        Vec3 leftPosition = position.add(rotation.operate(leftPositionLocal));
        Vec3 rightPosition = position.add(rotation.operate(rightPositionLocal));

        //Get ground values for rotated wheel positions
        double frontGround = World.getInstance().getGround(frontPosition.getEntry(0), frontPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();
        double backGround = World.getInstance().getGround(backPosition.getEntry(0), backPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();
        double leftGround = World.getInstance().getGround(leftPosition.getEntry(0), leftPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();
        double rightGround = World.getInstance().getGround(rightPosition.getEntry(0), rightPosition.getEntry(1), frontPosition.getEntry(2)).doubleValue();

        //Store new ground value in global wheel position
        frontPosition.setEntry(2, frontGround);
        backPosition.setEntry(2, backGround);
        leftPosition.setEntry(2, leftGround);
        rightPosition.setEntry(2, rightGround);

        //Compute relative vectors
        Vec3 backToFront = frontPosition.subtract(backPosition);
        Vec3 rightToLeft = leftPosition.subtract(rightPosition);
        Vec3 roadPlaneNorm = MathHelper.crossProduct(backToFront, rightToLeft);

        //Compute angles between relative vectors and X-Y-Plane
        Vec3 xyPlaneNorm = new Vec3(new double[]{0.0, 0.0, 1.0});
        double backToFrontAngle = (Math.PI / 2) - MathHelper.angle(xyPlaneNorm, backToFront);
        double rightToLeftAngle = (Math.PI / 2) - MathHelper.angle(xyPlaneNorm, rightToLeft);
        if (physicalVehicleInitialised) {
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
        Vec3 roadPlaneCenter = position.add(new Vec3(new double[]{0.0, 0.0, -z}));
        Vec3 roadPlaneCenterToPositionLocal = new Vec3(new double[]{0.0, 0.0, z});
        position = roadPlaneCenter.add(rotation.operate(roadPlaneCenterToPositionLocal));
    }

    /**
     * Function that returns a copy of the position vector of the center of the front right wheel
     * @return Position vector of the center of the front right wheel
     */
    @Override
    public Vec3 getFrontRightWheelGeometryPosition() {
        //Get values from VDM
        double L_1 = vehicleDynamicsModel.getValue(("L_1"));
        double TW_f = vehicleDynamicsModel.getValue("TW_f");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //double wheelanlge_1 = vehicleDynamicsModel.getValue("delta_");
        //Calculate localPosition and return global position
        Vec3 localPosition = new Vec3(new double[]{L_1, -TW_f / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that returns a copy of the position vector of the center of the front left wheel
     * @return Position vector of the center of the front left wheel
     */
    @Override
    public Vec3 getFrontLeftWheelGeometryPosition() {
        //Get values from VDM
        double L_1 = vehicleDynamicsModel.getValue(("L_1"));
        double TW_f = vehicleDynamicsModel.getValue("TW_f");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //Calculate localPosition and return global position
        Vec3 localPosition = new Vec3(new double[]{L_1, TW_f / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that returns a copy of the position vector of the center of the back right wheel
     * @return Position vector of the center of the back right wheel
     */
    @Override
    public Vec3 getBackRightWheelGeometryPosition() {
        //Get values from VDM
        double L_2 = vehicleDynamicsModel.getValue(("L_2"));
        double TW_r = vehicleDynamicsModel.getValue("TW_r");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //Calculate localPosition and return global position
        Vec3 localPosition = new Vec3(new double[]{-L_2, -TW_r / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    /**
     * Function that returns a copy of the position vector of the center of the back left wheel
     * @return Position vector of the center of the back left wheel
     */
    @Override
    public Vec3 getBackLeftWheelGeometryPosition() {
        //Get values from VDM
        double L_2 = vehicleDynamicsModel.getValue(("L_2"));
        double TW_r = vehicleDynamicsModel.getValue("TW_r");
        double z = vehicleDynamicsModel.getValue("z");
        double r_nom = vehicleDynamicsModel.getValue("r_nom");
        //Calculate localPosition and return global position
        Vec3 localPosition = new Vec3(new double[]{-L_2, TW_r / 2, -z + r_nom});
        return position.add(rotation.operate(localPosition));
    }

    @Override
    protected void initializeActuators() {
        super.initializeActuators();
        if (gear != null || clutch != null || brakes != null || throttle != null) {
            throw new IllegalStateException("Actuators can only be initialised once.");
        }
        VehicleActuatorType actuatorTypes[] = {VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE,
                VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH,
                VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR,
                VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE};
        List<VehicleActuator> actuators = super.initializeActuators(actuatorTypes);
        for (VehicleActuator actuator : actuators) {
            switch (actuator.getActuatorType()) {
                case VEHICLE_ACTUATOR_TYPE_GEAR:
                    gear = actuator;
                    break;
                case VEHICLE_ACTUATOR_TYPE_CLUTCH:
                    clutch = actuator;
                    break;
                case VEHICLE_ACTUATOR_TYPE_BRAKE:
                    brakes = actuator;
                    break;
                case VEHICLE_ACTUATOR_TYPE_THROTTLE:
                    throttle = actuator;
                    break;
                default:
                    Log.warning("Unknown actuator property");
            }
        }
    }

    /**
     * Function that returns the current vehicle actuators
     *
     * @param type Type of the vehicle actuator to get
     * @return Current vehicle actuator object for the type, otherwise null
     */
    public VehicleActuator getVehicleActuator(VehicleActuatorType type) {
        switch (type) {
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
     * Function that initialises the physics computations when the physicalVehicle is created
     * Should only be called by builder
     */
    @Override
    public void initPhysics() {
        if (physicalVehicleInitialised) {
            throw new IllegalStateException("Physical Vehicle can only be initialised once.");
        }
        // Set parameters for the VDM
        vehicleDynamicsModel.setParameter("rho_air", PhysicsEngine.AIR_DENSITY);
        vehicleDynamicsModel.setParameter("g", -PhysicsEngine.GRAVITY_EARTH);

        // Initialise the modelica components
        vehicleDynamicsModel.initialise();

        //Shift position and geometryPositionOffset
        double z = vehicleDynamicsModel.getValue("z");
        geometryPositionOffset = new Vec3(new double[]{0.0, 0.0, this.getHeight() / 2 - z});
        position = geometryPositionOffset.mapMultiply(-1.0);

        //Initialise remaining variables
        force = new Vec3(new double[]{0.0, 0.0, 0.0});
        yawAngle = 0.0;

        physicalVehicleInitialised = true;
    }

    @Override
    public void instantiatePhysicalVehicle() {
        this.vehicleDynamicsModel = new VehicleDynamicsModel();
    }

    /**
     * Function that returns the force that is acting on the vehicle
     * @return Force acting on the vehicle
     */
    @Override
    public Vec3 getForce() {
        return force.copy();
    }

    /**
     * Function that returns the torque that is acting on the vehicle
     * @return Torque acting on the vehicle
     */
    @Override
    public Vec3 getTorque() {
        //TODO: Expand the model to accept external torques
        throw new UnsupportedOperationException("External torques are currently not supported.");
    }


    /**
     * Function that returns the VDM
     * Should only be called by the builder and vehicle
     * @return The VDM of the vehicle
     */
    public VehicleDynamicsModel getVDM() {
        return vehicleDynamicsModel;
    }

    /**
     * Function that does a calculation step
     * @param deltaTms Length of the calculation step in milliseconds
     */
    private void doCalculationStep(long deltaTms) {
        // Calculate input values

        double deltaT = deltaTms / 1000.0;
        // Get motor acceleration and convert it in torque
        double throttle = getVehicleActuator(VEHICLE_ACTUATOR_TYPE_THROTTLE).getActuatorValueCurrent();
        vehicleDynamicsModel.setInput("t_input", throttle);

        // Get brake acceleration and convert it in torque
        double breakinput = getVehicleActuator((VEHICLE_ACTUATOR_TYPE_BRAKE)).getActuatorValueCurrent();
        vehicleDynamicsModel.setInput("b_input", breakinput);

        // Get steering angle and changerate
        double steeringAngle = getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();
        vehicleDynamicsModel.setInput("delta_sw", steeringAngle);
        double steeringAngleSpeed = (steeringAngle - old_sw_angle) / deltaT;
        vehicleDynamicsModel.setInput("omega_sw", steeringAngleSpeed);

        double clutchinput = getVehicleActuator(VEHICLE_ACTUATOR_TYPE_CLUTCH).getActuatorValueCurrent();
        vehicleDynamicsModel.setInput("c_input", clutchinput);

        double gear = getVehicleActuator(VEHICLE_ACTUATOR_TYPE_GEAR).getActuatorValueCurrent();
        vehicleDynamicsModel.setInput("i", gear);

        // Express the force vector in local coordinates
        Vec3 localForce = rotation.transpose().operate(force);
        vehicleDynamicsModel.setInput("F_ext_x", localForce.getEntry(0));
        vehicleDynamicsModel.setInput("F_ext_y", localForce.getEntry(1));

        // Take the wheel positions and get the frictions coefficients
        //TODO: Let the physical vehicle look up the ground type and not only the weather
        double frictionCoefficient = PhysicsEngine.calcFrictionCoefficient(getPosition());
        vehicleDynamicsModel.setInput("mu_1", frictionCoefficient);
        vehicleDynamicsModel.setInput("mu_2", frictionCoefficient);
        vehicleDynamicsModel.setInput("mu_3", frictionCoefficient);
        vehicleDynamicsModel.setInput("mu_4", frictionCoefficient);

        // Store z coordinate for interpolation later
        double oldZ = vehicleDynamicsModel.getValue("z");
        double oldv_x = vehicleDynamicsModel.getValue("v_x");
        double oldv_y = vehicleDynamicsModel.getValue("v_y");
        double oldv_z = vehicleDynamicsModel.getValue("v_z");

        // Do a computation step
        vehicleDynamicsModel.doStep(deltaT);

        // Integrate over model output
        // Integrate over the yaw rotation rate
        double omega_z = vehicleDynamicsModel.getValue("omega_z");
        double alpha_z = vehicleDynamicsModel.getValue("alpha_z");
        yawAngle = yawAngle + omega_z * deltaT + alpha_z * Math.pow(deltaT, 2);
        // Integrate over the velocity
        double a_x = vehicleDynamicsModel.getValue("a_x");
        double a_y = vehicleDynamicsModel.getValue("a_y");
        double a_z = vehicleDynamicsModel.getValue("a_z");
        double delta_x = oldv_x * deltaT + 0.5 * a_x * Math.pow(deltaT, 2);
        double delta_y = oldv_y * deltaT + 0.5 * a_y * Math.pow(deltaT, 2);
        double delta_z = oldv_z * deltaT + 0.5 * a_z * Math.pow(deltaT, 2);
        Vec3 delta_xyz = new Vec3(new double[]{delta_x, delta_y, delta_z});
        Vec3 rotate_delta = rotation.operate(delta_xyz);
        position = position.add(rotate_delta);

        // Update geometryPositionOffset
        double z = vehicleDynamicsModel.getValue("z");
        Vec3 deltaZ = new Vec3(new double[]{0.0, 0.0, oldZ - z});
        geometryPositionOffset = geometryPositionOffset.add(deltaZ);

        //remember old st_angle value
        old_sw_angle = getVehicleActuator(VEHICLE_ACTUATOR_TYPE_STEERING).getActuatorValueCurrent();
        //Set velocity to zero when braking if very near to zero
        double velocity = getVelocity().getNorm();
        if (velocity <= 0.1 && getVehicleActuator(VEHICLE_ACTUATOR_TYPE_BRAKE).getActuatorValueCurrent() > 0 && getVehicleActuator(VEHICLE_ACTUATOR_TYPE_THROTTLE).getActuatorValueCurrent() == 0) {

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
        return "PhysicalVehicle " + getId() +
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
                " , simulationVehicle: " + this.getVehicle();
    }

    @Override
    public void setCharging(boolean isCharging) {
        //TODO
        throw new java.lang.UnsupportedOperationException("Not supported yet.");
    }
}
