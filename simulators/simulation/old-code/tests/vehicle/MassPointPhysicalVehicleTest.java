/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import de.rwth.montisim.commons.utils.Vec3;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import de.rwth.montisim.commons.utils.Vec3;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;
import de.rwth.montisim.simulation.util.Log;
import de.rwth.montisim.simulation.util.MathHelper;

import java.time.Duration;
import java.time.Instant;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

/**
 * JUnit test for the MassPointPhysicalVehicle class
 */
public class MassPointPhysicalVehicleTest {
    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void setHeightNormal() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setHeight(1.0);
        Assert.assertEquals(1.0, physicalVehicle.getHeight(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setHeightFail() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setHeight(1.0);
    }

    @Test
    public void executeLoopIterationNoFlags() {
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();

        // Set up normal vehicle
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        // Set values for vehicle actuators
        VehicleActuator motor = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        motor.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);
        VehicleActuator frontLeftBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        frontLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator frontRightBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        frontRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator backLeftBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        backLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator backRightBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);
        backRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator steering = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Create reference actuators
        VehicleActuator motorReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR,
                motor.getActuatorValueMin(),
                motor.getActuatorValueMax(),
                motor.getActuatorValueChangeRate(), simulator, Collections.emptyList(), targetsByMessageId);
        motorReference.setActuatorValueTarget(motor.getActuatorValueTarget());
        motorReference.setActuatorValueCurrent(motor.getActuatorValueCurrent());

        VehicleActuator frontLeftBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT,
                frontLeftBrake.getActuatorValueMin(),
                frontLeftBrake.getActuatorValueMax(),
                frontLeftBrake.getActuatorValueChangeRate(), simulator, Collections.emptyList(), targetsByMessageId);
        frontLeftBrakeReference.setActuatorValueTarget(frontLeftBrake.getActuatorValueTarget());
        frontLeftBrakeReference.setActuatorValueCurrent(frontLeftBrake.getActuatorValueCurrent());

        VehicleActuator frontRightBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT,
                frontRightBrake.getActuatorValueMin(),
                frontRightBrake.getActuatorValueMax(),
                frontRightBrake.getActuatorValueChangeRate(), simulator, Collections.emptyList(), targetsByMessageId);
        frontRightBrakeReference.setActuatorValueTarget(frontRightBrake.getActuatorValueTarget());
        frontRightBrakeReference.setActuatorValueCurrent(frontRightBrake.getActuatorValueCurrent());

        VehicleActuator backLeftBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT,
                backLeftBrake.getActuatorValueMin(),
                backLeftBrake.getActuatorValueMax(),
                backLeftBrake.getActuatorValueChangeRate(), simulator, Collections.emptyList(), targetsByMessageId);
        backLeftBrakeReference.setActuatorValueTarget(backLeftBrake.getActuatorValueTarget());
        backLeftBrakeReference.setActuatorValueCurrent(backLeftBrake.getActuatorValueCurrent());

        VehicleActuator backRightBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT,
                backRightBrake.getActuatorValueMin(),
                backRightBrake.getActuatorValueMax(),
                backRightBrake.getActuatorValueChangeRate(), simulator, Collections.emptyList(), targetsByMessageId);
        backRightBrakeReference.setActuatorValueTarget(backRightBrake.getActuatorValueTarget());
        backRightBrakeReference.setActuatorValueCurrent(backRightBrake.getActuatorValueCurrent());

        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING,
                steering.getActuatorValueMin(),
                steering.getActuatorValueMax(),
                steering.getActuatorValueChangeRate(), simulator, Collections.emptyList(), targetsByMessageId);
        steeringReference.setActuatorValueTarget(steering.getActuatorValueTarget());
        steeringReference.setActuatorValueCurrent(steering.getActuatorValueCurrent());

        // Execute loop iteration
        vehicle.executeLoopIteration(Duration.ofMillis(33));
        motorReference.update(Instant.EPOCH.plusMillis(33));
        frontLeftBrakeReference.update(Instant.EPOCH.plusMillis(33));
        frontRightBrakeReference.update(Instant.EPOCH.plusMillis(33));
        backLeftBrakeReference.update(Instant.EPOCH.plusMillis(33));
        backRightBrakeReference.update(Instant.EPOCH.plusMillis(33));
        steeringReference.update(Instant.EPOCH.plusMillis(33));

        // All actuators should be updated
        Assert.assertEquals(motorReference.getActuatorValueCurrent(), motor.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(frontLeftBrakeReference.getActuatorValueCurrent(), frontLeftBrake.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(frontRightBrakeReference.getActuatorValueCurrent(), frontRightBrake.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(backLeftBrakeReference.getActuatorValueCurrent(), backLeftBrake.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(backRightBrakeReference.getActuatorValueCurrent(), backRightBrake.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(steeringReference.getActuatorValueCurrent(), steering.getActuatorValueCurrent(), 0.001);
        // Collision flag should not be set
        Assert.assertFalse(physicalVehicle.getCollision());
    }

    @Test
    public void executeLoopIterationCollisionFlag() {
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();

        // Set up vehicle with collision
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setCollision(true);

        // Set values for vehicle actuators
        VehicleActuator motor = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        motor.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MIN);
        motor.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);

        VehicleActuator frontLeftBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        frontLeftBrake.setActuatorValueCurrent(1.0);
        frontLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator frontRightBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        frontRightBrake.setActuatorValueCurrent(1.0);
        frontRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backLeftBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        backLeftBrake.setActuatorValueCurrent(1.0);
        backLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backRightBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);
        backRightBrake.setActuatorValueCurrent(1.0);
        backRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator steering = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Create reference actuators
        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING,
                steering.getActuatorValueMin(),
                steering.getActuatorValueMax(),
                steering.getActuatorValueChangeRate(), simulator, Collections.emptyList(), targetsByMessageId);
        steeringReference.setActuatorValueTarget(steering.getActuatorValueTarget());
        steeringReference.setActuatorValueCurrent(steering.getActuatorValueCurrent());

        // Execute loop iteration
        vehicle.executeLoopIteration(Duration.ofMillis(33));
        steeringReference.update(Instant.EPOCH.plusMillis(33));

        // Motor and brake actuators should be reset to zero
        Assert.assertEquals(0.0, motor.getActuatorValueCurrent(), 0);
        Assert.assertEquals(0.0, frontLeftBrake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(0.0, frontRightBrake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(0.0, backLeftBrake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(0.0, backRightBrake.getActuatorValueCurrent(), 0);

        // Steering actuator should be updated
        Assert.assertEquals(steeringReference.getActuatorValueCurrent(), steering.getActuatorValueCurrent(), 0.001);

        // Collision flag should not be set
        Assert.assertFalse(physicalVehicle.getCollision());
    }

    @Test
    public void executeLoopIterationErrorFlag() {
        // Set up vehicle with an error
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setError(true);

        // Set values for vehicle actuators
        VehicleActuator motor = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        motor.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);

        VehicleActuator frontLeftBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        frontLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator frontRightBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        frontRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backLeftBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        backLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backRightBrake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);
        backRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator steering = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Get reference values from actuators
        double motorValueReference = motor.getActuatorValueCurrent();
        double frontLeftBrakeValueReference = frontLeftBrake.getActuatorValueCurrent();
        double frontRightBrakeValueReference = frontRightBrake.getActuatorValueCurrent();
        double backLeftBrakeValueReference = backLeftBrake.getActuatorValueCurrent();
        double backRightBrakeValueReference = backRightBrake.getActuatorValueCurrent();
        double steeringValueReference = steering.getActuatorValueCurrent();

        // Execute loop iteration
        vehicle.executeLoopIteration(Duration.ofMillis(33));

        // Ass actuators should not be updated
        Assert.assertEquals(motorValueReference, motor.getActuatorValueCurrent(), 0);
        Assert.assertEquals(frontLeftBrakeValueReference, frontLeftBrake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(frontRightBrakeValueReference, frontRightBrake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(backLeftBrakeValueReference, backLeftBrake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(backRightBrakeValueReference, backRightBrake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(steeringValueReference, steering.getActuatorValueCurrent(), 0);

        // Error flag should be set
        Assert.assertTrue(physicalVehicle.getError());
    }

    @Test
    public void setPositionNormal() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        Vec3 position = new Vec3(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setPosition(position);
        Assert.assertTrue(MathHelper.vectorEquals(position, physicalVehicle.getPosition(), 0.00000001));
        //TODO: check massPoint information
    }

    @Test(expected = IllegalStateException.class)
    public void setPositionFail() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setPosition(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test
    public void setRotationNormal() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 1.0, 2.0, 3.0);
        RealMatrix rotation = new BlockRealMatrix(rot.getMatrix());
        physicalVehicle.setRotation(rotation);
        Assert.assertTrue(MathHelper.matrixEquals(rotation, physicalVehicle.getRotation(), 0.00000001));
        //TODO: check massPoint information
    }

    @Test(expected = IllegalStateException.class)
    public void setRotationFail() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 1.0, 2.0, 3.0);
        RealMatrix rotation = new BlockRealMatrix(rot.getMatrix());
        physicalVehicle.setRotation(rotation);
    }

    @Test
    public void setVelocityUninitialised() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        Vec3 velocity = new Vec3(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setVelocity(velocity);
        physicalVehicle.initPhysics();
        Assert.assertTrue(MathHelper.vectorEquals(velocity, physicalVehicle.getVelocity(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setVelocityInitialised() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setVelocity(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test
    public void setAngularVelocityUninitialized() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        Vec3 angularVelocity = new Vec3(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setAngularVelocity(angularVelocity);
        physicalVehicle.initPhysics();
        Assert.assertTrue(MathHelper.vectorEquals(angularVelocity, physicalVehicle.getAngularVelocity(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setAngularVelocityInitialized() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setAngularVelocity(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test
    public void setMassNormal() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
        Assert.assertEquals(1000.0, physicalVehicle.getMass(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFail() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test
    public void setGeometryPositionNormal() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        Vec3 geometryPosition = new Vec3(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setGeometryPosition(geometryPosition);
        Assert.assertTrue(MathHelper.vectorEquals(geometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
        //TODO: check massPoint information
    }

    @Test(expected = IllegalStateException.class)
    public void setPositionGeometryFail() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.setGeometryPosition(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = UnsupportedOperationException.class)
    public void setGeometryPositionOffsetFail() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.setGeometryPositionOffset(new Vec3(3));
    }

    @Test(expected = IllegalStateException.class)
    public void computePhysicsFail() {
        MassPointPhysicalVehicle physicalVehicle = new MassPointPhysicalVehicle();
        physicalVehicle.computePhysics(Duration.ofMillis(33));
    }

    @Test(expected = IllegalStateException.class)
    public void initPhysicsFail() {
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        physicalVehicle.initPhysics();
    }

    /**
     * Tests if a vehicle drives in a straight line when undisturbed
     */
    @Test
    public void testDriveStraightForward() {
        // Create a new vehicle with a velocity
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        physicalVehicleBuilder.setVelocity(new Vec3(new double[]{0.0, 14.0, 0.0}));
        Vehicle vehicle = createStandardVehicle(physicalVehicleBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        // Put physical vehicle on the surface
        physicalVehicle.putOnSurface(0.0, 0.0, 0.0);

        // Remember start position projected on xz plane
        Vec3 startPosition = physicalVehicle.getPosition();
        startPosition.setEntry(1, 0.0);

        // Mock a simulation
        mockSimulation(physicalVehicle, 5000, 33);

        // Remember end position projected on xz plane
        Vec3 endPosition = physicalVehicle.getPosition();
        endPosition.setEntry(1, 0.0);

        // Calculated projected distance
        Vec3 projectedDistance = endPosition.subtract(startPosition);

        // Check if distance is zero
        Assert.assertEquals(0.0, projectedDistance.getNorm(), 0);
    }

    /**
     * Tests whether the vehicle does not move if there is no acceleration
     */
    @Test
    public void testNoDriveIfNoAcceleration() {
        // Create a new vehicle
        MassPointPhysicalVehicleBuilder massPointBuilder = new MassPointPhysicalVehicleBuilder();
        Vehicle vehicle = createStandardVehicle(massPointBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        // Add physicalVehicle to simulation
        physicalVehicle.putOnSurface(0.0, 0.0, 0.0);

        // Remember start position
        Vec3 startPosition = physicalVehicle.getPosition();

        // Mock a simulation
        mockSimulation(physicalVehicle, 5000, 33);

        // Compute distance driven
        Vec3 distance = physicalVehicle.getPosition().subtract(startPosition);

        // Check if distance is zero
        Assert.assertEquals(0.0, distance.getNorm(), 0);
    }

    /**
     * Test if air drag and friction brings the vehicle to a full stop
     */
    @Test
    public void testWillComeToHold() {
        // Create a new vehicle with a velocity
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        physicalVehicleBuilder.setVelocity(new Vec3(new double[]{0.0, 0.01, 0.0}));
        Vehicle vehicle = createStandardVehicle(physicalVehicleBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();

        // Add physicalVehicle to simulation
        physicalVehicle.putOnSurface(0.0, 0.0, 0.0);

        // Mock a simulation
        mockSimulation(physicalVehicle, 60000, 33);

        // Test if velocity is zero
        Assert.assertEquals(0.0, physicalVehicle.getVelocity().getNorm(), 0);
    }

    //TODO: Test if the rotation matrix stays orthogonal

    private void mockSimulation(PhysicalVehicle physicalVehicle, long simulationLength, long stepSize) {
        long currentTime = 0;
        while (currentTime < simulationLength) {
            physicalVehicle.computePhysics(Duration.ofMillis(stepSize));
            updateActuators(physicalVehicle, stepSize);
            currentTime = currentTime + stepSize;
        }
    }

    private void updateActuators(PhysicalVehicle physicalVehicle, long timeDiffMs) {
        physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).update(Instant.EPOCH.plusMillis(timeDiffMs));
        physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT).update(Instant.EPOCH.plusMillis(timeDiffMs));
        physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT).update(Instant.EPOCH.plusMillis(timeDiffMs));
        physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).update(Instant.EPOCH.plusMillis(timeDiffMs));
        physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT).update(Instant.EPOCH.plusMillis(timeDiffMs));
        physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).update(Instant.EPOCH.plusMillis(timeDiffMs));
    }

    private Vehicle createStandardVehicle(PhysicalVehicleBuilder physicalVehicleBuilder) {
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        return new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
    }
}
