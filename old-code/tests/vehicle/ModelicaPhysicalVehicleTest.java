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
 * JUnit test for the ModelicaPhysicalVehicle class
 */
public class ModelicaPhysicalVehicleTest {
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
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setHeight(1.0);
        Assert.assertEquals(1.0, physicalVehicle.getHeight(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setHeightFail() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.setHeight(1.0);
    }

    @Test
    public void executeLoopIterationNoFlags() {
        // Set up normal vehicle
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();

        // Set values for vehicle actuators
        VehicleActuator throttle = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE);
        throttle.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MIN);
        throttle.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MAX);

        VehicleActuator brake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE);
        brake.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_BRAKE_PRESSURE_MIN);
        brake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKE_PRESSURE_MAX);

        VehicleActuator clutch = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH);
        clutch.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_CLUTCH_POSITION_MIN);
        clutch.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_CLUTCH_POSITION_MAX);

        VehicleActuator gear = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR);
        gear.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_GEAR_MIN);
        gear.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_GEAR_MAX);

        VehicleActuator steering = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MIN);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        EESimulator eeSim = new EESimulator(Instant.EPOCH);

        HashMap<BusEntry, List<EEComponent>> emptyMap = new HashMap<BusEntry, List<EEComponent>>();

        // Create reference actuators
        VehicleActuator throttleReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE,
                throttle.getActuatorValueMin(),
                throttle.getActuatorValueMax(),
                throttle.getActuatorValueChangeRate(), eeSim, Collections.emptyList(), emptyMap);
        throttleReference.setActuatorValueTarget(throttle.getActuatorValueTarget());
        throttleReference.setActuatorValueCurrent(throttle.getActuatorValueCurrent());

        VehicleActuator brakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE,
                brake.getActuatorValueMin(),
                brake.getActuatorValueMax(),
                brake.getActuatorValueChangeRate(), eeSim, Collections.emptyList(), emptyMap);
        brakeReference.setActuatorValueTarget(brake.getActuatorValueTarget());
        brakeReference.setActuatorValueCurrent(brake.getActuatorValueCurrent());

        VehicleActuator clutchReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH,
                clutch.getActuatorValueMin(),
                clutch.getActuatorValueMax(),
                clutch.getActuatorValueChangeRate(), eeSim, Collections.emptyList(), emptyMap);
        clutchReference.setActuatorValueTarget(clutch.getActuatorValueTarget());
        clutchReference.setActuatorValueCurrent(clutch.getActuatorValueCurrent());

        VehicleActuator gearReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR,
                gear.getActuatorValueMin(),
                gear.getActuatorValueMax(),
                gear.getActuatorValueChangeRate(), eeSim, Collections.emptyList(), emptyMap);
        gearReference.setActuatorValueTarget(gear.getActuatorValueTarget());
        gearReference.setActuatorValueCurrent(gear.getActuatorValueCurrent());

        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING,
                steering.getActuatorValueMin(),
                steering.getActuatorValueMax(),
                steering.getActuatorValueChangeRate(), eeSim, Collections.emptyList(), emptyMap);
        steeringReference.setActuatorValueTarget(steering.getActuatorValueTarget());
        steeringReference.setActuatorValueCurrent(steering.getActuatorValueCurrent());

        // Execute loop iteration
        vehicle.executeLoopIteration(Duration.ofMillis(33));
        throttleReference.update(Instant.EPOCH.plusMillis(33));
        brakeReference.update(Instant.EPOCH.plusMillis(33));
        clutchReference.update(Instant.EPOCH.plusMillis(33));
        gearReference.update(Instant.EPOCH.plusMillis(33));
        steeringReference.update(Instant.EPOCH.plusMillis(33));

        // All actuators should be updated
        Assert.assertEquals(throttleReference.getActuatorValueCurrent(), throttle.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(brakeReference.getActuatorValueCurrent(), brake.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(clutchReference.getActuatorValueCurrent(), clutch.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(gearReference.getActuatorValueCurrent(), gear.getActuatorValueCurrent(), 0.001);
        Assert.assertEquals(steeringReference.getActuatorValueCurrent(), steering.getActuatorValueCurrent(), 0.001);
        // Collision flag should not be set
        Assert.assertFalse(physicalVehicle.getCollision());
    }

    @Test
    public void executeLoopIterationCollisionFlag() {
        // Set up vehicle with collision
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.setCollision(true);

        // Set values for vehicle actuators
        VehicleActuator throttle = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE);
        throttle.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MIN);
        throttle.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MAX);

        VehicleActuator brake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE);
        brake.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_BRAKE_PRESSURE_MIN);
        brake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKE_PRESSURE_MAX);

        VehicleActuator clutch = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH);
        clutch.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_CLUTCH_POSITION_MIN);
        clutch.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_CLUTCH_POSITION_MAX);

        VehicleActuator gear = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR);
        gear.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_GEAR_MIN);
        gear.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_GEAR_MAX);

        VehicleActuator steering = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MIN);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        EESimulator eeSim = new EESimulator(Instant.EPOCH);

        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING,
                steering.getActuatorValueMin(),
                steering.getActuatorValueMax(),
                steering.getActuatorValueChangeRate(), eeSim, Collections.emptyList(), new HashMap<BusEntry, List<EEComponent>>());
        steeringReference.setActuatorValueTarget(steering.getActuatorValueTarget());
        steeringReference.setActuatorValueCurrent(steering.getActuatorValueCurrent());


        // Execute loop iteration
        vehicle.executeLoopIteration(Duration.ofMillis(33));
        steeringReference.update(Instant.EPOCH.plusMillis(33));

        // Throttle, brake, gear, and clutch actuators should be reset to minimum
        Assert.assertEquals(Vehicle.VEHICLE_DEFAULT_BRAKE_PRESSURE_MIN, brake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MIN, throttle.getActuatorValueCurrent(), 0);
        Assert.assertEquals(Vehicle.VEHICLE_DEFAULT_GEAR_MIN, gear.getActuatorValueCurrent(), 0);
        Assert.assertEquals(Vehicle.VEHICLE_DEFAULT_CLUTCH_POSITION_MIN, clutch.getActuatorValueCurrent(), 0);

        // Steering actuator should be updated
        Assert.assertEquals(steeringReference.getActuatorValueCurrent(), steering.getActuatorValueCurrent(), 0.001);

        // Collision flag should not be set
        Assert.assertFalse(physicalVehicle.getCollision());
    }

    @Test
    public void executeLoopIterationErrorFlag() {
        // Set up vehicle with an error
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.setError(true);

        // Set values for vehicle actuators
        VehicleActuator throttle = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE);
        throttle.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_THROTTLE_POSITION_MAX);

        VehicleActuator brake = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE);
        brake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKE_PRESSURE_MAX);

        VehicleActuator steering = physicalVehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Get reference values from actuators
        double motorValueReference = throttle.getActuatorValueCurrent();
        double frontLeftBrakeValueReference = brake.getActuatorValueCurrent();
        double steeringValueReference = steering.getActuatorValueCurrent();

        // Execute loop iteration
        vehicle.executeLoopIteration(Duration.ofMillis(33));

        // All actuators should not be updated
        Assert.assertEquals(motorValueReference, throttle.getActuatorValueCurrent(), 0);
        Assert.assertEquals(frontLeftBrakeValueReference, brake.getActuatorValueCurrent(), 0);
        Assert.assertEquals(steeringValueReference, steering.getActuatorValueCurrent(), 0);

        // Error flag should be set
        Assert.assertTrue(physicalVehicle.getError());
    }

    @Test
    public void setPositionNormal() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        Vec3 position = new Vec3(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setPosition(position);
        Assert.assertTrue(MathHelper.vectorEquals(position, physicalVehicle.getPosition(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setPositionFail() {
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setPosition(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test
    public void setRotationNormal() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 1.0, 2.0, 3.0);
        RealMatrix rotation = new BlockRealMatrix(rot.getMatrix());
        physicalVehicle.setRotation(rotation);
        Assert.assertTrue(MathHelper.matrixEquals(rotation, physicalVehicle.getRotation(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setRotationFail() {
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 1.0, 2.0, 3.0);
        RealMatrix rotation = new BlockRealMatrix(rot.getMatrix());
        physicalVehicle.setRotation(rotation);
    }

    @Test(expected = UnsupportedOperationException.class)
    public void setVelocityUninitialised() {
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setVelocity(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = IllegalStateException.class)
    public void setVelocityInitialised() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.setVelocity(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = IllegalStateException.class)
    public void setAngularVelocityInitialized() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.setAngularVelocity(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test
    public void setMassNormal() {
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
        physicalVehicle.initPhysics();
        Assert.assertEquals(1000.0, physicalVehicle.getMass(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFail() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test
    public void setGeometryPositionNormal() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        Vec3 geometryPosition = new Vec3(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setGeometryPosition(geometryPosition);
        Assert.assertTrue(MathHelper.vectorEquals(geometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setPositionGeometryFail() {
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setGeometryPosition(new Vec3(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = UnsupportedOperationException.class)
    public void setGeometryPositionOffsetFail() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.setGeometryPositionOffset(new Vec3(3));
    }

    @Test(expected = IllegalStateException.class)
    public void computePhysicsFail() {
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.computePhysics(Duration.ofMillis(33));
    }

    @Test(expected = IllegalStateException.class)
    public void initPhysicsFail() {
        Vehicle vehicle = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) vehicle.getPhysicalVehicle();
        physicalVehicle.initPhysics();
    }

    private Vehicle createStandardVehicle(PhysicalVehicleBuilder physicalVehicleBuilder) {
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        return new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
    }
}
