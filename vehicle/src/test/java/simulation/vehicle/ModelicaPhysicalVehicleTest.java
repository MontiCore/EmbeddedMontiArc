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

import java.time.Duration;
import java.time.Instant;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.EESimulator.EESimulator;
import simulation.util.Log;
import simulation.util.MathHelper;

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
    public void setHeightNormal(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setHeight(1.0);
        Assert.assertEquals(1.0, physicalVehicle.getHeight(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setHeightFail(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setHeight(1.0);
    }

    @Test
    public void executeLoopIterationNoFlags(){
        // Set up normal vehicle
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();

        // Set values for vehicle actuators
        VehicleActuator motor = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        motor.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);
        VehicleActuator frontLeftBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        frontLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator frontRightBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        frontRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator backLeftBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        backLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator backRightBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);
        backRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        VehicleActuator steering = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Create reference actuators
        VehicleActuator motorReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR,
                motor.getActuatorValueMin(),
                motor.getActuatorValueMax(),
                motor.getActuatorValueChangeRate(), simulator);
        motorReference.setActuatorValueTarget(motor.getActuatorValueTarget());
        motorReference.setActuatorValueCurrent(motor.getActuatorValueCurrent());

        VehicleActuator frontLeftBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT,
                frontLeftBrake.getActuatorValueMin(),
                frontLeftBrake.getActuatorValueMax(),
                frontLeftBrake.getActuatorValueChangeRate(), simulator);
        frontLeftBrakeReference.setActuatorValueTarget(frontLeftBrake.getActuatorValueTarget());
        frontLeftBrakeReference.setActuatorValueCurrent(frontLeftBrake.getActuatorValueCurrent());

        VehicleActuator frontRightBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT,
                frontRightBrake.getActuatorValueMin(),
                frontRightBrake.getActuatorValueMax(),
                frontRightBrake.getActuatorValueChangeRate(), simulator);
        frontRightBrakeReference.setActuatorValueTarget(frontRightBrake.getActuatorValueTarget());
        frontRightBrakeReference.setActuatorValueCurrent(frontRightBrake.getActuatorValueCurrent());

        VehicleActuator backLeftBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT,
                backLeftBrake.getActuatorValueMin(),
                backLeftBrake.getActuatorValueMax(),
                backLeftBrake.getActuatorValueChangeRate(), simulator);
        backLeftBrakeReference.setActuatorValueTarget(backLeftBrake.getActuatorValueTarget());
        backLeftBrakeReference.setActuatorValueCurrent(backLeftBrake.getActuatorValueCurrent());

        VehicleActuator backRightBrakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT,
                backRightBrake.getActuatorValueMin(),
                backRightBrake.getActuatorValueMax(),
                backRightBrake.getActuatorValueChangeRate(), simulator);
        backRightBrakeReference.setActuatorValueTarget(backRightBrake.getActuatorValueTarget());
        backRightBrakeReference.setActuatorValueCurrent(backRightBrake.getActuatorValueCurrent());

        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING,
                steering.getActuatorValueMin(),
                steering.getActuatorValueMax(),
                steering.getActuatorValueChangeRate(), simulator);
        steeringReference.setActuatorValueTarget(steering.getActuatorValueTarget());
        steeringReference.setActuatorValueCurrent(steering.getActuatorValueCurrent());

        // Execute loop iteration
        physicalVehicle.executeLoopIteration(Duration.ofMillis(33));
        motorReference.update(0.033);
        frontLeftBrakeReference.update(0.033);
        frontRightBrakeReference.update(0.033);
        backLeftBrakeReference.update(0.033);
        backRightBrakeReference.update(0.033);
        steeringReference.update(0.033);

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
    public void executeLoopIterationCollisionFlag(){
        // Set up vehicle with collision
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setCollision(true);

        // Set values for vehicle actuators
        VehicleActuator motor = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        motor.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MIN);
        motor.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);

        VehicleActuator frontLeftBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        frontLeftBrake.setActuatorValueCurrent(1.0);
        frontLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator frontRightBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        frontRightBrake.setActuatorValueCurrent(1.0);
        frontRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backLeftBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        backLeftBrake.setActuatorValueCurrent(1.0);
        backLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backRightBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);
        backRightBrake.setActuatorValueCurrent(1.0);
        backRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator steering = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Create reference actuators
        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING,
                steering.getActuatorValueMin(),
                steering.getActuatorValueMax(),
                steering.getActuatorValueChangeRate(), simulator);
        steeringReference.setActuatorValueTarget(steering.getActuatorValueTarget());
        steeringReference.setActuatorValueCurrent(steering.getActuatorValueCurrent());

        // Execute loop iteration
        physicalVehicle.executeLoopIteration(Duration.ofMillis(33));
        steeringReference.update(0.033);

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
    public void executeLoopIterationErrorFlag(){
        // Set up vehicle with an error
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setError(true);

        // Set values for vehicle actuators
        VehicleActuator motor = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        motor.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);

        VehicleActuator frontLeftBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        frontLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator frontRightBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        frontRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backLeftBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        backLeftBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator backRightBrake = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);
        backRightBrake.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);

        VehicleActuator steering = physicalVehicle.getSimulationVehicle().getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        // Get reference values from actuators
        double motorValueReference = motor.getActuatorValueCurrent();
        double frontLeftBrakeValueReference = frontLeftBrake.getActuatorValueCurrent();
        double frontRightBrakeValueReference = frontRightBrake.getActuatorValueCurrent();
        double backLeftBrakeValueReference = backLeftBrake.getActuatorValueCurrent();
        double backRightBrakeValueReference = backRightBrake.getActuatorValueCurrent();
        double steeringValueReference = steering.getActuatorValueCurrent();

        // Execute loop iteration
        physicalVehicle.executeLoopIteration(Duration.ofMillis(33));

        // All actuators should not be updated
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
    public void setPositionNormal(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        RealVector position = new ArrayRealVector(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setPosition(position);
        Assert.assertTrue(MathHelper.vectorEquals(position, physicalVehicle.getPosition(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setPositionFail(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setPosition(new ArrayRealVector(new double[]{1.0, 2.0, 3.0}));
    }

    @Test
    public void setRotationNormal(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 1.0, 2.0, 3.0);
        RealMatrix rotation = new BlockRealMatrix(rot.getMatrix());
        physicalVehicle.setRotation(rotation);
        Assert.assertTrue(MathHelper.matrixEquals(rotation, physicalVehicle.getRotation(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setRotationFail(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 1.0, 2.0, 3.0);
        RealMatrix rotation = new BlockRealMatrix(rot.getMatrix());
        physicalVehicle.setRotation(rotation);
    }

    @Test(expected = UnsupportedOperationException.class)
    public void setVelocityUninitialised(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setVelocity(new ArrayRealVector(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = IllegalStateException.class)
    public void setVelocityInitialised(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setVelocity(new ArrayRealVector(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = UnsupportedOperationException.class)
    public void setAngularVelocityUninitialized(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setAngularVelocity(new ArrayRealVector(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = IllegalStateException.class)
    public void setAngularVelocityInitialized(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setAngularVelocity(new ArrayRealVector(new double[]{1.0, 2.0, 3.0}));
    }

    @Test
    public void setMassNormal(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
        physicalVehicle.initPhysics();
        Assert.assertEquals(1000.0, physicalVehicle.getMass(), 0);
    }

    @Test(expected = IllegalStateException.class)
    public void setMassFail() {
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setMass(1000.0);
    }

    @Test
    public void setGeometryPositionNormal(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        RealVector geometryPosition = new ArrayRealVector(new double[]{1.0, 2.0, 3.0});
        physicalVehicle.setGeometryPosition(geometryPosition);
        Assert.assertTrue(MathHelper.vectorEquals(geometryPosition, physicalVehicle.getGeometryPosition(), 0.00000001));
    }

    @Test(expected = IllegalStateException.class)
    public void setPositionGeometryFail(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.setGeometryPosition(new ArrayRealVector(new double[]{1.0, 2.0, 3.0}));
    }

    @Test(expected = UnsupportedOperationException.class)
    public void setGeometryPositionOffsetFail(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.setGeometryPositionOffset(new ArrayRealVector(3));
    }

    @Test(expected = IllegalStateException.class)
    public void computePhysicsFail(){
        ModelicaPhysicalVehicle physicalVehicle = new ModelicaPhysicalVehicle();
        physicalVehicle.computePhysics(Duration.ofMillis(33));
    }

    @Test(expected = IllegalStateException.class)
    public void initPhysicsFail(){
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        physicalVehicle.initPhysics();
    }
}