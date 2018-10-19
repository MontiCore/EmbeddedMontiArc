package simulation.simulator;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.util.Log;
import simulation.vehicle.*;

/**
 * JUnit Test-suite for simulating a vehicle
 */
public class SimulateVehicleTest {

    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    private void setAccelerating(PhysicalVehicle physicalVehicle, double value) {
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator motor = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);

        motor.setActuatorValueCurrent(value);
        motor.setActuatorValueTarget(value);
    }

    private void setBraking(PhysicalVehicle physicalVehicle, double value) {
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator brakes1 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        VehicleActuator brakes2 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        VehicleActuator brakes3 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        VehicleActuator brakes4 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);

        brakes1.setActuatorValueCurrent(value);
        brakes1.setActuatorValueTarget(value);

        brakes2.setActuatorValueCurrent(value);
        brakes2.setActuatorValueTarget(value);

        brakes3.setActuatorValueCurrent(value);
        brakes3.setActuatorValueTarget(value);

        brakes4.setActuatorValueCurrent(value);
        brakes4.setActuatorValueTarget(value);
    }

    private void setSteering(PhysicalVehicle physicalVehicle, double value) {
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator steering = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);

        steering.setActuatorValueTarget(value);
    }
    /*
    @Test
    public void firstTest() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        sim.registerLoopObserver(new SimulationPlotter2D());

        // Create a new vehicle
        PhysicalVehicle physicalVehicle1 = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();

        // Add physicalVehicle1 to simulation
        sim.registerAndPutObject(physicalVehicle1, 0.0, 0.0, 0.0);

        setAccelerating(physicalVehicle1, Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);
        setBraking(physicalVehicle1, 0.0);
        setSteering(physicalVehicle1, 0.0);

        // Start simulation
        sim.setSimulationDuration(20000);
        sim.setSimulationPauseTime(5000);
        long firstRoundStartingTime = System.nanoTime();
        sim.startSimulation();
        long firstRoundEndTime = System.nanoTime();

        setAccelerating(physicalVehicle1, 0.0);
        setBraking(physicalVehicle1, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX);
        setSteering(physicalVehicle1, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);

        long secondRoundStartingTime = System.nanoTime();
        sim.continueSimulation();
        long secondRoundEndTime = System.nanoTime();

        System.out.println(firstRoundEndTime - firstRoundStartingTime + (secondRoundEndTime - secondRoundStartingTime));
    }*/
}