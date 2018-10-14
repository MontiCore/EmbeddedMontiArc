package simulation.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.junit.*;
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

    @Before
    public void setUp() {
        Simulator.resetSimulator();

        //Set update frequency to 30 loop iterations per second
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_FIXED_TIME);
        sim.setSimulationLoopFrequency(30);
        sim.setSynchronousSimulation(true);
        sim.setPausedInFuture(true);
    }

    private void setAccelerating(PhysicalVehicle physicalVehicle){
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Set actuator values for testing
        VehicleActuator motor = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        VehicleActuator brakes1 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        VehicleActuator brakes2 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        VehicleActuator brakes3 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        VehicleActuator brakes4 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);

        motor.setActuatorValueCurrent(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);
        motor.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);

        brakes1.setActuatorValueCurrent(0.0);
        brakes1.setActuatorValueTarget(0.0);

        brakes2.setActuatorValueCurrent(0.0);
        brakes2.setActuatorValueTarget(0.0);

        brakes3.setActuatorValueCurrent(0.0);
        brakes3.setActuatorValueTarget(0.0);

        brakes4.setActuatorValueCurrent(0.0);
        brakes4.setActuatorValueTarget(0.0);
    }

    private void setBraking(PhysicalVehicle physicalVehicle){
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Set actuator values for testing
        VehicleActuator motor = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        VehicleActuator brakes1 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
        VehicleActuator brakes2 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT);
        VehicleActuator brakes3 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        VehicleActuator brakes4 = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT);
        VehicleActuator steering = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);

        motor.setActuatorValueCurrent(0.0);
        motor.setActuatorValueTarget(0.0);

        brakes1.setActuatorValueCurrent(1.5);
        brakes1.setActuatorValueTarget(1.5);

        brakes2.setActuatorValueCurrent(1.5);
        brakes2.setActuatorValueTarget(1.5);

        brakes3.setActuatorValueCurrent(1.5);
        brakes3.setActuatorValueTarget(1.5);

        brakes4.setActuatorValueCurrent(1.5);
        brakes4.setActuatorValueTarget(1.5);

        steering.setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX);
    }
    /*
    @Test
    public void firstTest(){
        Simulator sim = Simulator.getSharedInstance();
        sim.registerLoopObserver(new SimulationPlotter2D());

        // Create a new vehicle
        ModelicaPhysicalVehicleBuilder physicalVehicleBuilder1 = new ModelicaPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle1 = physicalVehicleBuilder1.buildPhysicalVehicle();

        // Add physicalVehicle1 to simulation
        sim.registerAndPutObject(physicalVehicle1, 0.0, 0.0, 0.0);

        setAccelerating(physicalVehicle1);

        // Start simulation
        sim.stopAfter(5000);
        long firstRoundStartingTime = System.nanoTime();
        sim.startSimulation();
        long firstRoundEndTime = System.nanoTime();

        setBraking(physicalVehicle1);

        sim.extendSimulationTime(5000);
        long secondRoundStartingTime = System.nanoTime();
        sim.startSimulation();
        long secondRoundEndTime = System.nanoTime();

        sim.setPausedInFuture(false);
        sim.stopSimulation();
        System.out.println(firstRoundEndTime - firstRoundStartingTime + (secondRoundEndTime - secondRoundStartingTime));
        assertTrue(true);
    }*/

    /**
     * Tests if the vehicle drives straight forward, if there is no steering angle
     */
    @Test
    public void testDriveStraightForward() {
        Simulator sim = Simulator.getSharedInstance();

        // Create a new vehicle with a velocity
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        physicalVehicleBuilder.setVelocity(new ArrayRealVector(new double[]{0.0, 14.0, 0.0}));
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();

        // Add physicalVehicle to simulation
        sim.registerAndPutObject(physicalVehicle, 0.0, 0.0, 0.0);

        // Remember start position projected on xz plane
        RealVector startPosition = physicalVehicle.getPosition();
        startPosition.setEntry(1, 0.0);

        // Run simulation
        sim.stopAfter(5000);
        sim.startSimulation();

        // Remember end position projected on xz plane
        RealVector endPosition = physicalVehicle.getPosition();
        endPosition.setEntry(1, 0.0);

        // Calculated projected distance
        RealVector projectedDistance = endPosition.subtract(startPosition);

        // Check if distance is zero
        Assert.assertEquals(0.0, projectedDistance.getNorm(), 0);
    }

    /**
     * Tests whether the vehicle does not move if there is no acceleration
     */
    @Test
    public void testNoDriveIfNoAcceleration() {
        Simulator sim = Simulator.getSharedInstance();

        // Create a new vehicle
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();

        // Add physicalVehicle to simulation
        sim.registerAndPutObject(physicalVehicle, 0.0, 0.0, 0.0);

        // Remember start position
        RealVector startPosition = physicalVehicle.getPosition();

        //Run simulation
        sim.stopAfter(5000);
        sim.startSimulation();

        // Compute distance driven
        RealVector distance = physicalVehicle.getPosition().subtract(startPosition);

        // Check if distance is zero
        Assert.assertEquals(0.0, distance.getNorm(), 0);
    }

    /**
     * Test if air drag and friction brings the vehicle to a full stop
     */
    @Test
    public void testWillComeToHold() {
        Simulator sim = Simulator.getSharedInstance();

        // Create a new vehicle with a velocity
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        physicalVehicleBuilder.setVelocity(new ArrayRealVector(new double[]{0.0, 0.01, 0.0}));
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();

        // Add physicalVehicle to simulation
        sim.registerAndPutObject(physicalVehicle, 0.0, 0.0, 0.0);

        // Start simulation
        sim.stopAfter(60000);
        sim.startSimulation();

        // Test if velocity is zero
        Assert.assertEquals(0.0, physicalVehicle.getVelocity().getNorm(), 0);
    }
}