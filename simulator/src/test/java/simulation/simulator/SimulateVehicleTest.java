package simulation.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.junit.*;
import simulation.util.Log;
import simulation.util.MathHelper;
import simulation.vehicle.*;
import static org.junit.Assert.assertTrue;

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

    //ToDo
    /**
     * Check if the vehicle drives straight forward, if there is no steering angle
     */
    @Test
    public void testDriveStraightForward() {
        Simulator sim = Simulator.getSharedInstance();

        // Create a new vehicle
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Add physicalVehicle to simulation
        sim.registerSimulationObject(physicalVehicle);

        vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).setActuatorValueTarget(Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX);

        RealVector startPosition = physicalVehicle.getPosition();

        //Run simulation
        sim.stopAfter(5000);
        sim.startSimulation();

        RealVector endPosition = physicalVehicle.getPosition();

        //Ignore y direction
        startPosition.setEntry(1,0);
        endPosition.setEntry(1,0);

        assertTrue(MathHelper.vectorEquals(startPosition, endPosition, 0.001));
    }

    /**
     * Checks, whether the vehicle does not move if there is no acceleration
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
    //ToDo
    /**
     * Test if the vehicle will drive in a straight line after steering to the right and
     * constantly accelerating
     */
    @Test
    public void testWillDriveInStraightLineAfterSteering() {
        Simulator sim = Simulator.getSharedInstance();

        // Create a new vehicle
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Add physicalVehicle to simulation
        sim.registerSimulationObject(physicalVehicle);

        // Set simulation duration (30 seconds)
        sim.stopAfter(30000);

        vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).setActuatorValueTarget(1);
        vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).setActuatorValueTarget(0.3);

        // Start simulation
        sim.stopAfter(5000);
        sim.startSimulation();

        vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).setActuatorValueTarget(0);
        RealVector afterSteeringValuePos = physicalVehicle.getPosition();

        sim.extendSimulationTime(20000);
        sim.startSimulation();

        RealVector endValuePos = physicalVehicle.getPosition();

        assertTrue(afterSteeringValuePos.getEntry(0) < endValuePos.getEntry(0));
        assertTrue(afterSteeringValuePos.getEntry(1) < endValuePos.getEntry(1));
        assertTrue(afterSteeringValuePos.getEntry(2) == endValuePos.getEntry(2));
    }

    //Todo
    /**
     * Test if the rotation matrix stays orthogonal
     */
    @Test
    public void testRotationMatrixStaysOrthogonal() {
        Simulator sim = Simulator.getSharedInstance();

        // Create a new vehicle
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        // Add physicalVehicle to simulation
        sim.registerSimulationObject(physicalVehicle);

        // Set simulation duration (30 seconds)
        sim.stopAfter(30000);

        //Start simulation
        sim.startSimulation();

        vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR).setActuatorValueTarget(1);
        vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).setActuatorValueTarget(0.3);

        // Start simulation
        sim.stopAfter(2000);
        sim.startSimulation();

        RealMatrix matrix1 = MathHelper.matrixInvert(physicalVehicle.getRotation());
        RealMatrix matrix2 =  physicalVehicle.getRotation().transpose();
        assertTrue(MathHelper.matrixEquals(matrix1, matrix2, 0.00001));

        vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).setActuatorValueTarget(0);

        sim.extendSimulationTime(2000);
        sim.startSimulation();

        matrix1 = MathHelper.matrixInvert(physicalVehicle.getRotation());
        matrix2 =  physicalVehicle.getRotation().transpose();
        assertTrue(MathHelper.matrixEquals(matrix1, matrix2, 0.00001));

        sim.extendSimulationTime(2000);
        sim.startSimulation();

        matrix1 = MathHelper.matrixInvert(physicalVehicle.getRotation());
        matrix2 =  physicalVehicle.getRotation().transpose();
        assertTrue(MathHelper.matrixEquals(matrix1, matrix2, 0.00001));
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