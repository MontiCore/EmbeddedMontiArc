/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.simulator;

import org.junit.AfterClass;
import org.junit.BeforeClass;
import de.rwth.montisim.simulation.util.Log;
import simulation.vehicle.*;


/**
 * Debug test that can start a simulation to test implementation isolated to the simulator
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

    /*private void setAccelerating(PhysicalVehicle physicalVehicle, double value) {
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

    private void setThrottle(PhysicalVehicle physicalVehicle, double value){
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator throttle = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE);

        //throttle.setActuatorValueCurrent(value);
        throttle.setActuatorValueTarget(value);
    }

    private void setBrakePressure(PhysicalVehicle physicalVehicle, double value){
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator brakepressure = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE);

        brakepressure.setActuatorValueCurrent(value);
        brakepressure.setActuatorValueTarget(value);
    }

    private void setGear(PhysicalVehicle physicalVehicle, double value){
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator gear = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR);

        gear.setActuatorValueCurrent(value);
        gear.setActuatorValueTarget(value);
    }

    private void setClutch(PhysicalVehicle physicalVehicle, double value){
        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        VehicleActuator clutch = vehicle.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH);

        clutch.setActuatorValueCurrent(value);
        clutch.setActuatorValueTarget(value);
    }


    @Test
    public void firstTest() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(20000);

        sim.registerLoopObserver(new SimulationDebugPlotter("ModelicaComparasion"));

        // Create a new vehicle
        PhysicalVehicle physicalVehicle1 = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();

        // Add physicalVehicle1 to simulation
        sim.registerAndPutObject(physicalVehicle1, 0.0, 0.0, 0.0);

        setClutch(physicalVehicle1, 0);
        setThrottle(physicalVehicle1, 0.1);
        setBrakePressure(physicalVehicle1, 0);
        setSteering(physicalVehicle1, 6);
        setGear(physicalVehicle1, 1);

        // Start simulation
        sim.setSimulationPauseTime(5000);
        long firstRoundStartingTime = System.nanoTime();
        sim.startSimulation();
        long firstRoundEndTime = System.nanoTime();

        setThrottle(physicalVehicle1, 0.7);
        setBrakePressure(physicalVehicle1, 0);
        setSteering(physicalVehicle1, 3.0);
        setGear(physicalVehicle1, 3);
        setClutch(physicalVehicle1, 0);

        // Continue Simulation
        long secondRoundStartingTime = System.nanoTime();
        sim.continueSimulation(5000);
        long secondRoundEndTime = System.nanoTime();

        setThrottle(physicalVehicle1, 0.0);
        setBrakePressure(physicalVehicle1, 0.0);
        setSteering(physicalVehicle1, 3.0);
        setGear(physicalVehicle1, 1);
        setClutch(physicalVehicle1, 1);

        // Continue Simulation
        long thirdRoundStartingTime = System.nanoTime();
        sim.continueSimulation();
        long thirdRoundEndTime = System.nanoTime();

        System.out.println(firstRoundEndTime - firstRoundStartingTime + (secondRoundEndTime - secondRoundStartingTime) + (thirdRoundEndTime - thirdRoundStartingTime));

    }
    @Test
    public void Masspointstraight(){
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(25000);

        sim.registerLoopObserver(new SimulationDebugPlotter("MassPointStraight"));

        // Create a new vehicle
        PhysicalVehicle physicalVehicle2 = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();

        sim.registerAndPutObject(physicalVehicle2, 0.0, 0.0, 0.0);

        setAccelerating(physicalVehicle2, 3.5);
        setBraking(physicalVehicle2, 0);
        setSteering(physicalVehicle2, 0);

        sim.setSimulationPauseTime(10000);
        sim.startSimulation();


        setAccelerating(physicalVehicle2, 0);
        setBraking(physicalVehicle2, 5);
        setSteering(physicalVehicle2, 0);
        sim.continueSimulation();

    }

    @Test
    public void straighttest(){
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(10000);

        sim.registerLoopObserver(new SimulationDebugPlotter("ModelicaStraight"));

        // Create a new vehicle
        PhysicalVehicle physicalVehicle1 = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();

        // Add physicalVehicle1 to simulation
        sim.registerAndPutObject(physicalVehicle1, 0.0, 0.0, 0.0);

        //setClutch(physicalVehicle1, 0);
        setThrottle(physicalVehicle1, 0.1);
        setBrakePressure(physicalVehicle1, 0);
        setSteering(physicalVehicle1, 6.0);
        //setGear(physicalVehicle1, 1);

       // sim.setSimulationPauseTime(10000);
        sim.startSimulation();


        //setClutch(physicalVehicle1, 1);
        setThrottle(physicalVehicle1, 0.0);
        setBrakePressure(physicalVehicle1, 0);
        setSteering(physicalVehicle1, 0.0);
        //setGear(physicalVehicle1, 1);

        sim.continueSimulation();

    }*/

}
