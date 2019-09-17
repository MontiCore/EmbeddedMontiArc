/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.Pair;
import org.junit.Test;
import sensors.CameraSensor;
import sensors.CompassSensor;
import sensors.LocationSensor;
import sensors.ObstacleSensor;
import sensors.abstractsensors.AbstractSensor;
import sensors.factory.SensorFactory;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.bus.Bus;
import simulation.bus.BusMessage;
import simulation.bus.FlexRay;
import simulation.bus.InstantBus;

import java.io.File;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static junit.framework.TestCase.assertEquals;
import static org.junit.Assert.assertTrue;

public class EEVehicleTest {


	@Test
	public void sendConstantBusData() {
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        Vehicle vehicle = physicalVehicle.getVehicle();
        EEVehicle eeVehicle = vehicle.getEEVehicle();
        Bus bus = eeVehicle.getBusList().get(0);
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(BusEntry.SENSOR_COMPASS, Collections.singletonList(bus));
        //subscribe to all
        AbstractSensor sensor1 = new CompassSensor(physicalVehicle, eeVehicle.getEESimulator(), EEVehicle.constantMessages, targetsByMessageId);
        //subscribe to one
        List<BusEntry> subscribedMsgs = new ArrayList<BusEntry>();
        subscribedMsgs.add(EEVehicle.constantMessages.get(4));
        AbstractSensor sensor2 = new CompassSensor(physicalVehicle, eeVehicle.getEESimulator(), subscribedMsgs, targetsByMessageId);
        //subscribe to 3
        subscribedMsgs = new ArrayList<BusEntry>();
        subscribedMsgs.add(EEVehicle.constantMessages.get(0));
        subscribedMsgs.add(EEVehicle.constantMessages.get(2));
        subscribedMsgs.add(EEVehicle.constantMessages.get(5));
        AbstractSensor sensor3 = new CompassSensor(physicalVehicle, eeVehicle.getEESimulator(), subscribedMsgs, targetsByMessageId);

        eeVehicle.getSensorList().add(sensor1);
        eeVehicle.getSensorList().add(sensor2);
        eeVehicle.getSensorList().add(sensor3);
        
        eeVehicle.setConstantBusData();
        
        EESimulator sim = eeVehicle.getEESimulator();
        assertEquals(sim.getEventList().size(), EEVehicle.constantMessages.size() - 1 + 2 +3);
	}
	
    @Test
    public void testConstructor() throws IOException {
        //set up all needed classes
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().createPhysicalVehicle();

        List<Bus> busList = new LinkedList<>();
        List<EEComponent> componentList = new LinkedList<>();
        List<EEComponent> sensorCompareList = new LinkedList<>();
        List<VehicleActuator> actuatorCompareList = new LinkedList<>();
        List<Bus> busCompareList = new LinkedList<>();
        List<EEComponent> bridgeCompareList = new LinkedList<>();
        List<EEComponent> busOneCompareList = new LinkedList<>();
        List<EEComponent> busTwoCompareList = new LinkedList<>();
        List<EEComponent> busThreeCompareList = new LinkedList<>();



        FlexRay busOne = new FlexRay(simulator);
        FlexRay busTwo = new FlexRay(simulator);
        InstantBus busThree = new InstantBus(simulator);
        busList.add(busOne);
        busList.add(busTwo);
        busList.add(busThree);
        busCompareList.add(busOne);
        busCompareList.add(busTwo);
        busCompareList.add(busThree);

        VehicleActuator actuator1 = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, busOne);
        VehicleActuator actuator2 = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE, busOne);
        VehicleActuator actuator3 = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, busOne);
        VehicleActuator clutch = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH, busOne);

        VehicleActuator actuator4 = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, busTwo);
        VehicleActuator actuator5 = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, busTwo);
        VehicleActuator motor = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR, busTwo);
        CameraSensor cam = (CameraSensor) SensorFactory.createSensor(CameraSensor.class, physicalVehicle, busTwo).get();
        LocationSensor location = (LocationSensor) SensorFactory.createSensor(LocationSensor.class, physicalVehicle, busTwo).get();


        VehicleActuator actuator6 = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, busThree);
        VehicleActuator gear = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR, busThree);
        VehicleActuator throttle = VehicleActuator.createVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE, busThree);
        ObstacleSensor obstacle = (ObstacleSensor) SensorFactory.createSensor(ObstacleSensor.class, physicalVehicle, busThree).get();

        //all components in general minus bridges
        componentList.add(actuator1);
        componentList.add(actuator2);
        componentList.add(actuator3);
        componentList.add(actuator4);
        componentList.add(actuator5);
        componentList.add(actuator6);
        componentList.add(clutch);
        componentList.add(motor);
        componentList.add(gear);
        componentList.add(throttle);
        componentList.add(cam);
        componentList.add(location);
        componentList.add(obstacle);
        //all actuator
        actuatorCompareList.add(actuator1);
        actuatorCompareList.add(actuator2);
        actuatorCompareList.add(actuator3);
        actuatorCompareList.add(actuator4);
        actuatorCompareList.add(actuator5);
        actuatorCompareList.add(actuator6);
        actuatorCompareList.add(clutch);
        actuatorCompareList.add(motor);
        actuatorCompareList.add(gear);
        actuatorCompareList.add(throttle);
        //all sensors
        sensorCompareList.add(cam);
        sensorCompareList.add(location);
        sensorCompareList.add(obstacle);
        //bus one
        busOneCompareList.add(actuator1);
        busOneCompareList.add(actuator2);
        busOneCompareList.add(actuator3);
        busOneCompareList.add(clutch);
        //bus two
        busTwoCompareList.add(actuator4);
        busTwoCompareList.add(actuator5);
        busTwoCompareList.add(motor);
        busTwoCompareList.add(cam);
        busTwoCompareList.add(location);
        //bus three
        busThreeCompareList.add(actuator6);
        busThreeCompareList.add(gear);
        busThreeCompareList.add(throttle);
        busThreeCompareList.add(obstacle);



        Bridge bridgeOneTwo = new Bridge(simulator, Pair.of(busOne, busTwo), Duration.ofNanos(1));
        Bridge bridgeTwoThree = new Bridge(simulator, Pair.of(busTwo, busThree), Duration.ofNanos(4));
        componentList.add(bridgeOneTwo);
        componentList.add(bridgeTwoThree);
        bridgeCompareList.add(bridgeOneTwo);
        bridgeCompareList.add(bridgeTwoThree);
        busOneCompareList.add(bridgeOneTwo);
        busTwoCompareList.add(bridgeOneTwo);
        busTwoCompareList.add(bridgeTwoThree);
        busThreeCompareList.add(bridgeTwoThree);


        System.out.println("Test create EEVehicle by using Lists");
        //set up EEVehicle
        Vehicle vehicle = new Vehicle(physicalVehicle, simulator, busList, componentList);



        //tests
        assertEquals(new HashSet<>(busCompareList), new HashSet<>(vehicle.getEEVehicle().getBusList()));
        assertEquals(new HashSet<>(actuatorCompareList), new HashSet<>(vehicle.getEEVehicle().getActuatorList()));
        assertEquals(new HashSet<>(bridgeCompareList), new HashSet<>(vehicle.getEEVehicle().getBridgeList()));
        assertEquals(new HashSet<>(sensorCompareList), new HashSet<>(vehicle.getEEVehicle().getSensorList()));
        assertEquals(new HashSet<>(busOneCompareList), new HashSet<>(busOne.getConnectedComponents()));
        assertEquals(new HashSet<>(busTwoCompareList), new HashSet<>(busTwo.getConnectedComponents()));
        assertEquals(new HashSet<>(busThreeCompareList), new HashSet<>(busThree.getConnectedComponents()));



//        System.out.println("Test create EEVehicle by using JSON File");
//        //store and load from JSON file
//        File file = new File("C:/Users/Freddy/Desktop/SWP/EEVehicle Testordner/test.txt");
//        vehicle.getEEVehicle().storeInFile(file, vehicle.getEEVehicle());
//
//        PhysicalVehicle physicalVehicleJSON = new MassPointPhysicalVehicleBuilder().createPhysicalVehicle();
//        Vehicle vehicleJSON = new Vehicle(physicalVehicleJSON, simulator, file);
//
//        //tests
//        assertTrue(componentIsConnected(new LinkedList<EEComponent>(busCompareList), new LinkedList<EEComponent>(vehicleJSON.getEEVehicle().getBusList())));
//        assertTrue(componentIsConnected(new LinkedList<EEComponent>(actuatorCompareList), new LinkedList<EEComponent>(vehicleJSON.getEEVehicle().getActuatorList())));
//        assertTrue(componentIsConnected(new LinkedList<EEComponent>(bridgeCompareList), new LinkedList<EEComponent>(vehicleJSON.getEEVehicle().getBridgeList())));
//        assertTrue(componentIsConnected(new LinkedList<EEComponent>(sensorCompareList), new LinkedList<EEComponent>(vehicleJSON.getEEVehicle().getSensorList())));
//        assertTrue(componentIsConnected(busOneCompareList, busOne.getConnectedComponents()));
//        assertTrue(componentIsConnected(busTwoCompareList, busTwo.getConnectedComponents()));
//        assertTrue(componentIsConnected(busThreeCompareList, busThree.getConnectedComponents()));


    }

    private boolean componentIsConnected(List<EEComponent> shouldBeConnected, List<EEComponent> isConnected) {
        boolean notFound = false;
        for (EEComponent shouldBe : shouldBeConnected) {
            if (notFound) {
                return false;
            }
            notFound = true;
            for (EEComponent is : isConnected) {
                if (shouldBe.getClass() == is.getClass()) {
                    notFound = false;
                    break;
                }
            }
        }
        return true;
    }

    @Test
    public void testMessageToActuator(){
	    //create vehicle
        PhysicalVehicle physicalVehicle = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        Vehicle vehicle = physicalVehicle.getVehicle();

        HashMap<BusEntry,List<EEComponent>> emptyMap = new HashMap<BusEntry, List<EEComponent>>();
        //create reference actuator
        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MIN, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_RATE, vehicle.getEEVehicle().getEESimulator(), Collections.emptyList(), emptyMap);
        VehicleActuator brakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE, vehicle.getEEVehicle().getEESimulator(), Collections.emptyList(), emptyMap);
        VehicleActuator steering = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).get();
        VehicleActuator brake = vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).get();
        
        //set to same inital value
        steeringReference.setActuatorValueCurrent(0);
        brakeReference.setActuatorValueCurrent(0);
        steering.setActuatorValueCurrent(0);
        brake.setActuatorValueCurrent(0);

        //create messages
        BusMessage messageSteering = new BusMessage(4.0d, 6, BusEntry.ACTUATOR_STEERING, Instant.EPOCH.plusMillis(1), UUID.randomUUID(), vehicle.getEEVehicle().getBusList().get(0));
        BusMessage messageBrak = new BusMessage(3.0d, 6, BusEntry.ACTUATOR_BRAKE, Instant.EPOCH.plusMillis(2), UUID.randomUUID(), vehicle.getEEVehicle().getBusList().get(0));
        vehicle.getEEVehicle().getEESimulator().addEvent(messageSteering);
        vehicle.getEEVehicle().getEESimulator().addEvent(messageBrak);

        //set reference actuator
        steeringReference.setActuatorValueTarget(4);
        brakeReference.setActuatorValueTarget(3);
        steeringReference.update(Instant.EPOCH.plusMillis(33));
        brakeReference.update(Instant.EPOCH.plusMillis(33));

        //loop for 33 milliseconds
        vehicle.executeLoopIteration(Instant.EPOCH.plusMillis(33));

        //test if value of actuator are correct
        assertEquals(steeringReference.getActuatorValueCurrent(), vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).get().getActuatorValueCurrent());
        assertEquals(brakeReference.getActuatorValueCurrent(), vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).get().getActuatorValueCurrent());

    }


}
