/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import static junit.framework.TestCase.assertEquals;

import java.io.IOException;
import java.time.Instant;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.junit.Assert;
import org.junit.Test;

import com.google.common.collect.Sets;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.bus.Bus;
import simulation.bus.FlexRay;
import simulation.bus.InstantBus;

public class EEVehicleBuilderTest {

    @Test
    public void testConstructor() throws IOException {
        //set up all needed classes
    	PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
    	
        Set<Bus> buses = new HashSet<>();
        List<EEComponent> componentList = new LinkedList<>();
        List<VehicleActuator> actuatorCompareList = new LinkedList<>();
        List<Bus> busCompareList = new LinkedList<>();
        List<EEComponent> bridgeCompareList = new LinkedList<>();
        List<EEComponent> busOneCompareList = new LinkedList<>();
        List<EEComponent> busTwoCompareList = new LinkedList<>();
        List<EEComponent> busThreeCompareList = new LinkedList<>();

        FlexRay busOne = new FlexRay(eeSimulator);
        FlexRay busTwo = new FlexRay(eeSimulator);
        InstantBus busThree = new InstantBus(eeSimulator);
        buses.add(busOne);
        buses.add(busTwo);
        buses.add(busThree);
        busCompareList.add(busOne);
        busCompareList.add(busTwo);
        busCompareList.add(busThree);
        
        VehicleActuator actuator1 = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, busOne);
        VehicleActuator actuator2 = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE, busOne);
        VehicleActuator actuator3 = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, busOne);
        VehicleActuator clutch = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_CLUTCH, busOne);

        VehicleActuator actuator4 = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, busTwo);
        VehicleActuator actuator5 = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, busTwo);
        VehicleActuator motor = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR, busTwo);
        eeVehicleBuilder.createSensor(BusEntry.SENSOR_CAMERA, busTwo);
        eeVehicleBuilder.createSensor(BusEntry.SENSOR_GPS_COORDINATES, busTwo);

        VehicleActuator actuator6 = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, busThree);
        VehicleActuator gear = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_GEAR, busThree);
        VehicleActuator throttle = eeVehicleBuilder.createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE, busThree);
        eeVehicleBuilder.createSensor(BusEntry.SENSOR_OBSTACLE, busThree);

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

        //bus one
        busOneCompareList.add(actuator1);
        busOneCompareList.add(actuator2);
        busOneCompareList.add(actuator3);
        busOneCompareList.add(clutch);
        //bus two
        busTwoCompareList.add(actuator4);
        busTwoCompareList.add(actuator5);
        busTwoCompareList.add(motor);

        //bus three
        busThreeCompareList.add(actuator6);
        busThreeCompareList.add(gear);
        busThreeCompareList.add(throttle);



        Bridge bridgeOneTwo = eeVehicleBuilder.connectBuses(busOne, busTwo);
        Bridge bridgeTwoThree = eeVehicleBuilder.connectBuses(busTwo, busThree);
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
		Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
		EEVehicle eeVehicle = vehicle.getEEVehicle();


        //tests
        assertEquals(new HashSet<>(busCompareList), new HashSet<>(eeVehicle.getBusList()));
        Set<VehicleActuator> symDiffAct = Sets.symmetricDifference(new HashSet<>(actuatorCompareList), new HashSet<>(eeVehicle.getActuatorList()));
        assertEquals(0, symDiffAct.size());
        assertEquals(new HashSet<>(actuatorCompareList), new HashSet<>(eeVehicle.getActuatorList()));
        assertEquals(new HashSet<>(bridgeCompareList), new HashSet<>(eeVehicle.getBridgeList()));
        assertEquals(3, eeVehicle.getSensorList().size());
        AbstractSensor cam = null;
        AbstractSensor location = null;
        AbstractSensor obstacle = null;
        for(AbstractSensor sensor : eeVehicle.getSensorList()) {
        	if(sensor.getType() == BusEntry.SENSOR_CAMERA) {
        		cam = sensor;
        	}
        	else if(sensor.getType() == BusEntry.SENSOR_GPS_COORDINATES) {
        		location = sensor;
        	}
        	else if(sensor.getType() == BusEntry.SENSOR_OBSTACLE) {
        		obstacle = sensor;
        	}
        }
        Assert.assertNotNull(cam);
        Assert.assertNotNull(location);
        Assert.assertNotNull(obstacle);
        
        assertEquals(new HashSet<>(busOneCompareList), new HashSet<>(busOne.getConnectedComponents()));
        
        Set<EEComponent> symDiffComps = Sets.symmetricDifference(new HashSet<>(busTwoCompareList), new HashSet<>(busTwo.getConnectedComponents()));
        assertEquals(2, symDiffComps.size());
        Assert.assertTrue(symDiffComps.contains(cam));
        Assert.assertTrue(symDiffComps.contains(location));
        
        symDiffComps = Sets.symmetricDifference(new HashSet<>(busThreeCompareList), new HashSet<>(busThree.getConnectedComponents()));
        assertEquals(1, symDiffComps.size());
        Assert.assertTrue(symDiffComps.contains(obstacle));
        
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

}
