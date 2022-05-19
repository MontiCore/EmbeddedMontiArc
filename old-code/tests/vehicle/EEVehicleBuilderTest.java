/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import static junit.framework.TestCase.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.time.Instant;
import java.util.*;

import org.junit.Assert;
import org.junit.Test;

import com.google.common.collect.Sets;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import sensors.CameraSensor;
import sensors.LocationSensor;
import sensors.ObstacleSensor;
import sensors.abstractsensors.AbstractSensor;
import sensors.factory.SensorFactory;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEComponentType;
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
        List<AbstractSensor> sensorCompoareList = new LinkedList<>();
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


        //set up EEVehicle
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        EEVehicle eeVehicle = vehicle.getEEVehicle();

        //search and collect all created bridges
        HashSet<Bridge> createdBridges = new HashSet<>();
        for (Bus bus : eeVehicle.getBusList()) {
            for (EEComponent comp : bus.getConnectedComponents()) {
                if (comp.getComponentType() == EEComponentType.BRIDGE) {
                    createdBridges.add((Bridge) comp);
                }
            }
        }

        //tests
        assertEquals(new HashSet<>(busCompareList), new HashSet<>(eeVehicle.getBusList()));
        Set<VehicleActuator> symDiffAct = Sets.symmetricDifference(new HashSet<>(actuatorCompareList), new HashSet<>(eeVehicle.getActuatorList()));
        assertEquals(0, symDiffAct.size());
        assertEquals(new HashSet<>(actuatorCompareList), new HashSet<>(eeVehicle.getActuatorList()));
        assertEquals(new HashSet<>(bridgeCompareList), createdBridges);
        assertEquals(3, eeVehicle.getSensorList().size());
        AbstractSensor cam = null;
        AbstractSensor location = null;
        AbstractSensor obstacle = null;
        for (AbstractSensor sensor : eeVehicle.getSensorList()) {
            if (sensor.getType() == BusEntry.SENSOR_CAMERA) {
                cam = sensor;
            } else if (sensor.getType() == BusEntry.SENSOR_GPS_COORDINATES) {
                location = sensor;
            } else if (sensor.getType() == BusEntry.SENSOR_OBSTACLE) {
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

        // //store and load from JSON file
        // File file = new File("ee_serialize.json");
        // file.createNewFile();
        // eeVehicleBuilder.storeInFile(file, vehicle.getEEVehicle());

        // Vehicle vehicleJSON = new Vehicle(new MassPointPhysicalVehicleBuilder(), eeVehicleBuilder, file);

        //search and collect all created bridges
        HashSet<Bridge> createdBridgesJSON = new HashSet<>();
        List<UUID> processedBridges = new LinkedList<>();
        for (Bus bus : eeVehicle.getBusList()) {
            for (EEComponent comp : bus.getConnectedComponents()) {
                if (comp.getComponentType() == EEComponentType.BRIDGE && !createdBridgesJSON.contains(comp)) {
                    processedBridges.add(comp.getId());
                    createdBridgesJSON.add((Bridge) comp);
                }
            }
        }

        //tests
        // assertTrue(componentIsConnected(new LinkedList<EEComponent>(busCompareList), new LinkedList<EEComponent>(vehicleJSON.getEEVehicle().getBusList())));
        // assertTrue(componentIsConnected(new LinkedList<EEComponent>(actuatorCompareList), new LinkedList<EEComponent>(vehicleJSON.getEEVehicle().getActuatorList())));
        // assertTrue(correctSensors(vehicleJSON.getEEVehicle().getSensorList()));
        assertTrue(componentIsConnected(new LinkedList<EEComponent>(bridgeCompareList), new LinkedList<EEComponent>(createdBridgesJSON)));
        assertTrue(componentIsConnected(busOneCompareList, busOne.getConnectedComponents()));
        assertTrue(componentIsConnected(busTwoCompareList, busTwo.getConnectedComponents()));
        assertTrue(componentIsConnected(busThreeCompareList, busThree.getConnectedComponents()));


    }

    @Test
    public void addAllSensorsTest() {
        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensors(bus);
        eeVehicleBuilder.createAllActuators(bus);
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        EEVehicle eeVehicle = vehicle.getEEVehicle();
        Set<BusEntry> sensorBusEntries = new HashSet<>();
        sensorBusEntries.add(BusEntry.SENSOR_CAMERA);
        sensorBusEntries.add(BusEntry.SENSOR_COMPASS);
        sensorBusEntries.add(BusEntry.SENSOR_DAYNIGHT);
        sensorBusEntries.add(BusEntry.SENSOR_DISTANCE_TO_LEFT);
        sensorBusEntries.add(BusEntry.SENSOR_DISTANCE_TO_RIGHT);
        sensorBusEntries.add(BusEntry.SENSOR_GPS_COORDINATES);
        sensorBusEntries.add(BusEntry.SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR);
        sensorBusEntries.add(BusEntry.SENSOR_LEFT_FRONT_DISTANCE);
        sensorBusEntries.add(BusEntry.SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR);
        sensorBusEntries.add(BusEntry.SENSOR_OBSTACLE);
        sensorBusEntries.add(BusEntry.SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR);
        sensorBusEntries.add(BusEntry.SENSOR_RIGHT_FRONT_DISTANCE);
        sensorBusEntries.add(BusEntry.SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR);
        sensorBusEntries.add(BusEntry.SENSOR_STEERING);
        sensorBusEntries.add(BusEntry.SENSOR_STREETTYPE);
        sensorBusEntries.add(BusEntry.SENSOR_VELOCITY);
        sensorBusEntries.add(BusEntry.SENSOR_WEATHER);
        sensorBusEntries.add(BusEntry.PLANNED_TRAJECTORY_X);
        sensorBusEntries.add(BusEntry.PLANNED_TRAJECTORY_Y);

        for (BusEntry entry : sensorBusEntries) {
            Optional<AbstractSensor> sensor = eeVehicle.getSensorByType(entry);
            Assert.assertTrue(sensor.isPresent());
        }
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

    private boolean correctSensors(List<AbstractSensor> isConnected) {
        boolean notFound = false;
        for (AbstractSensor is : isConnected) {
            if (notFound) {
                return false;
            }
            notFound = true;
            if (is.getType() == BusEntry.SENSOR_CAMERA) {
                notFound = false;
            } else if (is.getType() == BusEntry.SENSOR_GPS_COORDINATES) {
                notFound = false;
            } else if (is.getType() == BusEntry.SENSOR_OBSTACLE) {
                notFound = false;
            }
        }
        return true;
    }


}
