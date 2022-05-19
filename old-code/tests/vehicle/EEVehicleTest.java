/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.controller.commons.BusEntry;
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
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EESimulator;
import simulation.bus.Bus;
import simulation.bus.BusMessageEvent;
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
        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
        EEVehicle eeVehicle = vehicle.getEEVehicle();
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
        assertEquals(sim.getEventList().size(), EEVehicle.constantMessages.size() - 1 + 2 + 3);
    }


    @Test
    public void testMessageToActuator() {
        //create vehicle
        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        EEVehicle eeVehicle = vehicle.getEEVehicle();

        HashMap<BusEntry, List<EEComponent>> emptyMap = new HashMap<BusEntry, List<EEComponent>>();
        //create reference actuator
        VehicleActuator steeringReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MIN, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_RATE, vehicle.getEEVehicle().getEESimulator(), Collections.emptyList(), emptyMap);
        VehicleActuator brakeReference = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE, vehicle.getEEVehicle().getEESimulator(), Collections.emptyList(), emptyMap);
        VehicleActuator steering = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).get();
        VehicleActuator brake = eeVehicle.getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).get();

        //set to same inital value
        steeringReference.setActuatorValueCurrent(0);
        brakeReference.setActuatorValueCurrent(0);
        steering.setActuatorValueCurrent(0);
        brake.setActuatorValueCurrent(0);

        //create messages
        BusMessageEvent messageSteering = new BusMessageEvent(4.0d, 6, BusEntry.ACTUATOR_STEERING, Instant.EPOCH.plusMillis(1), UUID.randomUUID(), vehicle.getEEVehicle().getBusList().get(0));
        BusMessageEvent messageBrake = new BusMessageEvent(3.0d, 6, BusEntry.ACTUATOR_BRAKE, Instant.EPOCH.plusMillis(2), UUID.randomUUID(), vehicle.getEEVehicle().getBusList().get(0));
        eeSimulator.addEvent(messageSteering);
        eeSimulator.addEvent(messageBrake);

        //set reference actuator
        steeringReference.setActuatorValueTarget(4);
        brakeReference.setActuatorValueTarget(3);
        //steering event arrives at 1 ms at actual actuator (33-1 = 32)
        steeringReference.update(Instant.EPOCH.plusMillis(32));
        //brake event arrives at 2 ms at actual actuator (33-2 = 31)
        brakeReference.update(Instant.EPOCH.plusMillis(31));

        //loop for 33 milliseconds
        vehicle.executeLoopIteration(Duration.ofMillis(33));

        //test if value of actuator are correct
        assertEquals(steeringReference.getActuatorValueCurrent(), vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING).get().getActuatorValueCurrent());
        assertEquals(brakeReference.getActuatorValueCurrent(), vehicle.getEEVehicle().getActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT).get().getActuatorValueCurrent());

    }

    @Test
    public void notifySensorsTest() {
        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        EEVehicle eeVehicle = vehicle.getEEVehicle();

        eeVehicle.notifySensors(Instant.EPOCH.plusMillis(30));
        List<EEDiscreteEvent> events = new ArrayList<EEDiscreteEvent>(eeVehicle.getEESimulator().getEventList());
        assertEquals(eeVehicle.getSensorList().size(), events.size());
        for (EEDiscreteEvent event : events) {
            assertEquals(eeVehicle.getEESimulator().getSimulationTime(), event.getEventTime());
        }
    }

    @Test
    public void notifyActuatorsTest() {
        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
        EEVehicle eeVehicle = vehicle.getEEVehicle();

        Instant endTime = eeVehicle.getEESimulator().getSimulationTime().plusMillis(30);
        eeVehicle.notifyActuator(endTime);
        List<EEDiscreteEvent> events = new ArrayList<EEDiscreteEvent>(eeVehicle.getEESimulator().getEventList());
        assertEquals(eeVehicle.getActuatorList().size(), events.size());
        for (EEDiscreteEvent event : events) {
            assertEquals(endTime, event.getEventTime());
        }
    }

}
