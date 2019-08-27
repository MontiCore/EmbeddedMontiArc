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

import commons.controller.commons.BusEntry;
import commons.simulation.Sensor;
import org.junit.*;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.bus.Bus;
import simulation.bus.FlexRay;
import simulation.bus.InstantBus;

import java.io.File;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static junit.framework.TestCase.assertEquals;

public class EEVehicleTest {


    @Test
    public void testConstructor() throws IOException {
        //set up all needed classes
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        MassPointPhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = builder.createPhysicalVehicle();

        List<Bus> busList = new LinkedList<>();
        List<EEComponent> componentList = new LinkedList<>();
        Set<Sensor> sensorCompareSet = new HashSet<>();
        Set<VehicleActuator> actuatorCompareSet = new HashSet<>();
        Set<Bus> busCompareSet = new HashSet<>();
        Set<Bridge> bridgeCompareSet = new HashSet<>();
        Set<EEComponent> busOneCompareSet = new HashSet<>();
        Set<EEComponent> busTwoCompareSet = new HashSet<>();
        Set<EEComponent> busThreeCompareSet = new HashSet<>();



        FlexRay busOne = new FlexRay(simulator);
        FlexRay busTwo = new FlexRay(simulator);
        InstantBus busThree = new InstantBus(simulator);
        busList.add(busOne);
        busList.add(busTwo);
        busList.add(busThree);
        busCompareSet.add(busOne);
        busCompareSet.add(busTwo);
        busCompareSet.add(busThree);

        HashMap<BusEntry, List<EEComponent>> targetByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetByMessageId.put(BusEntry.ACTUATOR_BRAKE_CURRENT, Collections.singletonList(busOne));
        VehicleActuator actuator1 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT,0,1,4,simulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetByMessageId);
        VehicleActuator actuator2 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE, 1, 10, 6, simulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetByMessageId);
        VehicleActuator actuator3 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, 2, 8, 2, simulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetByMessageId);
        targetByMessageId.clear();
        targetByMessageId.put(BusEntry.ACTUATOR_BRAKE_CURRENT, Collections.singletonList(busTwo));
        VehicleActuator actuator4 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, 4, 9, 6, simulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetByMessageId);
        VehicleActuator actuator5 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, 12, 16, 4, simulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetByMessageId);
        targetByMessageId.clear();
        targetByMessageId.put(BusEntry.ACTUATOR_STEERING_CURRENT, Collections.singletonList(busThree));
        VehicleActuator actuator6 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, 4, 5, 1, simulator, Collections.singletonList(BusEntry.ACTUATOR_STEERING), targetByMessageId);
        componentList.add(actuator1);
        componentList.add(actuator2);
        componentList.add(actuator3);
        componentList.add(actuator4);
        componentList.add(actuator5);
        componentList.add(actuator6);
        actuatorCompareSet.add(actuator1);
        actuatorCompareSet.add(actuator2);
        actuatorCompareSet.add(actuator3);
        actuatorCompareSet.add(actuator4);
        actuatorCompareSet.add(actuator5);
        actuatorCompareSet.add(actuator6);
        busOneCompareSet.add(actuator1);
        busOneCompareSet.add(actuator2);
        busOneCompareSet.add(actuator3);
        busTwoCompareSet.add(actuator4);
        busTwoCompareSet.add(actuator5);
        busThreeCompareSet.add(actuator6);



        Bridge bridgeOneTwo = new Bridge(simulator, org.apache.commons.lang3.tuple.Pair.of(busOne, busTwo), Duration.ofNanos(1));
        Bridge bridgeTwoThree = new Bridge(simulator, org.apache.commons.lang3.tuple.Pair.of(busTwo, busThree), Duration.ofNanos(4));
        componentList.add(bridgeOneTwo);
        componentList.add(bridgeTwoThree);
        bridgeCompareSet.add(bridgeOneTwo);
        bridgeCompareSet.add(bridgeTwoThree);
        busOneCompareSet.add(bridgeOneTwo);
        busTwoCompareSet.add(bridgeOneTwo);
        busTwoCompareSet.add(bridgeTwoThree);
        busThreeCompareSet.add(bridgeTwoThree);


        System.out.println("Test create EEVehicle by using Lists");
        //set up EEVehicle
        EEVehicle eeVehicle = new EEVehicle(simulator, busList, componentList);

        //tests
        assertEquals(busCompareSet, new HashSet<>(eeVehicle.getBusList()));
        assertEquals(actuatorCompareSet, new HashSet<>(eeVehicle.getActuatorList()));
        assertEquals(bridgeCompareSet, new HashSet<>(eeVehicle.getBridgeList()));
        assertEquals(busOneCompareSet, new HashSet<>(busOne.getConnectedComponents()));
        assertEquals(busTwoCompareSet, new HashSet<>(busTwo.getConnectedComponents()));
        assertEquals(busThreeCompareSet, new HashSet<>(busThree.getConnectedComponents()));



        System.out.println("Test create EEVehicle by using JSON File");
        //store and load from JSON file
        File file = new File("C:/Users/Freddy/Desktop/SWP/EEVehicle Testordner/test.txt");
        eeVehicle.storeInFile(file, eeVehicle);

        EEVehicle eeVehicleJSON = new EEVehicle(simulator, physicalVehicle, file);

        //tests
        assertEquals(busCompareSet, new HashSet<>(eeVehicleJSON.getBusList()));
        assertEquals(actuatorCompareSet, new HashSet<>(eeVehicleJSON.getActuatorList()));
        assertEquals(bridgeCompareSet, new HashSet<>(eeVehicleJSON.getBridgeList()));
        assertEquals(busOneCompareSet, new HashSet<>(busOne.getConnectedComponents()));
        assertEquals(busTwoCompareSet, new HashSet<>(busTwo.getConnectedComponents()));
        assertEquals(busThreeCompareSet, new HashSet<>(busThree.getConnectedComponents()));

    }
}
