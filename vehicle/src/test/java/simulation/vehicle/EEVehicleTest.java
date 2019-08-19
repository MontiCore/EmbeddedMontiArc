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

        HashMap<Bus, List<EEComponent>> busSystem = new HashMap<>();
        Set<Sensor> sensorCompareSet = new HashSet<>();
        Set<VehicleActuator> actuatorCompareSet = new HashSet<>();
        Set<Bus> busCompareSet = new HashSet<>();
        Set<Bridge> bridgeCompareSet = new HashSet<>();


        FlexRay busOne = new FlexRay(simulator);
        FlexRay busTwo = new FlexRay(simulator);
        InstantBus busThree = new InstantBus(simulator);
        busCompareSet.add(busOne);
        busCompareSet.add(busTwo);
        busCompareSet.add(busThree);

        VehicleActuator actuator1 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT,0,1,4,simulator, null, null);
        VehicleActuator actuator2 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE, 1, 10, 6, simulator, null, null);
        VehicleActuator actuator3 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, 2, 8, 2, simulator, null, null);
        VehicleActuator actuator4 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, 4, 9, 6, simulator, null, null);
        VehicleActuator actuator5 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, 12, 16, 4, simulator, null, null);
        VehicleActuator actuator6 = new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, 4, 5, 1, simulator, null, null);
        actuatorCompareSet.add(actuator1);
        actuatorCompareSet.add(actuator2);
        actuatorCompareSet.add(actuator3);
        actuatorCompareSet.add(actuator4);
        actuatorCompareSet.add(actuator5);
        actuatorCompareSet.add(actuator6);

        Bridge bridgeOneTwo = new Bridge(simulator, org.apache.commons.lang3.tuple.Pair.of(busOne, busTwo), Duration.ofNanos(1));
        Bridge bridgeTwoThree = new Bridge(simulator, org.apache.commons.lang3.tuple.Pair.of(busTwo, busThree), Duration.ofNanos(4));
        bridgeCompareSet.add(bridgeOneTwo);
        bridgeCompareSet.add(bridgeTwoThree);

        //set up the bus system
        List<EEComponent> connectedToBusOne = new LinkedList<>();
        connectedToBusOne.add(actuator1);
        connectedToBusOne.add(actuator2);
        connectedToBusOne.add(actuator3);
        connectedToBusOne.add(bridgeOneTwo);
        busSystem.put(busOne, connectedToBusOne);

        List<EEComponent> connectedToBusTwo = new LinkedList<>();
        connectedToBusTwo.add(actuator4);
        connectedToBusTwo.add(actuator5);
        connectedToBusTwo.add(bridgeOneTwo);
        connectedToBusTwo.add(bridgeTwoThree);
        busSystem.put(busTwo, connectedToBusTwo);

        List<EEComponent> connectedToBusThree = new LinkedList<>();
        connectedToBusThree.add(actuator6);
        connectedToBusThree.add(bridgeTwoThree);
        busSystem.put(busThree, connectedToBusThree);

        System.out.println("Test create EEVehicle by using HashMap");
        //set up EEVehicle
        EEVehicle eeVehicle = new EEVehicle(simulator, busSystem);

        //tests
        assertEquals(busCompareSet, new HashSet<>(eeVehicle.getBusList()));
        assertEquals(actuatorCompareSet, new HashSet<>(eeVehicle.getActuatorList()));
        assertEquals(bridgeCompareSet, new HashSet<>(eeVehicle.getBridgeList()));

        System.out.println("Test create EEVehicle by using JSON File");
        //store and load from JSON file
        File file = new File("C:/Users/Freddy/Desktop/SWP");
        eeVehicle.storeInFile(file, eeVehicle);

        EEVehicle eeVehicleJSON = new EEVehicle(simulator, file);

        //tests
        assertEquals(busCompareSet, new HashSet<>(eeVehicleJSON.getBusList()));
        assertEquals(actuatorCompareSet, new HashSet<>(eeVehicleJSON.getActuatorList()));
        assertEquals(bridgeCompareSet, new HashSet<>(eeVehicleJSON.getBridgeList()));

    }
}
