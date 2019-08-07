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
package sensors.util;

import commons.controller.commons.BusEntry;
import sensors.factory.SensorFactory;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.vehicle.PhysicalVehicle;

import java.time.Instant;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

/**
 * Created by Aklima Zaman on 2/15/2017.
 */
public class SensorUtil {
    public static PhysicalVehicle sensorAdder(PhysicalVehicle physicalVehicle) {
        EESimulator simulator = new EESimulator(Instant.EPOCH);
        SensorFactory sensorFactory = new SensorFactory(physicalVehicle, simulator);
        HashMap<BusEntry, List<EEComponent>> map = new HashMap<>();

        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_VELOCITY, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_GPS_COORDINATES, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_DISTANCE_TO_RIGHT, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_DISTANCE_TO_LEFT, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_STEERING, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_WEATHER, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_CAMERA, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_COMPASS, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_STREETTYPE, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_DAYNIGHT, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_LEFT_FRONT_DISTANCE, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_RIGHT_FRONT_DISTANCE, Collections.emptyList(), map));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_OBSTACLE, Collections.emptyList(), map));

        return physicalVehicle;
    }
}