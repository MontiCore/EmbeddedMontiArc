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

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import sensors.factory.SensorFactory;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Aklima Zaman on 2/15/2017.
 */
public class SensorUtil {
    public static PhysicalVehicle sensorAdder(PhysicalVehicle physicalVehicle) {
        SensorFactory sensorFactory = new SensorFactory(physicalVehicle);

        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_VELOCITY));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_GPS_COORDINATES));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_DISTANCE_TO_RIGHT));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_DISTANCE_TO_LEFT));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_STEERING));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_WEATHER));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_CAMERA));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_COMPASS));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_STREETTYPE));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_DAYNIGHT));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_LEFT_FRONT_DISTANCE));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_RIGHT_FRONT_DISTANCE));
        physicalVehicle.getSimulationVehicle().addSensor(sensorFactory.getSensor(BusEntry.SENSOR_OBSTACLE));

        return physicalVehicle;
    }
}