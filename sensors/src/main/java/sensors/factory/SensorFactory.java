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
package sensors.factory;

import commons.controller.commons.BusEntry;
import commons.simulation.Sensor;
import sensors.*;
import simulation.EESimulator.EESimulator;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Aklima Zaman on 2/8/2017.
 */
public class SensorFactory {
    private PhysicalVehicle physicalVehicle;
    private EESimulator simulator;

    public SensorFactory(PhysicalVehicle physicalVehicle, EESimulator simulator) {
        this.physicalVehicle = physicalVehicle;
        this.simulator = simulator;
    }

    public Sensor getSensor(BusEntry busEntry) {
        switch (busEntry) {
        case SENSOR_VELOCITY:
            return new SpeedSensor(this.physicalVehicle, simulator);
        case SENSOR_GPS_COORDINATES:
            return new LocationSensor(this.physicalVehicle, simulator);
        case SENSOR_STEERING:
            return new SteeringAngleSensor(this.physicalVehicle, simulator);
        case SENSOR_DISTANCE_TO_RIGHT:
            return new DistanceToRightSensor(this.physicalVehicle, simulator);
        case SENSOR_DISTANCE_TO_LEFT:
            return new DistanceToLeftSensor(this.physicalVehicle, simulator);
        case SENSOR_WEATHER:
            return new WeatherSensor(this.physicalVehicle, simulator);
        case SENSOR_CAMERA:
            return new CameraSensor(this.physicalVehicle, simulator);
        case SENSOR_COMPASS:
            return new CompassSensor(this.physicalVehicle, simulator);
        case SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new LeftBackWheelDistanceToStreetSensor(this.physicalVehicle, simulator);
        case SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new LeftFrontWheelDistanceToStreetSensor(this.physicalVehicle, simulator);
        case SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new RightFrontWheelDistanceToStreetSensor(this.physicalVehicle, simulator);
        case SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new RightBackWheelDistanceToStreetSensor(this.physicalVehicle, simulator);
        case SENSOR_STREETTYPE:
            return new StreetTypeSensor(this.physicalVehicle, simulator);
        case SENSOR_DAYNIGHT:
            return new DayNightSensor(this.physicalVehicle, simulator);
        case SENSOR_LEFT_FRONT_DISTANCE:
            return new LeftFrontDistanceSensor(this.physicalVehicle, simulator);
        case SENSOR_RIGHT_FRONT_DISTANCE:
            return new RightFrontDistanceSensor(this.physicalVehicle, simulator);
        case SENSOR_OBSTACLE:
            return new ObstacleSensor(this.physicalVehicle, simulator);
        default:
            break;
        }
        return null;
    }
}