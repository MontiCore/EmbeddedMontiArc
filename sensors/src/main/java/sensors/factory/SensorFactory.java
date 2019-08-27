/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors.factory;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.Sensor;
import sensors.*;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Aklima Zaman on 2/8/2017.
 */
public class SensorFactory {
    private PhysicalVehicle physicalVehicle;

    public SensorFactory(PhysicalVehicle physicalVehicle) {
        this.physicalVehicle = physicalVehicle;
    }

    public Sensor getSensor(BusEntry busEntry) {
        switch (busEntry) {
        case SENSOR_VELOCITY:
            return new SpeedSensor(this.physicalVehicle);
        case SENSOR_GPS_COORDINATES:
            return new LocationSensor(this.physicalVehicle);
        case SENSOR_STEERING:
            return new SteeringAngleSensor(this.physicalVehicle);
        case SENSOR_DISTANCE_TO_RIGHT:
            return new DistanceToRightSensor(this.physicalVehicle);
        case SENSOR_DISTANCE_TO_LEFT:
            return new DistanceToLeftSensor(this.physicalVehicle);
        case SENSOR_WEATHER:
            return new WeatherSensor(this.physicalVehicle);
        case SENSOR_CAMERA:
            return new CameraSensor(this.physicalVehicle);
        case SENSOR_COMPASS:
            return new CompassSensor(this.physicalVehicle);
        case SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new LeftBackWheelDistanceToStreetSensor(this.physicalVehicle);
        case SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new LeftFrontWheelDistanceToStreetSensor(this.physicalVehicle);
        case SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new RightFrontWheelDistanceToStreetSensor(this.physicalVehicle);
        case SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return new RightBackWheelDistanceToStreetSensor(this.physicalVehicle);
        case SENSOR_STREETTYPE:
            return new StreetTypeSensor(this.physicalVehicle);
        case SENSOR_DAYNIGHT:
            return new DayNightSensor(this.physicalVehicle);
        case SENSOR_LEFT_FRONT_DISTANCE:
            return new LeftFrontDistanceSensor(this.physicalVehicle);
        case SENSOR_RIGHT_FRONT_DISTANCE:
            return new RightFrontDistanceSensor(this.physicalVehicle);
        case SENSOR_OBSTACLE:
            return new ObstacleSensor(this.physicalVehicle);
        default:
            break;
        }
        return null;
    }
}
