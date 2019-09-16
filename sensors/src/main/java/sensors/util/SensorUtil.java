/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors.util;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IPhysicalVehicle;
import sensors.abstractsensors.AbstractSensor;
import sensors.factory.SensorFactory;
import simulation.bus.Bus;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Created by Aklima Zaman on 2/15/2017.
 */
public class SensorUtil {
    public static List<AbstractSensor>  sensorAdder(IPhysicalVehicle physicalVehicle, Bus bus) {
        return sensorAdder(physicalVehicle, Collections.singletonList(bus));
    }

    public static List<AbstractSensor>  sensorAdder(IPhysicalVehicle physicalVehicle, List<Bus> buses) {
        SensorFactory sensorFactory = new SensorFactory(physicalVehicle, buses);
        List<AbstractSensor> sensors = new ArrayList<>();

        sensorFactory.createSensor(BusEntry.SENSOR_VELOCITY).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_GPS_COORDINATES).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_DISTANCE_TO_RIGHT).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_DISTANCE_TO_LEFT).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_STEERING).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_WEATHER).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_CAMERA).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_COMPASS).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_STREETTYPE).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_DAYNIGHT).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_LEFT_FRONT_DISTANCE).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_RIGHT_FRONT_DISTANCE).ifPresent(s -> sensors.add(s));
        sensorFactory.createSensor(BusEntry.SENSOR_OBSTACLE).ifPresent(s -> sensors.add(s));

        return sensors;
    }


}
