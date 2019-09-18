/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors.factory;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IPhysicalVehicle;
import org.jfree.util.Log;
import sensors.*;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.bus.Bus;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.*;

/**
 * Created by Aklima Zaman on 2/8/2017.
 */
public class  SensorFactory {
    private IPhysicalVehicle physicalVehicle;

    private List<Bus> buses;

    public SensorFactory(IPhysicalVehicle physicalVehicle, List<Bus> buses) {
        this.physicalVehicle = physicalVehicle;
        this.buses = buses;
    }

    public SensorFactory(IPhysicalVehicle physicalVehicle, Bus bus) {
        this.physicalVehicle = physicalVehicle;
        this.buses = Collections.singletonList(bus);
    }

    public  Optional<AbstractSensor> createSensor(BusEntry busEntry) {
        switch (busEntry) {
        case SENSOR_VELOCITY:
            return createSensor(SpeedSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_GPS_COORDINATES:
            return createSensor( LocationSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_STEERING:
            return createSensor(SteeringAngleSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_DISTANCE_TO_RIGHT:
            return createSensor(DistanceToRightSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_DISTANCE_TO_LEFT:
            return createSensor(DistanceToLeftSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_WEATHER:
            return createSensor(WeatherSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_CAMERA:
            return createSensor(CameraSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_COMPASS:
            return createSensor(CompassSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return createSensor(LeftBackWheelDistanceToStreetSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return createSensor(LeftFrontDistanceSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return createSensor(RightFrontDistanceSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
            return createSensor(RightBackWheelDistanceToStreetSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_STREETTYPE:
            return createSensor(StreetTypeSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_DAYNIGHT:
            return createSensor(DayNightSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_LEFT_FRONT_DISTANCE:
            return createSensor(LeftFrontDistanceSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_RIGHT_FRONT_DISTANCE:
            return createSensor(RightFrontDistanceSensor.class, this.physicalVehicle, this.buses);
        case SENSOR_OBSTACLE:
            return createSensor(ObstacleSensor.class, this.physicalVehicle, this.buses);
        default:
            break;
        }
        return Optional.empty();
    }

    /**
     * Creates a sensor. The simulator for the sensor is taken from the first bus in buses.
     * SubscribedMessages is empty.
     * TargetsByMessageId is inferred from buses and sensorClass.
     *
     * @param sensorClass the class of the sensor that should be created
     * @param physicalVehicle the physicalVehicle the created sensor belongs to
     * @param buses the buses that the sensor is connected to
     */
    public static Optional<AbstractSensor> createSensor(Class<? extends AbstractSensor> sensorClass, IPhysicalVehicle physicalVehicle, List<Bus> buses) {
        if(buses == null || buses.isEmpty()) {
            throw new IllegalArgumentException("Buses can not be null or empty");
        }
        try {
            Method getSensorType = sensorClass.getMethod("getSensorType");
            BusEntry sensorType = (BusEntry) getSensorType.invoke(null);
            List<BusEntry> subscribedMessages = Collections.emptyList();
            HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
            targetsByMessageId.put(sensorType, new ArrayList<EEComponent>(buses));
            if(sensorType.equals(CameraSensor.getSensorType()) || sensorType.equals(StaticPlannedTrajectoryXSensor.getSensorType())
                    || sensorType.equals(StaticPlannedTrajectoryYSensor.getSensorType())) {
                Constructor<? extends AbstractSensor> constructor = sensorClass.getConstructor(IPhysicalVehicle.class, EESimulator.class, List.class, HashMap.class, List.class);
                return Optional.of(constructor.newInstance(physicalVehicle, buses.get(0).getSimulator(), subscribedMessages, targetsByMessageId, Collections.emptyList()));
            }
            else {
                Constructor<? extends AbstractSensor> constructor = sensorClass.getConstructor(IPhysicalVehicle.class, EESimulator.class, List.class, HashMap.class);
                return Optional.of(constructor.newInstance(physicalVehicle, buses.get(0).getSimulator(), subscribedMessages, targetsByMessageId));
            }
        } catch (NoSuchMethodException | SecurityException | IllegalAccessException | IllegalArgumentException | InvocationTargetException | InstantiationException e) {
            Log.error("Failed to create sensor " + sensorClass);
            e.printStackTrace();
        }
        return Optional.empty();
    }


    /**
     * Creates a sensor. The simulator for the sensor is taken from bus.
     * SubscribedMessages is empty.
     * TargetsByMessageId is inferred from bus and sensorClass.
     *
     * @param sensorClass the class of the sensor that should be created
     * @param physicalVehicle the physicalVehicle the created sensor belongs to
     * @param bus the bus that the sensor is connected to
     */
    public static Optional<AbstractSensor> createSensor(Class<? extends AbstractSensor> sensorClass, IPhysicalVehicle physicalVehicle, Bus bus) {
        return createSensor(sensorClass, physicalVehicle, Collections.singletonList(bus));
    }
}
