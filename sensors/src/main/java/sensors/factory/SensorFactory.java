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

    public Optional<AbstractSensor> createSensor(BusEntry sensorType) {
    	Optional<Class<? extends AbstractSensor>> sensorClass = getSensorClassByBusEntry(sensorType);
    	if(sensorClass.isPresent()) {
    		return createSensor(sensorClass.get(), physicalVehicle, buses);
    	}
        return Optional.empty();
    }
    
    public static Optional<AbstractSensor> createSensor(BusEntry sensorType, IPhysicalVehicle physicalVehicle, Bus bus) {
    	return createSensor(sensorType, physicalVehicle, Collections.singletonList(bus));
    }
    
    public static Optional<AbstractSensor> createSensor(BusEntry sensorType, IPhysicalVehicle physicalVehicle, List<Bus> buses) {
    	Optional<Class<? extends AbstractSensor>> sensorClass = getSensorClassByBusEntry(sensorType);
    	if(sensorClass.isPresent()) {
    		return createSensor(sensorClass.get(), physicalVehicle, buses);
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
    
    public static Optional<Class<? extends AbstractSensor>> getSensorClassByBusEntry(BusEntry busEntry){
    	Optional<Class<? extends AbstractSensor>> sensorClass = Optional.empty();
        switch(busEntry) {
    	case SENSOR_VELOCITY:
    		sensorClass = Optional.of(SpeedSensor.class);
    		break;
        case SENSOR_GPS_COORDINATES:
			sensorClass = Optional.of(LocationSensor.class);
			break;
        case SENSOR_STEERING:
			sensorClass = Optional.of(SteeringAngleSensor.class);
			break;
        case SENSOR_DISTANCE_TO_RIGHT:
			sensorClass = Optional.of(DistanceToRightSensor.class);
			break;
        case SENSOR_DISTANCE_TO_LEFT:
			sensorClass = Optional.of(DistanceToLeftSensor.class);
			break;
        case SENSOR_WEATHER:
			sensorClass = Optional.of(WeatherSensor.class);
			break;
        case SENSOR_CAMERA:
  			sensorClass = Optional.of(CameraSensor.class);
			break;
        case SENSOR_COMPASS:
			sensorClass = Optional.of(CompassSensor.class);
			break;
        case SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
			sensorClass = Optional.of(LeftBackWheelDistanceToStreetSensor.class);
			break;
        case SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
			sensorClass = Optional.of(LeftFrontWheelDistanceToStreetSensor.class);
			break;
        case SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR:
			sensorClass = Optional.of(RightFrontWheelDistanceToStreetSensor.class);
			break;
        case SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR:
			sensorClass = Optional.of(RightBackWheelDistanceToStreetSensor.class);
			break;
        case SENSOR_STREETTYPE:
			sensorClass = Optional.of(StreetTypeSensor.class);
			break;
        case SENSOR_DAYNIGHT:
			sensorClass = Optional.of(DayNightSensor.class);
			break;
        case SENSOR_LEFT_FRONT_DISTANCE:
			sensorClass = Optional.of(LeftFrontDistanceSensor.class);
			break;
        case SENSOR_RIGHT_FRONT_DISTANCE:
			sensorClass = Optional.of(RightFrontDistanceSensor.class);
			break;
        case SENSOR_OBSTACLE:
			sensorClass = Optional.of(ObstacleSensor.class);
			break;
        case PLANNED_TRAJECTORY_X:
			sensorClass = Optional.of(StaticPlannedTrajectoryXSensor.class);
			break;
        case PLANNED_TRAJECTORY_Y:
			sensorClass = Optional.of(StaticPlannedTrajectoryYSensor.class);
			break;	
        default:
            break;
        }
        return sensorClass;
    }
}
