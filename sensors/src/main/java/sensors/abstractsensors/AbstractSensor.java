/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors.abstractsensors;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.*;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.jfree.util.Log;

import commons.simulation.IPhysicalVehicle;
import commons.simulation.Sensor;
import sensors.CameraSensor;
import sensors.StaticPlannedTrajectoryXSensor;
import sensors.StaticPlannedTrajectoryYSensor;
import simulation.bus.Bus;


/**
 * Created by Aklima Zaman on 1/20/2017.
 */
public abstract class AbstractSensor extends ImmutableEEComponent implements Sensor {

	private final IPhysicalVehicle physicalVehicle;

	public AbstractSensor(IPhysicalVehicle physicalVehicle, EESimulator simulator, List<BusEntry> subscribedMessages,
			HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
		super(simulator, EEComponentType.SENSOR, subscribedMessages, targetsByMessageId);
		this.physicalVehicle = physicalVehicle;
	}

	@Override
	public void update(Instant actualTime) {
		calculateValue();
		this.sendMessage(this.getValue(), this.getDataLength(), this.getType(), actualTime);
	}

	/**
	 * Return the physicalVehicle this sensor belongs to
	 * @return
	 */
	public IPhysicalVehicle getPhysicalVehicle() {
		return physicalVehicle;
	}

	/**
	 * This method do the sensor calculations
	 */
	protected abstract void calculateValue();

	/**
	 *
	 * @return individual lenght of the data of the sensor in bytes
	 */
	public abstract int getDataLength();

	@Override
	public void processEvent(EEDiscreteEvent event) {
		// TODO implement
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
