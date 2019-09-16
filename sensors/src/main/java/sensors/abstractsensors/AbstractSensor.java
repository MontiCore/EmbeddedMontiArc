/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package sensors.abstractsensors;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IPhysicalVehicle;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.Sensor;
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
}
