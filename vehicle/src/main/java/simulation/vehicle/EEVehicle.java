/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.gson.Gson;


import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.Pair;

import sensors.abstractsensors.AbstractSensor;
import sensors.factory.SensorFactory;
import simulation.EESimulator.*;
import simulation.bus.Bus;
import simulation.bus.BusMessage;
import simulation.bus.FlexRay;
import simulation.bus.BusUtils;
import simulation.bus.InstantBus;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.time.Duration;
import java.time.Instant;
import java.util.*;

public class EEVehicle {

	protected static final List<BusEntry> constantMessages = new ArrayList<BusEntry>() {
		{
			add(BusEntry.CONSTANT_NUMBER_OF_GEARS);
			add(BusEntry.CONSTANT_WHEELBASE);
			add(BusEntry.CONSTANT_MAXIMUM_TOTAL_VELOCITY);
			add(BusEntry.CONSTANT_MOTOR_MAX_ACCELERATION);
			add(BusEntry.CONSTANT_MOTOR_MIN_ACCELERATION);
			add(BusEntry.CONSTANT_BRAKES_MAX_ACCELERATION);
			add(BusEntry.CONSTANT_BRAKES_MIN_ACCELERATION);
			add(BusEntry.CONSTANT_STEERING_MAX_ANGLE);
			add(BusEntry.CONSTANT_STEERING_MIN_ANGLE);
			add(BusEntry.CONSTANT_TRAJECTORY_ERROR);
		}
	};

	private final EESimulator eeSimulator;

	private final Vehicle vehicle;

	private List<Bus> busList = new LinkedList<>();

	private List<AbstractSensor> sensorList = new ArrayList<>();

	private List<VehicleActuator> actuatorList = new ArrayList<>();

	private boolean collision = false;




	public EEVehicle(Vehicle vehicle, EESimulator eeSimulator, Set<Bus> buses, List<EEComponent> components) {
		this.vehicle = vehicle;
		this.eeSimulator = eeSimulator;
		this.busList = new ArrayList<Bus>(buses);
		Map<UUID, Bus> busById = new HashMap<UUID, Bus>();
		for (Bus bus : buses) {
			busById.put(bus.getId(), bus);
		}
		for (EEComponent component : components) {
			switch (component.getComponentType()) {
				case ACTUATOR:
					this.actuatorList.add((VehicleActuator) component);
					break;
				case SENSOR:
					this.sensorList.add((AbstractSensor) component);
					break;
				case BRIDGE:
				case AUTOPILOT:
				case NAVIGATION:
					//noop
					break;
				default:
					throw new IllegalStateException("Invalid component type. Component type was: " + component.getComponentType());
			}
			if (component.getComponentType() != EEComponentType.BRIDGE	) {
				List<UUID> seenIds = new ArrayList<UUID>();
				for (List<EEComponent> targets : component.getTargetsByMessageId().values()) {
					for (EEComponent target : targets) {
						if (target.getComponentType() == EEComponentType.BUS) {
							if (!seenIds.contains(target.getId())) {
								seenIds.add(target.getId());
								((Bus) target).registerComponent(component);
							}
						}

					}
				}
			}
		}
	}

	public void executeLoopIteration(Instant time) {
		if(time.isAfter(this.eeSimulator.getSimulationTime())) {
			this.notifySensors(time);
			this.eeSimulator.simulateNextTick(time);
			this.notifyActuator(time);
		}
	}

	/**
	 * function that notifies all sensors to send their actual data to the busAndParameter
	 *
	 * @param actualTime actual time of the simulation
	 */
	public void notifySensors(Instant actualTime) {
		for (AbstractSensor sensor : sensorList) {
			sensor.update(actualTime);
		}
	}

	/**
	 * function that notifies all actuators to update
	 *
	 * @param actualTime time the actuators get to update their value
	 */
	public void notifyActuator(Instant actualTime) {
		if (!this.collision) {
			for (VehicleActuator actuator : actuatorList) {
				actuator.update(actualTime);
			}
		} else {
			for (VehicleActuator actuator : actuatorList) {
				actuator.reset();
			}
			this.collision = false;
		}
	}

	public EESimulator getEESimulator() {
		return eeSimulator;
	}

	public List<Bus> getBusList() {
		return busList;
	}

	public List<AbstractSensor> getSensorList() {
		return sensorList;
	}

	public List<VehicleActuator> getActuatorList() {
		return actuatorList;
	}

	public Optional<VehicleActuator> getActuator(VehicleActuatorType actuatorType) {
		for (VehicleActuator actuator : this.actuatorList) {
			if (actuator.getActuatorType() == actuatorType) {
				return Optional.of(actuator);
			}
		}
		return Optional.empty();
	}

	/**
	 * Add sensor to sensor list and register at target buses
	 *
	 * @param sensor to be registered
	 */
	public void addSensor(AbstractSensor sensor) {
		for (List<EEComponent> targets : sensor.getTargetsByMessageId().values()) {
			for (EEComponent target : targets) {
				if (target.getComponentType() == EEComponentType.BUS) {
					Bus bus = (Bus) target;
					bus.registerComponent(sensor);
				}
			}
		}
		this.sensorList.add(sensor);
	}

	/**
	 * Add actuator to actuator list and register at target buses
	 *
	 * @param actuator actuator to be registered
	 */
	private void addActuator(VehicleActuator actuator) {
		for (List<EEComponent> targets : actuator.getTargetsByMessageId().values()) {
			for (EEComponent target : targets) {
				if (target.getComponentType() == EEComponentType.BUS) {
					Bus bus = (Bus) target;
					bus.registerComponent(actuator);
				}
			}
		}
		this.actuatorList.add(actuator);
	}



	public Optional<AbstractSensor> getSensorByType(BusEntry type) {
		Predicate<AbstractSensor> IsSensorType = new Predicate<AbstractSensor>() {
			public boolean apply(AbstractSensor comp) {
				return comp.getType().equals(type);
			}
		};
 		return Optional.ofNullable(Iterables.find(this.sensorList, IsSensorType, null));
	}

	protected void setCollision(boolean collision) {
		this.collision = collision;
	}

	protected void setConstantBusData() {
		List<EEComponent> components = new ArrayList<EEComponent>(
				this.actuatorList.size() + this.sensorList.size() + 5);
		components.addAll(actuatorList);
		components.addAll(sensorList);

		for (EEComponent component : components) {
			List<BusEntry> subscribedConstMsgs = this.getSubscribedConstantMessages(component);
			if (!subscribedConstMsgs.isEmpty()) {
				Set<Bus> connectedBuses = BusUtils.findConnectedBuses(component);
				if (connectedBuses.isEmpty()) {
					throw new IllegalStateException(
							"Component that requires busAndParameter connection not connected to busAndParameter. Component was : "
									+ component);
				}
				Bus bus = connectedBuses.iterator().next();
				this.sendConstMsgs(subscribedConstMsgs, bus, component);
			}
		}
	}

	private void sendConstMsgs(List<BusEntry> constMsgs, Bus bus, EEComponent component) {
		PhysicalVehicle physicalVehilce = vehicle.getPhysicalVehicle();
		VehicleActuator brakes = physicalVehilce
				.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT);
		VehicleActuator motor = physicalVehilce.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
		// ModelicaPhysicalVehicle
		if (brakes == null) {
			brakes = physicalVehilce.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKE);
			motor = physicalVehilce.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_THROTTLE);
		}
		VehicleActuator steering = physicalVehilce
				.getVehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
		for(BusEntry constMsg : constMsgs) {
			Object msg;
			switch (constMsg) {
			case CONSTANT_BRAKES_MAX_ACCELERATION:
				msg = brakes.getActuatorValueMax();
				break;
			case CONSTANT_BRAKES_MIN_ACCELERATION:
				msg = brakes.getActuatorValueMin();
				break;
			case CONSTANT_MAXIMUM_TOTAL_VELOCITY:
				msg = Vehicle.VEHICLE_DEFAULT_APPROX_MAX_VELOCITY;
				break;
			case CONSTANT_MOTOR_MAX_ACCELERATION:
				msg = motor.getActuatorValueMax();
				break;
			case CONSTANT_MOTOR_MIN_ACCELERATION:
				msg = motor.getActuatorValueMin();
				break;
			case CONSTANT_NUMBER_OF_GEARS:
				msg = 1;
				break;
			case CONSTANT_STEERING_MAX_ANGLE:
				msg = steering.getActuatorValueMax();
				break;
			case CONSTANT_STEERING_MIN_ANGLE:
				msg = steering.getActuatorValueMin();
				break;
			case CONSTANT_TRAJECTORY_ERROR:
				msg = 0.0;
				break;
			case CONSTANT_WHEELBASE:
				msg = physicalVehilce.getWheelDistToFront() + physicalVehilce.getWheelDistToBack();
				break;
			default:
				throw new IllegalArgumentException("Unknown constant message: " + constMsg);
			}
			if(msg != null) {
				this.eeSimulator.addEvent(new BusMessage(msg, 8, constMsg, this.eeSimulator.getSimulationTime(), bus.getId(), component));
			}
		}
	}

	private List<BusEntry> getSubscribedConstantMessages(EEComponent component) {
		List<BusEntry> subscribedConstMsgs = new ArrayList<BusEntry>();
		for (BusEntry messageId : component.getSubscribedMessages()) {
			if (constantMessages.contains(messageId)) {
				subscribedConstMsgs.add(messageId);
			}
		}
		return subscribedConstMsgs;
	}

}


