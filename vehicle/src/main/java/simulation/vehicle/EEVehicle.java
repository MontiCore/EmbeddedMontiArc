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
package simulation.vehicle;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.gson.Gson;
import commons.controller.commons.BusEntry;

import org.apache.commons.lang3.tuple.Pair;

import sensors.CameraSensor;
import sensors.CompassSensor;
import sensors.DayNightSensor;
import sensors.DistanceToLeftSensor;
import sensors.DistanceToRightSensor;
import sensors.LeftBackWheelDistanceToStreetSensor;
import sensors.LeftFrontDistanceSensor;
import sensors.LeftFrontWheelDistanceToStreetSensor;
import sensors.LocationSensor;
import sensors.ObstacleSensor;
import sensors.RightBackWheelDistanceToStreetSensor;
import sensors.RightFrontDistanceSensor;
import sensors.RightFrontWheelDistanceToStreetSensor;
import sensors.SpeedSensor;
import sensors.SteeringAngleSensor;
import sensors.StreetTypeSensor;
import sensors.WeatherSensor;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.*;
import simulation.bus.Bus;
import simulation.bus.BusMessage;
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

	// private AutoPilot autoPilot;

	private List<Bus> busList = new LinkedList<>();

	private List<AbstractSensor> sensorList = new LinkedList<>();

	private List<VehicleActuator> actuatorList = new LinkedList<>();

	private List<Bridge> bridgeList = new LinkedList<>();

	private boolean collision = false;

	/*
	 * TODO: - maybe add Autopilot
	 */

	/**
	 * Constructor of EEVehicle
	 * 
	 * @param eeSimulator  simulator of the Vehicle
	 * @param busStructure JSON file that describes the bus structures
	 */
	public EEVehicle(Vehicle vehicle, EESimulator eeSimulator, File busStructure) {
		this.vehicle = vehicle;
		// set EESimulator
		this.eeSimulator = eeSimulator;

		// set bus, sensor, actuator and bridge lists
		List<Pair<EEComponent, UUID>> busSystems = new LinkedList<>();
		try {
			busSystems = loadFromFile(busStructure);
		} catch (IOException e) {
			e.printStackTrace();
		}

		for (Pair<EEComponent, UUID> pair : busSystems) {
			EEComponent comp = pair.getLeft();
			switch (comp.getComponentType()) {
			case BUS:
				if (!busList.contains(comp)) {
					busList.add((Bus) comp);
				}
				break;
			case SENSOR:
				if (!sensorList.contains(comp)) {
					sensorList.add((AbstractSensor) comp);
				}
				break;
			case ACTUATOR:
				if (!actuatorList.contains(comp)) {
					actuatorList.add((VehicleActuator) comp);
				}
				break;
			case BRIDGE:
				if (!bridgeList.contains(comp)) {
					bridgeList.add((Bridge) comp);
				}
				break;
			case AUTOPILOT:
//                    if (autoPilot == ObjectUtils.Null) {
//                        break;
//                    }
//                    autoPilot = comp;
			default:
				// exception?
			}

		}
	}

	/**
	 * Constructor if no JSON file is available
	 * 
	 * @param eeSimulator  simulator of the vehicle
	 * @param busStructure bus structure. Each key is one bus on the car. The value
	 *                     for each key is a list of connected EEComponents to this
	 *                     bus/key. Bridges between two buses have to be in at least
	 *                     one of the lists of connected components of the two
	 *                     buses.
	 */
	public EEVehicle(Vehicle vehicle, EESimulator eeSimulator, Map<Bus, List<EEComponent>> busStructure) {
		this.vehicle = vehicle;
		// set EESimulator
		this.eeSimulator = eeSimulator;

		// set bus, sensor, actuator and bridge lists

		for (Bus bus : busStructure.keySet()) {
			busList.add(bus);
			for (EEComponent comp : busStructure.get(bus)) {
				switch (comp.getComponentType()) {
				case BUS:
					if (!busList.contains(comp)) {
						busList.add((Bus) comp);
					}
					break;
				case SENSOR:
					if (!sensorList.contains(comp)) {
						this.addSensor((AbstractSensor) comp);
					}
					break;
				case ACTUATOR:
					if (!actuatorList.contains(comp)) {
						this.addActuator((VehicleActuator) comp);
					}
					break;
				case BRIDGE:
					if (!bridgeList.contains(comp)) {
						bridgeList.add((Bridge) comp);
					}
					break;
				case AUTOPILOT:
//                    if (autoPilot == ObjectUtils.Null) {
//                        break;
//                    }
//                    autoPilot = comp;
				default:
					// exception?
				}

			}
		}
	}

	public EEVehicle(Vehicle vehicle,EESimulator eeSimulator, List<Bus> buses, List<EEComponent> components) {
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
			case AUTOPILOT:
				break;
			default:
				throw new IllegalStateException(
						"Invalid component type. Component type was: " + component.getComponentType());
			}
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

	public void executeLoopIteration(Instant time) {
		this.notifySensors(time);
		this.eeSimulator.simulateNextTick(time);
		this.notifyActuator(time);
	}

	/**
	 * function that notifies all sensors to send their actual data to the bus
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
	// TODO: wert des updates nach update auf den bus schreiben
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

	public List<Bridge> getBridgeList() {
		return bridgeList;
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
	private void addSensor(AbstractSensor sensor) {
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

	public List<Pair<EEComponent, UUID>> loadFromFile(File file) throws IOException {

		String jsonContents = new String(Files.readAllBytes(file.toPath()));
		Gson g = new Gson();
		ParsableBusStructureProperties data = g.fromJson(jsonContents, ParsableBusStructureProperties.class);

		return data.getBusSystems();

	}

	public void storeInFile(File whereToStore, EEVehicle vehicle) throws IOException {

		ParsableBusStructureProperties properties = new ParsableBusStructureProperties(vehicle);

		Gson g = new Gson();
		String json = g.toJson(properties, ParsableBusStructureProperties.class);

		FileWriter fileWriter = new FileWriter(whereToStore, false);

		fileWriter.write(json);
		fileWriter.flush();
		fileWriter.close();
	}

	public Optional<AbstractSensor> getSensorByType(BusEntry type) {
		Predicate<EEComponent> IsSensorType = new Predicate<EEComponent>() {
			public boolean apply(EEComponent comp) {
				return comp.getTargetsByMessageId().containsKey(type);
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

		Predicate<EEComponent> containsOne = new Predicate<EEComponent>() {
			public boolean apply(EEComponent comp) {
				return comp.getComponentType() == EEComponentType.BRIDGE;
			}
		};
		for (EEComponent component : components) {
			List<BusEntry> subscribedConstMsgs = this.getSubscribedConstantMessages(component);
			if (!subscribedConstMsgs.isEmpty()) {
				Set<Bus> connectedBuses = BusUtils.findConnectedBuses(component);
				if (connectedBuses.isEmpty()) {
					throw new IllegalStateException(
							"Component that requires bus connection not connected to bus. Component was : "
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

/**
 * all data of a bus structure, including all connected actuators, buses and
 * sensors
 */
class ParsableBusStructureProperties {

	private List<Pair<EEComponent, UUID>> busSystems = new LinkedList<>();

	public ParsableBusStructureProperties(EEVehicle vehicle) {

		for (Bus bus : vehicle.getBusList()) {
			busSystems.add(Pair.of(bus, bus.getId()));
			UUID actualId = bus.getId();
			// add all sensors connected to this bus
			for (AbstractSensor sensor : vehicle.getSensorList()) {
				if (!bus.getConnectedComponents().contains(Pair.of(sensor, actualId))) {
					busSystems.add(Pair.of(sensor, actualId));
				}
			}
			// add all actuator connected to this bus
			for (VehicleActuator actuator : vehicle.getActuatorList()) {
				if (!bus.getConnectedComponents().contains(Pair.of(actuator, actualId))) {
					busSystems.add(Pair.of(actuator, actualId));
				}
			}

			// add all bridges connected to this bus
			for (Bridge bridge : vehicle.getBridgeList()) {
				if (!bus.getConnectedComponents().contains(Pair.of(bridge, actualId))) {
					busSystems.add(Pair.of(bridge, actualId));
				}
			}

		}

	}

	public List<Pair<EEComponent, UUID>> getBusSystems() {
		return busSystems;
	}

}
