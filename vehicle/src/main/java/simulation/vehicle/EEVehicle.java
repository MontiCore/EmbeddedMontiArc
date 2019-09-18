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

	private DirectModelAsEEComponent autoPilot;

	private List<Bus> busList = new LinkedList<>();

	private List<AbstractSensor> sensorList = new LinkedList<>();

	private List<VehicleActuator> actuatorList = new LinkedList<>();

	private List<Bridge> bridgeList = new LinkedList<>();

	private Optional<NavigationBlockAsEEComponent> navigation = Optional.empty();

	private boolean collision = false;


    /*
    TODO: create autopilot
     */

    /**
     * Constructor of EEVehicle
     * @param eeSimulator simulator of the Vehicle
     * @param data JSON file that describes the busAndParameter structures
     */
    public EEVehicle(Vehicle vehicle, EESimulator eeSimulator, File data) {

        //set EESimulator, vehicle
        this.eeSimulator = eeSimulator;

        this.vehicle = vehicle;

        //set busAndParameter, sensor, actuator and bridge lists
		ParsableBusStructureProperties busStructure;

		try {
			busStructure = loadFromFile(data);

			//create buses
			for (ParsableBusStructureProperties.Pair component : busStructure.getBuses()) {
				if (component.eeComponent.equals("flexRay")) {
					FlexRay flexRay = new FlexRay(eeSimulator);
					busList.add((int) component.busAndParameter[0], flexRay);
				} else if (component.eeComponent.equals("instantBus")) {
					InstantBus instantBus = new InstantBus(eeSimulator);
					busList.add((int) component.busAndParameter[0], instantBus);
				}
			}

			//create sensors, actuator and bridges
			for (ParsableBusStructureProperties.Pair sensor : busStructure.getSensors()) {
				try {
					Class<? extends AbstractSensor> sensorClass = (Class<? extends AbstractSensor>) Class.forName(sensor.eeComponent);
					AbstractSensor newSensor = SensorFactory.createSensor(sensorClass, vehicle.getPhysicalVehicle(), busList.get((int) sensor.busAndParameter[0])).get();
					sensorList.add(newSensor);
					busList.get((int) sensor.busAndParameter[0]).registerComponent(newSensor);
				} catch (ClassNotFoundException e) {
					throw new IllegalArgumentException("Can not create EEVehicle. File contains non-existent sensor: " + sensor.eeComponent);
				}
			}

			for (ParsableBusStructureProperties.Pair actuator : busStructure.getActuators()) {
				VehicleActuator newActuator = VehicleActuator.createVehicleActuator(VehicleActuatorType.valueOf(actuator.eeComponent), actuator.busAndParameter[1], actuator.busAndParameter[2], actuator.busAndParameter[3], busList.get((int) actuator.busAndParameter[0]));
				actuatorList.add(newActuator);
				busList.get((int) actuator.busAndParameter[0]).registerComponent(newActuator);
			}

			for (ParsableBusStructureProperties.Pair bridge : busStructure.getBridges()) {
				Bridge newBridge = new Bridge(eeSimulator, Pair.of(busList.get((int) bridge.busAndParameter[0]), busList.get((int) bridge.busAndParameter[1])), Duration.ofMillis((long) bridge.busAndParameter[2]));
				bridgeList.add(newBridge);
			}

			if (busStructure.getAutopilot() != null) {
				navigation = Optional.of(NavigationBlockAsEEComponent.createNavigationBlockAsEEComponent(busList.get((int) busStructure.getAutopilot().busAndParameter[0])));
				busList.get((int) busStructure.getAutopilot().busAndParameter[0]).registerComponent(navigation.get());
			}

		} catch (IOException e) {
			throw  new IllegalArgumentException("Can not create EEVehicle. Failed to read file: " + data);
		}

	}



	public EEVehicle(Vehicle vehicle, EESimulator eeSimulator, List<Bus> buses, List<EEComponent> components) {
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
				if(navigation.isPresent()){
					throw new IllegalStateException("Autopilot can only be set once");
				}
				autoPilot = (DirectModelAsEEComponent) component;
				break;
			case BRIDGE:
				this.bridgeList.add((Bridge) component);
				break;
				case NAVIGATION:
					if(navigation.isPresent()){
						throw new IllegalStateException("Navigation can only be set once");
					}
					navigation = Optional.of((NavigationBlockAsEEComponent) component);
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
		this.notifySensors(time);
		this.eeSimulator.simulateNextTick(time);
		this.notifyActuator(time);
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

	public DirectModelAsEEComponent getAutoPilot() {
		return this.autoPilot;
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

	public void setAutoPilot(DirectModelAsEEComponent autoPilot) {
		this.autoPilot = autoPilot;
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

	protected Optional<NavigationBlockAsEEComponent> getNavigation(){
		return navigation;
	}

	/**
	 * Function that load the ParsableBusStructure out of a JSON file
	 * @param file JSON File of the bus structure of an EEVehicle
	 * @return the bus structure as a ParsabelBusStructure
	 * @throws IOException
	 */

	public ParsableBusStructureProperties loadFromFile(File file) throws IOException {

		String jsonContents = new String(Files.readAllBytes(file.toPath()));
		Gson g = new Gson();
		ParsableBusStructureProperties data = g.fromJson(jsonContents, ParsableBusStructureProperties.class);

		return data;
	}

	/**
	 * stores the actual bus structure in JSON file
	 * @param whereToStore path where the JSON file will be stored
	 * @param vehicle vehicle which bus should be stored
	 * @throws IOException
	 */
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

/**
 * all data of a busAndParameter structure, including all connected actuators, buses,
 * sensors and autopilots.
 * Sensors are saved by their class name.
 * Actuators are saved by their actuatorType.
 * Buses are saved by their busType.
 * Autopilot is saved by String "navigation".
 */
class ParsableBusStructureProperties {

	class Pair {
		String eeComponent;
		double[] busAndParameter;

		public Pair(String eeComponent, double[] bus) {
			this.eeComponent = eeComponent;
			this.busAndParameter = bus;
		}


		public String getEeComponent() {
			return eeComponent;
		}

		public double[] getBus() {
			return busAndParameter;
		}
	}


	private List<Pair> buses = new LinkedList<>();
	private List<Pair> sensors = new LinkedList<>();
	private List<Pair> actuators = new LinkedList<>();
	private List<Pair> bridges = new LinkedList<>();
	private Pair autopilot;
	private Pair controller;

	public ParsableBusStructureProperties(EEVehicle vehicle) {

		List<Bridge> processedBridges = new LinkedList<>();

		//id to assign component to the busAndParameter and index of the busAndParameter in busList of the eeVehicle
		int busId = 0;

		for (Bus bus : vehicle.getBusList()) {
			double[] busIdArr = {busId};
			buses.add(new Pair(bus.getBusType().toString(), busIdArr));
			for (AbstractSensor sensor : vehicle.getSensorList()) {
				if (bus.getConnectedComponents().contains(sensor)) {
					sensors.add(new Pair(sensor.getClass().getName(), busIdArr));
				}
			}
			for (VehicleActuator actuator : vehicle.getActuatorList()) {
				if (bus.getConnectedComponents().contains(actuator)) {
					double[] arr = {busId, actuator.getActuatorValueMin(), actuator.getActuatorValueMax(), actuator.getActuatorValueChangeRate()};
					actuators.add(new Pair(actuator.getActuatorType().toString(), arr));
				}
			}
			for (Bridge bridge : vehicle.getBridgeList()) {
				if (bus.getConnectedComponents().contains(bridge) && !processedBridges.contains(bridge)) {
					double[] arr = {busId, vehicle.getBusList().indexOf(bridge.getConnectedBuses().getRight()), bridge.getDelay().toMillis()};
					bridges.add(new Pair("bridge", arr));
					processedBridges.add(bridge);
				}
			}
			if (bus.getConnectedComponents().contains(vehicle.getNavigation())) {
				autopilot = new Pair("navigation", busIdArr);
			}

			busId++;
		}


	}

	public List<Pair> getBuses() {
		return buses;
	}

	public List<Pair> getActuators() {
		return actuators;
	}

	public List<Pair> getBridges() {
		return bridges;
	}

	public List<Pair> getSensors() {
		return sensors;
	}

	public Pair getAutopilot() {
		return autopilot;
	}
}
