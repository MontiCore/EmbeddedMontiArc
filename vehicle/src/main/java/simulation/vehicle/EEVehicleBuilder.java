/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import com.google.gson.Gson;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.HardwareEmulatorInterface;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import sensors.abstractsensors.AbstractSensor;
import sensors.factory.SensorFactory;
import sensors.util.SensorUtil;
import simulation.EESimulator.*;
import simulation.bus.Bus;
import simulation.bus.FlexRay;
import simulation.bus.FlexRayOperationMode;
import simulation.bus.InstantBus;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.time.Duration;
import java.util.*;

public class EEVehicleBuilder {
	
	private final EESimulator eeSimulator;
	
	private Map<BusEntry,List<Bus>> busesBySensorBusEntry = new HashMap<BusEntry, List<Bus>>();
	
	Set<Bus> addAllSensors = new HashSet<Bus>();
	
	private List<EEComponent> components = new ArrayList<EEComponent>();   
	
	public EEVehicleBuilder(EESimulator eeSimulator) {
		this.eeSimulator = eeSimulator;
	}
	
	public EEVehicle buildEEVehicle(Vehicle vehicle, PhysicalVehicle physicalVehicle) {
		Set<Bus> buses = new HashSet<Bus>();
		if(!addAllSensors.isEmpty()) {
			components.addAll(SensorUtil.sensorAdder(physicalVehicle, new ArrayList<Bus>(addAllSensors)));
		
		}
		for(Map.Entry<BusEntry ,List<Bus>> entry: busesBySensorBusEntry.entrySet()) {
			Optional<AbstractSensor> sensor = SensorFactory.createSensor(entry.getKey(), physicalVehicle, entry.getValue());
			if(sensor.isPresent()) {
				components.add(sensor.get());
			}
			else {
				throw new IllegalStateException("Could not create sensor " + entry.getKey());
			}
		}
		for(EEComponent component : components) {
			for(List<EEComponent> targets : component.getTargetsByMessageId().values()){
				for(EEComponent target : targets) {
					if(target.getComponentType() != EEComponentType.BUS) {
						throw new IllegalStateException("Target of components can only be buses");
					}
					buses.add((Bus) target);
				}
			}
		}
		return new EEVehicle(vehicle, eeSimulator, buses, components);
	}

	/**
	 * function to build an EEVehicle by using a JSON File with the bus structure
	 * @param vehicle
	 * @param data JSON file with bus structure (ParsableBusStructure class)
	 * @return
	 */
	public EEVehicle buildEEVehicle(Vehicle vehicle, File data) {
		List<Bus> busList = new LinkedList<>();
		//set busAndParameter, sensor, actuator and bridge lists
		ParsableBusStructureProperties busStructure;

		try {
			busStructure = loadFromFile(data);

			//create buses
			for (ParsableBusStructureProperties.Tupel component : busStructure.getBuses()) {
				if (component.eeComponent.equals("flexRay")) {
					FlexRay flexRay = new FlexRay(eeSimulator);
					flexRay.setOperationMode(FlexRayOperationMode.valueOf(component.parameter[0]));
					busList.add(flexRay);
				} else if (component.eeComponent.equals("instantBus")) {
					InstantBus instantBus = new InstantBus(eeSimulator);
					busList.add(instantBus);
				} else if (component.eeComponent.equals("can")) {
//					CAN canBus = new CAN(eeSimulator, CANOperationMode.valueOf(component.parameter[0]));
//					busList.add(canBus);
				}
			}

			//create sensors, actuator and bridges
			for (ParsableBusStructureProperties.Tupel sensor : busStructure.getSensors()) {
				try {
					Class<? extends AbstractSensor> sensorClass = (Class<? extends AbstractSensor>) Class.forName(sensor.eeComponent);
					AbstractSensor newSensor = SensorFactory.createSensor(sensorClass, vehicle.getPhysicalVehicle(), busList.get(sensor.busIds[0])).get();
					components.add(newSensor);
//					busList.get(sensor.busIds[0]).registerComponent(newSensor);
				} catch (ClassNotFoundException e) {
					throw new IllegalArgumentException("Can not create EEVehicle. File contains non-existent sensor: " + sensor.eeComponent);
				}
			}

			for (ParsableBusStructureProperties.Tupel actuator : busStructure.getActuators()) {
				VehicleActuator newActuator = VehicleActuator.createVehicleActuator(VehicleActuatorType.valueOf(actuator.eeComponent), Double.parseDouble(actuator.parameter[0]), Double.parseDouble(actuator.parameter[1]), Double.parseDouble(actuator.parameter[2]), busList.get(actuator.busIds[0]));
				components.add(newActuator);
//				busList.get(actuator.busIds[0]).registerComponent(newActuator);
			}

			for (ParsableBusStructureProperties.Tupel bridge : busStructure.getBridges()) {
				Bridge newBridge = new Bridge(Pair.of(busList.get((bridge.busIds[0])), busList.get(Integer.parseInt(bridge.parameter[1]))), Duration.ofMillis(Long.parseLong(bridge.parameter[2])));
				components.add(newBridge);
			}

			if (busStructure.getNavigation() != null && busStructure.getNavigation().busIds != null) {
				NavigationBlockAsEEComponent navigation;
				navigation = NavigationBlockAsEEComponent.createNavigationBlockAsEEComponent(busList.get(busStructure.getNavigation().busIds[0]));
				vehicle.initNavigation(navigation);
				components.add(navigation);
//				busList.get(busStructure.getNavigation().busIds[0]).registerComponent(navigation);
			}

			if (!busStructure.getController().isEmpty()) {
				for (ParsableBusStructureProperties.Tupel controller : busStructure.getController()) {
					DirectModelAsEEComponent newController = DirectModelAsEEComponent.createDirectModelAsEEComponent(busList.get(controller.busIds[0]));
					newController.setCycleTime(Duration.parse(controller.parameter[0]));
					components.add(newController);
//					busList.get(controller.busIds[0]).registerComponent(newController);
				}
			}


		} catch (IOException e) {
			throw  new IllegalArgumentException("Can not create EEVehicle. Failed to read file: " + data);
		}

		return new EEVehicle(vehicle, eeSimulator, new HashSet<>(busList), components);
	}

	public NavigationBlockAsEEComponent createNavigation(List<Bus> buses) {
		NavigationBlockAsEEComponent comp = NavigationBlockAsEEComponent.createNavigationBlockAsEEComponent(buses);
		components.add(NavigationBlockAsEEComponent.createNavigationBlockAsEEComponent(buses));
		return comp;
	}
	
	public NavigationBlockAsEEComponent createNavigation(Bus bus) {
		return createNavigation(Collections.singletonList(bus));
	}

	public DirectModelAsEEComponent createController(HardwareEmulatorInterface modelServer, String autopilotConfig, List<Bus> buses) {
		List<EEComponent> targets = new ArrayList<EEComponent>(buses);
		HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
		for(BusEntry busEntry : DirectModelAsEEComponent.MASSPOINT_OUTPUT_MESSAGES) {
        	targetsByMessageId.put(busEntry, targets);
        }
		DirectModelAsEEComponent comp = new DirectModelAsEEComponent(eeSimulator, targetsByMessageId);
		try {
			comp.initializeController(modelServer, autopilotConfig, Duration.ofMillis(30));
		} catch (Exception e) {
			throw new IllegalStateException("Controller could not be created. " + e.getMessage() + "\r\n" + e.getStackTrace());
		}
		components.add(comp);
		return comp;
	}

	public DirectModelAsEEComponent createController(HardwareEmulatorInterface modelServer, String autopilotConfig, Bus bus) {
		return createController(modelServer, autopilotConfig, Collections.singletonList(bus));
	}
	
	public void createSensor(BusEntry sensorType, List<Bus> buses) {
		busesBySensorBusEntry.put(sensorType, buses);
	}
	
	public void createSensor(BusEntry sensorType, Bus bus) {
		createSensor(sensorType, Collections.singletonList(bus));
	}
	
	public void createAllSensors(List<Bus> buses) {
		addAllSensors.addAll(buses);
	}
	
	public void createAllSensors(Bus bus) {
		addAllSensors.add(bus);
	}
	
	public VehicleActuator createActuator(VehicleActuatorType actuatorType, List<Bus> buses) {
		VehicleActuator comp = VehicleActuator.createVehicleActuator(actuatorType, buses);
		components.add(comp);
		return comp;
	}
	
	public VehicleActuator createActuator(VehicleActuatorType actuatorTpye, Bus bus) {
		return createActuator(actuatorTpye, Collections.singletonList(bus));
	}
	
	public List<VehicleActuator> createAllActuators(List<Bus> buses) {
		List<VehicleActuator> comps = new ArrayList<VehicleActuator>();
		for(VehicleActuatorType type : VehicleActuatorType.values()) {
			comps.add(createActuator(type, buses));
		}
		return comps;
	}
	
	public List<VehicleActuator> createAllActuators(Bus bus) {
		return createAllActuators(Collections.singletonList(bus));
	}

	public List<VehicleActuator> createMassPointActuators(List<Bus> buses){
		List<VehicleActuator> comps = new ArrayList<VehicleActuator>();
		comps.add(createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, buses));
		comps.add(createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, buses));
		comps.add(createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, buses));
		comps.add(createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, buses));
		comps.add(createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR, buses));
		comps.add(createActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, buses));
		return comps;
	}

	public List<VehicleActuator> createMassPointActuators(Bus bus){
		return createMassPointActuators(Collections.singletonList(bus));
	}

	public void createControllerSensors(List<Bus> buses){
		createSensor(BusEntry.SENSOR_VELOCITY, buses);
		createSensor(BusEntry.SENSOR_GPS_COORDINATES, buses);
		createSensor(BusEntry.SENSOR_COMPASS, buses);
		createSensor(BusEntry.PLANNED_TRAJECTORY_X, buses);
		createSensor(BusEntry.PLANNED_TRAJECTORY_Y, buses);
	}

	public void createControllerSensors(Bus bus){
		createControllerSensors(Collections.singletonList(bus));
	}
	
	public void createAllSensorsNActuators(Bus bus) {
		createAllSensors(bus);
		createAllActuators(bus);
	}
	
	public void createAllSensorsNActuators(List<Bus> buses) {
		createAllSensors(buses);
		createAllActuators(buses);
	}
	
	public Bridge connectBuses(Bus bus1, Bus bus2, Duration duration) {
		Bridge comp = new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus2), duration);
		components.add(comp);
		return comp;
	}
	
	public Bridge connectBuses(Bus bus1, Bus bus2) {
		return connectBuses(bus1, bus2, Duration.ofMillis(1));
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

	class Tupel {
		String eeComponent;
		int[] busIds;
		String[] parameter;

		public Tupel(String eeComponent, int[] bus, String[] parameter) {
			this.eeComponent = eeComponent;
			this.busIds= bus;
			this.parameter = parameter;
		}
	}


	private List<Tupel> buses = new LinkedList<>();
	private List<Tupel> sensors = new LinkedList<>();
	private List<Tupel> actuators = new LinkedList<>();
	private List<Tupel> bridges = new LinkedList<>();
	private Tupel navigation;
	private List<Tupel> controller = new LinkedList<>();

	public ParsableBusStructureProperties(EEVehicle vehicle) {

		List<Bridge> processedBridges = new LinkedList<>();
		List<AbstractSensor> processedSensors = new LinkedList<>();
		List<VehicleActuator> processedActuators = new LinkedList<>();
		List<DirectModelAsEEComponent> processedController = new LinkedList<>();

		//id to assign component to the busAndParameter and index of the busAndParameter in busList of the eeVehicle
		int busId = 0;

		for (Bus bus : vehicle.getBusList()) {
			int[] busIdArr = {busId};
			switch (bus.getBusType()) {
				case CAN:
//					String[] param = {((CAN) bus).getMode()};
//					buses.add(new Tupel(bus.getBusType().toString(), busIdArr, param));
					break;
				case FLEXRAY:
					String[] param = {((FlexRay) bus).getOperationMode().toString()};
					buses.add(new Tupel(bus.getBusType().toString(), busIdArr, param));
					break;
				case INSTANT_BUS:
					buses.add(new Tupel(bus.getBusType().toString(), busIdArr, null));
					break;
				default:
					throw new IllegalStateException("connected Bus has unknown type. Type was " + bus.getBusType());
			}
			for (EEComponent component : bus.getConnectedComponents()) {
				switch (component.getComponentType()) {
					case SENSOR:
						sensors.add(new Tupel(component.getClass().getName(), busIdArr, null));
						break;
					case ACTUATOR:
						String[] paramAct = {(((VehicleActuator) component).getActuatorValueMin()) + "", ((VehicleActuator) component).getActuatorValueMax() + "", ((VehicleActuator) component).getActuatorValueChangeRate() + ""};
						actuators.add(new Tupel(((VehicleActuator) component).getActuatorType().toString(), busIdArr, paramAct));
						break;
					case AUTOPILOT:
						String[] paramController = {((DirectModelAsEEComponent) component).getCycleTime().toString()};
						controller.add(new Tupel("autopilot", busIdArr, paramController));
						break;
					case NAVIGATION:
						navigation = new Tupel("navigation", busIdArr, null);
						break;
					case BRIDGE:
						if (!processedBridges.contains(component)) {
							String[] paramBridge = {vehicle.getBusList().indexOf(((Bridge) component).getConnectedBuses().getRight()) + "", ((Bridge) component).getDelay().toMillis() + ""};
							bridges.add(new Tupel("bridge", busIdArr, paramBridge));
							processedBridges.add((Bridge) component);
						}
						break;
					default:
						throw new IllegalStateException("connected component was unknown type. Type was " + component.getComponentType());
				}
			}
			busId++;
		}
	}

	public List<Tupel> getBuses(){
		return buses;
	}

	public List<Tupel> getActuators() {
		return actuators;
	}

	public List<Tupel> getBridges() {
		return bridges;
	}

	public List<Tupel> getSensors() {
		return sensors;
	}

	public List<Tupel> getController() {
		return controller;
	}

	public Tupel getNavigation() {
		return navigation;
	}

}

