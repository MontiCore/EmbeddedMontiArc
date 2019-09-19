/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.jfree.util.Log;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.HardwareEmulatorInterface;
import sensors.factory.SensorFactory;
import sensors.util.SensorUtil;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.DirectModelAsEEComponent;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEComponentType;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.NavigationBlockAsEEComponent;
import simulation.bus.Bus;

public class EEVehicleBuilder {
	
	private final EESimulator eeSimulator;
	
	private Map<BusEntry,List<Bus>> busesBySensorClass = new HashMap<BusEntry, List<Bus>>();
	
	Set<Bus> addAllSensors = new HashSet<Bus>();
	
	private List<EEComponent> components = new ArrayList<EEComponent>();   
	
	public EEVehicleBuilder(EESimulator eeSimulator) {
		this.eeSimulator = eeSimulator;
	}
	
	public EEVehicle buildEEVehicle(Vehicle vehicle, PhysicalVehicle physicalVehicle) {
		Set<Bus> buses = new HashSet<Bus>();
		if(!addAllSensors.isEmpty()) {
			SensorUtil.sensorAdder(physicalVehicle, new ArrayList<Bus>(addAllSensors));

		}
		for(Map.Entry<BusEntry ,List<Bus>> entry: busesBySensorClass.entrySet()) {
			SensorFactory.createSensor(entry.getKey(), physicalVehicle, entry.getValue()).ifPresent(s -> components.add(s));
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

	public NavigationBlockAsEEComponent createNavigation(List<Bus> buses) {
		NavigationBlockAsEEComponent comp = NavigationBlockAsEEComponent.createNavigationBlockAsEEComponent(buses);
		components.add(NavigationBlockAsEEComponent.createNavigationBlockAsEEComponent(buses));
		return comp;
	}
	
	public NavigationBlockAsEEComponent createNavigation(Bus bus) {
		return createNavigation(Collections.singletonList(bus));
	}

	public Optional<DirectModelAsEEComponent> createController(HardwareEmulatorInterface modelServer, String autopilotConfig, List<Bus> buses) {
		List<EEComponent> targets = new ArrayList<EEComponent>(buses);
		HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
		Optional<DirectModelAsEEComponent> comp = Optional.empty();
		for(BusEntry busEntry : DirectModelAsEEComponent.MASSPOINT_OUTPUT_MESSAGES) {
        	targetsByMessageId.put(busEntry, targets);
        }
		try {
			comp = Optional.of(new DirectModelAsEEComponent(eeSimulator, targetsByMessageId));
			comp.get().initializeController(modelServer, autopilotConfig);
			components.add(comp.get());
		} catch (Exception e) {
			Log.error("Could not generate controller");
		}
		return comp;
	}

	public Optional<DirectModelAsEEComponent> createController(HardwareEmulatorInterface modelServer, String autopilotConfig, Bus bus) {
		return createController(modelServer, autopilotConfig, Collections.singletonList(bus));
	}
	
	public void createSensor(BusEntry sensorType, List<Bus> buses) {
		busesBySensorClass.put(sensorType, buses);
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
	
	public VehicleActuator createActuator(VehicleActuatorType actuatorTpye, List<Bus> buses) {
		VehicleActuator comp = VehicleActuator.createVehicleActuator(actuatorTpye, buses);
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
}
