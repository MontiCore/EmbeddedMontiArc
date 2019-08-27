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

import commons.controller.interfaces.FunctionBlockInterface;
import commons.simulation.IPhysicalVehicle;
import commons.simulation.Sensor;
import jdk.internal.org.jline.utils.Log;
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
import simulation.bus.FlexRay;
import simulation.bus.InstantBus;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.time.Duration;
import java.time.Instant;
import java.util.*;

public class EEVehicle {

    private EESimulator eeSimulator;

    private FunctionBlockInterface autoPilot;

    private List<Bus> busList = new LinkedList<>();

    private List<AbstractSensor> sensorList = new LinkedList<>();

    private List<VehicleActuator> actuatorList = new LinkedList<>();

    private List<Bridge> bridgeList = new LinkedList<>();
    
    private boolean collision = false;


    /*
    TODO: create autopilot
     */

    /**
     * Constructor of EEVehicle
     * @param eeSimulator simulator of the Vehicle
     * @param data JSON file that describes the bus structures
     */
    public EEVehicle(EESimulator eeSimulator, IPhysicalVehicle physicalVehicle, File data) {

        //set EESimulator
        this.eeSimulator = eeSimulator;

        //set bus, sensor, actuator and bridge lists
		ParsableBusStructureProperties busStructure;

		try {
			busStructure = loadFromFile(data);

			//create buses
			for (ParsableBusStructureProperties.Pair component : busStructure.getBuses()) {
				if (component.eeComponent.equals("flexRay")) {
					FlexRay flexRay = new FlexRay(eeSimulator);
					busList.add((int) component.bus[0], flexRay);
				} else if (component.eeComponent.equals("instantBus")) {
					InstantBus instantBus = new InstantBus(eeSimulator);
					busList.add((int) component.bus[0], instantBus);
				}
			}

			//create sensors, actuator and bridges
			for (ParsableBusStructureProperties.Pair sensor : busStructure.getSensors()) {
				Class<? extends AbstractSensor> sensorClass = (Class<? extends AbstractSensor>) Class.forName(sensor.eeComponent);
				AbstractSensor newSensor = AbstractSensor.createSensor(sensorClass, physicalVehicle, busList.get((int) sensor.bus[0])).get();
				sensorList.add(newSensor);
				busList.get((int) sensor.bus[0]).registerComponent(newSensor);
			}

			for (ParsableBusStructureProperties.Pair actuator : busStructure.getActuators()) {
				VehicleActuator newActuator = VehicleActuator.createVehicleActuator(VehicleActuatorType.valueOf(actuator.eeComponent), actuator.bus[1], actuator.bus[2], actuator.bus[3], busList.get((int) actuator.bus[0]));
				actuatorList.add(newActuator);
				busList.get((int) actuator.bus[0]).registerComponent(newActuator);
			}

			for (ParsableBusStructureProperties.Pair bridge : busStructure.getBridges()) {
				Bridge newBridge = new Bridge(eeSimulator, Pair.of(busList.get((int) bridge.bus[0]), busList.get((int) bridge.bus[1])), Duration.ofMillis((long) bridge.bus[2]));
				bridgeList.add(newBridge);
				busList.get((int) bridge.bus[0]).registerComponent(newBridge);
				busList.get((int) bridge.bus[1]).registerComponent(newBridge);
			}

		} catch (IOException e) {
			Log.error("Can not create EEVehicle. Failed to read the file");
		} catch (ClassNotFoundException e) {
			Log.error("Can not create EEVehicle. File contains non-existent sensor");
		}

	}







	/*
	TODO: register autopilot at the bus he is connected to.
	 */

	/**
	 * Constructor if no JSON file is available
	 * 
	 * @param eeSimulator  simulator of the vehicle
	 * @param buses list of buses the vehicle has
	 * @param components list of components (actuators, sensors, bridges and autopilot) the vehicle has
	 */
    public EEVehicle(EESimulator eeSimulator, List<Bus> buses, List<EEComponent> components) {

        //set EESimulator
        this.eeSimulator = eeSimulator;

		for (Bus bus : buses) {
			busList.add(bus);
		}
		for (EEComponent component : components) {
			switch (component.getComponentType()) {
				case SENSOR:
					sensorList.add((AbstractSensor) component);
					for (EEComponent bus : component.getTargetsByMessageId().get(((AbstractSensor) component).getType())) {
						((Bus) bus).registerComponent(component);
					}
					break;
				case ACTUATOR:
					actuatorList.add((VehicleActuator) component);
					for (EEComponent bus : component.getTargetsByMessageId().get(((VehicleActuator) component).getSendMsgId())) {
						((Bus) bus).registerComponent(component);
					}
					break;
				case BRIDGE:
					bridgeList.add((Bridge) component);
					((Bridge) component).getConnectedBuses().getLeft().registerComponent(component);
					((Bridge) component).getConnectedBuses().getRight().registerComponent(component);
					break;
				case AUTOPILOT:
					autoPilot = (FunctionBlockInterface) component;
					//TODO: implement!

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
	 * @param actualTime actual time of the simulation
	 */
	public void notifySensors(Instant actualTime) {
		for (AbstractSensor sensor : sensorList) {
			sensor.update(actualTime);
		}
	}

	/**
	 * function that notifies all actuators to update
	 * @param actualTime time the actuators get to update their value
	 */
	//TODO: wert des updates nach update auf den bus schreiben
	public void notifyActuator(Instant actualTime){
		if(!this.collision) {
			for (VehicleActuator actuator : actuatorList) {
				actuator.update(actualTime);
			}
		}
		else {
			for (VehicleActuator actuator : actuatorList) {
				actuator.reset();
			}
			this.collision = false;
		}
	}


	public EESimulator getEeSimulator() {
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

	public FunctionBlockInterface getAutoPilot() {
		return this.autoPilot;
	}
	/**
	 * Add sensor to sensor list and register at target buses
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
	 * @param actuator actuator to be registered
	 */
	private void addActuator(VehicleActuator actuator) {
		for(List<EEComponent> targets : actuator.getTargetsByMessageId().values()) {
			for (EEComponent target : targets) {
				if (target.getComponentType() == EEComponentType.BUS) {
					Bus bus = (Bus) target;
					bus.registerComponent(actuator);
				}
			}
		}
		this.actuatorList.add(actuator);
	}

	public ParsableBusStructureProperties loadFromFile(File file) throws IOException {

		List<EEComponent> components = new LinkedList<>();
		String jsonContents = new String(Files.readAllBytes(file.toPath()));
		Gson g = new Gson();
		ParsableBusStructureProperties data = g.fromJson(jsonContents, ParsableBusStructureProperties.class);

		return data;
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

	public void setCollision(boolean collision) {
		this.collision = collision;
	}

}

/**
 * all data of a bus structure, including all connected actuators, buses,
 * sensors and autopilots.
 * Sensors are saved by their class name.
 * Actuators are saved by their actuatorType.
 * Buses are saved by their busType.
 * Autopilot is saved by String "navigation".
 */
class ParsableBusStructureProperties {

	class Pair {
		String eeComponent;
		double[] bus;

		public Pair(String eeComponent, double[] bus) {
			this.eeComponent = eeComponent;
			this.bus = bus;
		}


		public String getEeComponent() {
			return eeComponent;
		}

		public double[] getBus() {
			return bus;
		}
	}


	private List<Pair> buses = new LinkedList<>();
	private List<Pair> sensors = new LinkedList<>();
	private List<Pair> actuators = new LinkedList<>();
	private List<Pair> bridges = new LinkedList<>();
	private Pair autopilot;

	public ParsableBusStructureProperties(EEVehicle vehicle) {

		List<Bridge> processedBridges = new LinkedList<>();

		//id to assign component to the bus and index of the bus in busList of the eeVehicle
		int busId = 0;

		for (Bus bus : vehicle.getBusList()) {
			double[] busIdArr = {busId};
			buses.add(new Pair(bus.getBusType(), busIdArr));
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
			if (bus.getConnectedComponents().contains(vehicle.getAutoPilot())) {
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
