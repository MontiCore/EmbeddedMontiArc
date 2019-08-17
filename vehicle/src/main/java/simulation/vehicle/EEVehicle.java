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
import commons.simulation.Sensor;

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
import org.apache.commons.lang3.tuple.Pair;
import simulation.EESimulator.*;
import simulation.bus.Bus;
import simulation.bus.BusMessage;
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

    //private AutoPilot autoPilot;

    private List<Bus> busList = new LinkedList<>();

    private List<Sensor> sensorList = new LinkedList<>();

    private List<VehicleActuator> actuatorList = new LinkedList<>();

    private List<Bridge> bridgeList = new LinkedList<>();

    Double allowedVelocityByStreetType = 0.;


    /*
    TODO:   - maybe add Autopilot
     */

    /**
     * Constructor of EEVehicle
     * @param eeSimulator             simulator of the Vehicle
     * @param busStructure            JSON file that describes the bus structures
     */
    public EEVehicle(EESimulator eeSimulator, File busStructure) {

        //set EESimulator
        this.eeSimulator = eeSimulator;

        //set bus, sensor, actuator and bridge lists
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
                        sensorList.add((Sensor) comp);
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
    public EEVehicle(EESimulator eeSimulator, HashMap<Bus, List<EEComponent>> busStructure) {

        //set EESimulator
        this.eeSimulator = eeSimulator;

        //set bus, sensor, actuator and bridge lists

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
                            sensorList.add((Sensor) comp);
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
	}

	/**
	 * function that notifies all sensors to send their actual data to the bus
	 * 
	 * @param actualTime actual time of the simulation
	 */
	public void notifySensors(Instant actualTime) {
		for (AbstractSensor sensor : sensorList) {
			if (sensor.getType() == BusEntry.SENSOR_STREETTYPE) {
				Double allowedVelocityByStreetType;
				switch ((String) sensor.getValue()) {
				case "MOTORWAY":
					allowedVelocityByStreetType = (100.0 / 3.6);
					break;
				case "A_ROAD":
					allowedVelocityByStreetType = (70.0 / 3.6);
					break;
				case "STREET":
					allowedVelocityByStreetType = (50.0 / 3.6);
					break;
				case "LIVING_STREET":
					allowedVelocityByStreetType = (30.0 / 3.6);
					break;
				default:
					allowedVelocityByStreetType = ((StreetTypeSensor) sensor).getLastVelocityValue();
					break;
				}
				((StreetTypeSensor) sensor).setLastVelocityValue(allowedVelocityByStreetType);
				for (EEComponent target : sensor.getTargetsByMessageId().get(BusEntry.SENSOR_STREETTYPE)) {
					BusMessage sensorMess = new BusMessage(allowedVelocityByStreetType, 6, BusEntry.SENSOR_STREETTYPE,
							actualTime, sensor.getId(), target);
					eeSimulator.addEvent(sensorMess);
				}
			}
			for (EEComponent target : sensor.getTargetsByMessageId().get(sensor.getType())) {
				BusMessage sensorMess = new BusMessage(sensor.getValue(), sensor.getDataLength(), sensor.getType(),
						actualTime, sensor.getId(), target);
				eeSimulator.addEvent(sensorMess);
			}
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

	public Optional<VehicleActuator> getActuator(VehicleActuatorType actuatorType) {
		for (VehicleActuator actuator : this.actuatorList) {
			if (actuator.getActuatorType() == actuatorType) {
				return Optional.of(actuator);
			}
		}
		return Optional.empty();
	}

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

	public static EEVehicle createBasicEEVehicle(PhysicalVehicle pyhsicalVehicle) {
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicle eeVehicle = new EEVehicle(pyhsicalVehicle, eeSimulator);
		Bus bus = new InstantBus(eeSimulator);
		HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
		targetsByMessageId.put(SpeedSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new SpeedSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(LocationSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new LocationSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(SteeringAngleSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new SteeringAngleSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(DistanceToRightSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new DistanceToRightSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(DistanceToLeftSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new DistanceToLeftSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(WeatherSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new WeatherSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(CameraSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new CameraSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId, Collections.emptyList()));

        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(CompassSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new CompassSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(LeftBackWheelDistanceToStreetSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new LeftBackWheelDistanceToStreetSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));

        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(LeftFrontWheelDistanceToStreetSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new LeftFrontWheelDistanceToStreetSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(RightFrontWheelDistanceToStreetSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new RightFrontWheelDistanceToStreetSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
     
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(RightBackWheelDistanceToStreetSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new RightBackWheelDistanceToStreetSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
    
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(StreetTypeSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new StreetTypeSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(DayNightSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new DayNightSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(LeftFrontDistanceSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new LeftFrontDistanceSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(RightFrontDistanceSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new RightFrontDistanceSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(ObstacleSensor.getSensorType(), Collections.singletonList(bus));
        eeVehicle.addSensor(new ObstacleSensor(pyhsicalVehicle, eeSimulator, Collections.emptyList(), targetsByMessageId));
        

        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(BusEntry.ACTUATOR_ENGINE_CURRENT, Collections.singletonList(bus));
        eeVehicle.addActuator(new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR, Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MIN, Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_MAX, Vehicle.VEHICLE_DEFAULT_MOTOR_ACCELERATION_RATE, eeSimulator, Collections.singletonList(BusEntry.ACTUATOR_ENGINE), targetsByMessageId));
        // Create the brakes
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(BusEntry.ACTUATOR_BRAKE_CURRENT, Collections.singletonList(bus));
        eeVehicle.addActuator(new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_LEFT, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE, eeSimulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetsByMessageId));
        eeVehicle.addActuator(new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_FRONT_RIGHT, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE, eeSimulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetsByMessageId));
        eeVehicle.addActuator(new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE, eeSimulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetsByMessageId));
        eeVehicle.addActuator(new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_RIGHT, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MIN, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_MAX, Vehicle.VEHICLE_DEFAULT_BRAKES_ACCELERATION_RATE, eeSimulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE), targetsByMessageId));
        // Create the steering
        targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
        targetsByMessageId.put(BusEntry.ACTUATOR_STEERING_CURRENT, Collections.singletonList(bus));
        eeVehicle.addActuator(new VehicleActuator(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MIN, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_MAX, Vehicle.VEHICLE_DEFAULT_STEERING_ANGLE_RATE, eeSimulator, Collections.singletonList(BusEntry.ACTUATOR_STEERING), targetsByMessageId));
 
        
		return eeVehicle;
	}



    //TODO: change the time of this function
    /**
     * function that notifies all actuators to update
     * @param actualTime time the actuators get to update their value
     */
    public void notifyActuator(Duration actualTime){
        for (VehicleActuator actuator : actuatorList) {
            actuator.update((double) actualTime.toMillis()/1000);
        }
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
