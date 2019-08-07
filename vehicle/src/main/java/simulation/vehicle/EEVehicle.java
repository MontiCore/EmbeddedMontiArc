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

import com.google.gson.Gson;
import commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.Pair;
import sensors.StreetTypeSensor;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.*;
import simulation.bus.Bus;
import simulation.bus.BusMessage;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.time.Duration;
import java.time.Instant;
import java.util.*;

public class EEVehicle {

    private PhysicalVehicle physicalVehicle;

    private EESimulator eeSimulator;

    //private AutoPilot autoPilot;

    private List<Bus> busList = new LinkedList<>();

    private List<AbstractSensor> sensorList = new LinkedList<>();

    private List<VehicleActuator> actuatorList = new LinkedList<>();

    /*
    TODO:
            - modelica und MassPoint mal untersuchen und gucken ob/wo actuator unterschiedlich
     */

    /**
     * Constructor of EEVehicle
     *
     * @param eeSimulator             simulator of the Vehicle
     * @param busSystems              a HashMap of all bus systems on this vehicle. The keys of the HashMap are the bus systems on this vehicle, the entries for each key are the components connected to the bus
     * @param modelicaPhysicalVehicle true if physicalVehicle should be type of ModelicaPhysicalVehicle, false if physicalVehicle should be type of MassPointPhysicalVehicle
     */
    public EEVehicle(EESimulator eeSimulator, boolean modelicaPhysicalVehicle, File physicalVehicleProperties, File busStructure) {
        PhysicalVehicleBuilder builder = new MassPointPhysicalVehicleBuilder();
        /*if(modelicaPhysicalVehicle){
            builder = new ModelicaPhysicalVehicleBuilder();
        } else {
            builder = new MassPointPhysicalVehicleBuilder();
        }*/
        try {
            this.physicalVehicle = ((MassPointPhysicalVehicleBuilder) builder).loadFromFile(physicalVehicleProperties);
        } catch (IOException e) {
            e.printStackTrace();
        }

        HashMap<Bus, List<EEComponent>> busSystems = null;
        try {
            busSystems = loadFromFile(busStructure);
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.eeSimulator = eeSimulator;

        Set<Bus> processedKeys = new HashSet<>();

        for (Bus actualBus : busSystems.keySet()) {
            busList.add(actualBus);
            for (EEComponent comp : busSystems.get(actualBus)) {
                switch (comp.getComponentType()) {
                    case ACTUATOR:
                        physicalVehicle.getSimulationVehicle().setActuatorProperties((VehicleActuator) comp);
                        if (!actuatorList.contains(comp)) {
                            actuatorList.add((VehicleActuator) comp);
                        }
                        break;
                    case SENSOR:
                        physicalVehicle.getSimulationVehicle().addSensor((AbstractSensor) comp);
                        if (!sensorList.contains(comp)) {
                            sensorList.add((AbstractSensor) comp);
                        }
                        break;
                    case AUTOPILOT:
                        //autoPilot = comp;
                    case BUS:
                        if (!processedKeys.contains(comp)) {
                            Pair<Bus, Bus> busPair = Pair.of(actualBus, (Bus) comp);
                            Duration delay = Duration.ofNanos(2);
                            Bridge bridge = new Bridge(eeSimulator, busPair, delay);
                            bridge.update(actualBus, actualBus.getSubscribedMessages());
                            ((Bus) comp).registerComponent(bridge);

                            comp = bridge;
                        }
                        break;
                    default: //exception?
                }
                actualBus.registerComponent(comp);
                processedKeys.add(actualBus);

            }
        }
    }


    /**
     * function that notify all sensors to send their actual data to the bus
     *
     * @param timeDiff   duration between last tick and actual tick
     * @param actualTime actual time of the simulation
     */
    public void notifySensors(Instant actualTime, Duration timeDiff) {
        physicalVehicle.executeLoopIteration(timeDiff);
        for (AbstractSensor sensor : sensorList) {
            if (sensor.getType() == BusEntry.SENSOR_STREETTYPE) {
                Double allowedVelocityByStreetType = physicalVehicle.getSimulationVehicle().getMaxTemporaryAllowedVelocity();
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
                        allowedVelocityByStreetType = physicalVehicle.getSimulationVehicle().getMaxTemporaryAllowedVelocity();
                        break;
                }
                for (EEComponent target : sensor.getTargetsByMessageId().get(BusEntry.SENSOR_STREETTYPE)) {
                    BusMessage sensorMess = new BusMessage(allowedVelocityByStreetType, 6, BusEntry.SENSOR_STREETTYPE, actualTime, sensor.getId(), target);
                    eeSimulator.addEvent(sensorMess);
                }
            }
            for (EEComponent target : sensor.getTargetsByMessageId().get(sensor.getType())) {
                BusMessage sensorMess = new BusMessage(sensor.getValue(), sensor.getDataLength(), sensor.getType(), actualTime, sensor.getId(), target);
                eeSimulator.addEvent(sensorMess);
            }
        }
    }


    public EESimulator getEeSimulator() {
        return eeSimulator;
    }

    public PhysicalVehicle getPhysicalVehicle() {
        return physicalVehicle;
    }

    private List<Bus> getBusList() {
        return busList;
    }

    private List<AbstractSensor> getSensorList() {
        return sensorList;
    }

    private List<VehicleActuator> getActuatorList() {
        return actuatorList;
    }


    public HashMap<Bus, List<EEComponent>> loadFromFile(File file) throws IOException {

        String jsonContents = new String(Files.readAllBytes(file.toPath()));
        Gson g = new Gson();
        EEVehicle.ParsableBusStructureProperties data = g.fromJson(jsonContents, EEVehicle.ParsableBusStructureProperties.class);

        return data.getBusSystems();

    }

    public void storeInFile(File whereToStore, EEVehicle vehicle) throws IOException {

        ParsableBusStructureProperties properties = new ParsableBusStructureProperties(vehicle);

        Gson g = new Gson();
        String json = g.toJson(properties, EEVehicle.ParsableBusStructureProperties.class);

        FileWriter fileWriter = new FileWriter(whereToStore, false);

        fileWriter.write(json);
        fileWriter.flush();
        fileWriter.close();
    }



    /**
     * all data of a busstructure, including all connected actuators, buses and sensors
     */
    public static class ParsableBusStructureProperties {

        HashMap<Bus, List<EEComponent>> busSystems = new HashMap<>();

        public ParsableBusStructureProperties(EEVehicle vehicle) {

            for (Bus bus : vehicle.getBusList()) {
                busSystems.put(bus, new LinkedList<>());

                //add all sensors connected to this bus
                for (AbstractSensor sensor : vehicle.getSensorList()) {
                    if (bus.getConnectedComponents().contains(sensor)) {
                        busSystems.get(bus).add(sensor);
                    }
                }

                //add all actuator connected to this bus
                for (VehicleActuator actuator : vehicle.getActuatorList()) {
                    if (bus.getConnectedComponents().contains(actuator)) {
                        busSystems.get(bus).add(actuator);
                    }
                }

            }

        }

        public HashMap<Bus, List<EEComponent>> getBusSystems() {
            return busSystems;
        }
    }
}
