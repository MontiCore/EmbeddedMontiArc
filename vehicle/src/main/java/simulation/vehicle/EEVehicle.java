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
import commons.simulation.Sensor;
import org.apache.commons.lang3.tuple.Pair;
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

    private EESimulator eeSimulator;

    //private AutoPilot autoPilot;

    private List<Bus> busList = new LinkedList<>();

    private List<Sensor> sensorList = new LinkedList<>();

    private List<VehicleActuator> actuatorList = new LinkedList<>();

    private List<Bridge> bridgeList = new LinkedList<>();

    double allowedVelocityByStreetType = 0.;


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
                    //exception?
            }

        }
    }


    /**
     * Constructor if no JSON file is available
     * @param eeSimulator   simulator of the vehicle
     * @param busStructure  bus structure. Each key is one bus on the car. The value for each key is a list of connected EEComponents to this bus/key.
     *                      Bridges between two buses have to be in at least one of the lists of connected components of the two buses.
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
                        //exception?
                }

            }
        }
    }


    /**
     * function that notifies all sensors to send their actual data to the bus
     * @param actualTime actual time of the simulation
     */
    public void notifySensors(Instant actualTime) {
        for (Sensor sensor : sensorList) {
            sensor.update(actualTime);
        }
    }

    /**
     * function that notifies all actuators to update
     * @param actualTime time the actuators get to update their value
     */
    //TODO: wert des updates nach update auf den bus schreiben
    public void notifyActuator(Instant actualTime){
        for (VehicleActuator actuator : actuatorList) {
            actuator.update(actualTime);
        }
    }



    public EESimulator getEeSimulator() {
        return eeSimulator;
    }

    public List<Bus> getBusList() {
        return busList;
    }

    public List<Sensor> getSensorList() {
        return sensorList;
    }

    public List<VehicleActuator> getActuatorList() {
        return actuatorList;
    }

    public List<Bridge> getBridgeList() { return bridgeList; }





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

}

/**
     * all data of a bus structure, including all connected actuators, buses and sensors
     */
class ParsableBusStructureProperties {

        private List<Pair<EEComponent, UUID>> busSystems = new LinkedList<>();

        public ParsableBusStructureProperties(EEVehicle vehicle) {

            for (Bus bus : vehicle.getBusList()) {
                busSystems.add(Pair.of(bus, bus.getId()));
                UUID actualId = bus.getId();

                //add all sensors connected to this bus
                for (Sensor sensor : vehicle.getSensorList()) {
                    if (!bus.getConnectedComponents().contains(Pair.of(sensor,actualId))) {
                        busSystems.add(Pair.of((EEComponent) sensor, actualId));
                    }
                }

                //add all actuator connected to this bus
                for (VehicleActuator actuator : vehicle.getActuatorList()) {
                    if (!bus.getConnectedComponents().contains(Pair.of(actuator, actualId))) {
                        busSystems.add(Pair.of(actuator, actualId));
                    }
                }

                //add all bridges connected to this bus
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

