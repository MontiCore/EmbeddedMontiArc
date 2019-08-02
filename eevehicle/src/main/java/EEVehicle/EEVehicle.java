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
package EEVehicle;

import commons.simulation.IPhysicalVehicle;
import org.apache.commons.lang3.tuple.Pair;
import sensors.abstractsensors.AbstractSensor;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEComponentType;
import simulation.EESimulator.EESimulator;
import simulation.bus.Bus;
import simulation.bus.BusMessage;
import simulation.vehicle.*;
import sun.font.PhysicalFont;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static simulation.vehicle.Vehicle.*;
import static simulation.vehicle.VehicleActuatorType.*;

public class EEVehicle {

    private PhysicalVehicle physicalVehicle;

    private EESimulator eeSimulator;

    //private AutoPilot autoPilot;

    private List<Bus> busList = new LinkedList<>();

    private List<AbstractSensor> sensorList = new LinkedList<>();

    private List<VehicleActuator> actuatorList = new LinkedList<>();

    /*
    TODO:
            - rausschmeißen in neues Package
            - modelica und MassPoint mal untersuchen und gucken ob/wo actuator unterschiedlich
     */
    /**
     * Constructor of EEVehicle
     * @param eeSimulator simulator of the Vehicle
     * @param busSystems a HashMap of all bus systems on this vehicle. The keys of the HashMap are the bus systems on this vehicle, the entries for each key are the components connected to the bus
     * @param modelicaPhysicalVehicle true if physicalVehicle should be type of ModelicaPhysicalVehicle, false if physicalVehicle should be type of MassPointPhysicalVehicle
     */
    public EEVehicle(EESimulator eeSimulator, HashMap<Bus,List<EEComponent>> busSystems, boolean modelicaPhysicalVehicle) {
        PhysicalVehicleBuilder builder;
        if(modelicaPhysicalVehicle){
            builder = new ModelicaPhysicalVehicleBuilder();
        } else {
            builder = new MassPointPhysicalVehicleBuilder();
        }
        this.physicalVehicle = builder.buildPhysicalVehicle();
        this.eeSimulator = eeSimulator;

        Set<Bus> processedKeys = new HashSet<>();

        for (Bus actualBus : busSystems.keySet()) {
            busList.add(actualBus);
            for (EEComponent comp : busSystems.get(actualBus)) {
                switch (comp.getComponentType()){
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
                            bridge.update(actualBus, actualBus.getListenTo());
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

    /*
    TODO:   - change message length
            - sentTo wurde umgenannt
            - villt getMessageLength bei sensoren einbauen

     */
    /**
     * function that notify all sensors to send their actual data to the bus
     * @param timeDiff duration between last tick and actual tick
     * @param actualTime actual time of the simulation
     */
    public void notifySensors(Instant actualTime, Duration timeDiff){
        physicalVehicle.executeLoopIteration(timeDiff);
        for (AbstractSensor sensor : sensorList) {
            for (EEComponent target : sensor.getSendTo().get(sensor.getType())){
                BusMessage sensorMess = new BusMessage(sensor.getValue(), 6, sensor.getType(), actualTime, sensor.getID(), target);
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

}
