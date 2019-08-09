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
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller;


import javax.json.JsonObject;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopNotifiable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.HardwareEmulatorInterface;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui.Category;
import simulation.simulator.Simulator;
import simulation.vehicle.PhysicalVehicle;
import simulation.environment.WorldModel;
import simulation.environment.osm.ParserSettings;
import simulation.environment.weather.WeatherSettings;
import simulation.simulator.SimulationType;

import java.io.File;
import java.io.FileInputStream;
import java.util.List;

public class BasicController extends SimulationLoopNotifiable implements Runnable {


    @Override
    public void run() {
        startSimulation();
    }

    

    Simulator simulator;
    SimulationResult result;
    HardwareEmulatorInterface model_server;
    VehicleBuilder vehicle_builder;
    FileSystem file_system;

    public BasicController(FileSystem file_system) throws Exception {
        model_server = new HardwareEmulatorInterface("autopilots_folder=autopilots", "");
        result = new SimulationResult(file_system);
        vehicle_builder = new VehicleBuilder(model_server, result);
        simulator = null;
        this.file_system = file_system;
    }


    public void initFromJsonScenario(JsonObject scenario, String simulation_name) throws Exception {
        ScenarioConfig config = new ScenarioConfig(scenario);

        //Setup map
        File mapPath = file_system.getPath(Category.CategoryType.MAPS.id, config.map_name + ".osm");
        if (!mapPath.exists())
            throw new Exception("Could not find map: " + config.map_name + " for scenario " + config.scenario_name);

        MapData map = new MapData(config.map_name, file_system);

        
        //TODO free input stream when parsed
        WorldModel.init(
                new ParserSettings(new FileInputStream(mapPath), ParserSettings.ZCoordinates.ALLZERO),
                new WeatherSettings()
        );

        //Setup simulator
        Simulator.resetSimulator();
        simulator = Simulator.getSharedInstance();
        simulator.setSimulationDuration(config.max_sim_duration*1000);
        simulator.setSimulationLoopFrequency((int) config.sim_frequ);
        simulator.setSimulationType(SimulationType.SIMULATION_TYPE_FIXED_TIME);
        simulator.setStartDaytime(config.time.toDate());
        simulator.registerLoopObserver(this);

        result.init_for_simulation(simulation_name, config.map_name, false, simulator.getSimulationLoopFrequency());

        //Setup Vehicles
        for (VehicleConfig car : config.vehicles){
            vehicle_builder.createVehicle(car, map);
        }
    }

    public void startSimulation(){
        simulator.startSimulation();
        //Export simulation data when done:
        result.export();
    }

    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {
        this.result.setup_next_frame(totalTime);
        System.out.print("T: "+totalTime);
    }

    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {
        if (simulationObject instanceof PhysicalVehicle){
            PhysicalVehicle vehicle = ((PhysicalVehicle) simulationObject);
            if (this.result.add_car_frame(vehicle)){
                simulator.setSimulationDuration(1000+simulator.getSimulationTime());
            }
        }
    }

    @Override
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {
        result.simulation_start();
    }

}
