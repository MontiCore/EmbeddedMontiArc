package de.rwth_aachen.se.montisim.simulators.basic_simulator.controller;

import commons.simulation.SimulationLoopExecutable;
import commons.simulation.SimulationLoopNotifiable;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.gui.Category;
import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;
import simulation.environment.WorldModel;
import simulation.environment.osm.ParserSettings;
import simulation.environment.weather.WeatherSettings;
import simulation.simulator.SimulationType;
import simulation.simulator.Simulator;
import simulation.vehicle.PhysicalVehicle;
import simulator.integration.HardwareEmulatorInterface;

import javax.json.JsonArray;
import javax.json.JsonNumber;
import javax.json.JsonObject;
import javax.json.JsonValue;
import java.io.File;
import java.io.FileInputStream;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.List;

public class BasicController implements SimulationLoopNotifiable, Runnable {


    @Override
    public void run() {
        startSimulation();
    }

    public enum ScenarioSettings {
        NAME("name"),
        MAX_SIM_DURATION("max_sim_duration"),
        SIMULATION_FREQUENCY("simulation_frequency"),
        TIME("time"),
        PEDESTRIAN_DENSITY("pedestrian_density"),
        CARS("cars"),
        MAP_NAME("map_name");


        private String key_name;
        ScenarioSettings(String key_name){
            this.key_name = key_name;
        }
        public String get_key_name(){
            return this.key_name;
        }
    }

    Simulator simulator;
    HardwareEmulatorInterface model_server;
    VehicleBuilder vehicle_builder;
    FileSystem file_system;

    HashMap<Long, VehicleBuilder.VehicleTrajectory> vehicleTrajectories = new HashMap<>();

    public BasicController(FileSystem file_system) throws Exception {
        String[] args = new String[2];
        model_server = new HardwareEmulatorInterface("autopilots_folder=autopilots", "");
        vehicle_builder = new VehicleBuilder(model_server);
        simulator = null;
        this.file_system = file_system;
    }


    public void initFromJsonScenario(JsonObject config) throws Exception {
        JsonNumber p_d = config.getJsonNumber(ScenarioSettings.PEDESTRIAN_DENSITY.get_key_name());
        double pedestrian_density = p_d != null ? p_d.doubleValue() : 0;
        int max_sim_duration = config.getInt(ScenarioSettings.MAX_SIM_DURATION.get_key_name(), 60);
        String scenario_name = config.getString(ScenarioSettings.NAME.get_key_name(), "default");
        JsonNumber s_f = config.getJsonNumber(ScenarioSettings.SIMULATION_FREQUENCY.get_key_name());
        double sim_frequ = s_f != null ? s_f.doubleValue() : 30;
        String time = config.getString(ScenarioSettings.TIME.get_key_name(), "12:00");
        String map_name = config.getString(ScenarioSettings.MAP_NAME.get_key_name());

        //Check cars
        JsonArray cars = config.getJsonArray(ScenarioSettings.CARS.get_key_name());
        if (cars == null || cars.size() == 0){
            throw new Exception("No cars specified in scenario " + scenario_name);
        }


        //Setup map
        File mapPath = file_system.getPath(Category.CategoryType.MAPS.id, map_name + ".osm");
        if (!mapPath.exists())
            throw new Exception("Could not find map: " + map_name + " for scenario " + scenario_name);

        //TODO free input stream when parsed
        WorldModel.init(
                new ParserSettings(new FileInputStream(mapPath), ParserSettings.ZCoordinates.ALLZERO),
                new WeatherSettings()
        );


        //Setup simulator
        Simulator.resetSimulator();
        simulator = Simulator.getSharedInstance();
        simulator.registerLoopObserver(this);
        simulator.setSimulationDuration(max_sim_duration*1000);
        simulator.setSimulationLoopFrequency((int) sim_frequ);
        simulator.setSimulationType(SimulationType.SIMULATION_TYPE_FIXED_TIME);
        org.joda.time.format.DateTimeFormatter formatter = DateTimeFormat.forPattern("HH:mm");
        DateTime dt = formatter.parseDateTime(time);
        simulator.setStartDaytime(dt.toDate());

        //Setup Vehicles
        vehicleTrajectories.clear();
        for (JsonValue car : cars){
            if (car.getValueType() != JsonValue.ValueType.OBJECT)
                throw new Exception("Expected Json object as car in scenario " + scenario_name);
            JsonObject car_config = (JsonObject) car;
            vehicle_builder.createVehicle(car_config, vehicleTrajectories);
        }
    }

    public void startSimulation(){
        simulator.startSimulation();
    }



    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {

    }

    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {

    }

    @Override
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {

    }

    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {
        PhysicalVehicle vehicle = ((PhysicalVehicle) simulationObject);
        double[] position = vehicle.getPosition().toArray();
        double longitude = position[0];
        double latitude = position[1];
        System.out.println("\tPosition of car: [" +latitude + ", " + longitude + "]");

        //Temporary hack to stop simulation when vehicle attained its target
        VehicleBuilder.VehicleTrajectory traj = vehicleTrajectories.get(vehicle.getId());
        if (!traj.notified){
            if (traj.target.getDistance(vehicle.getPosition()) < 1){
                System.out.println("\tCar at target");
                simulator.setSimulationDuration(1000+simulator.getSimulationTime());
                traj.notified = true;
            }
        }

    }

    @Override
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {

    }

    @Override
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, long totalTime) {

    }


}
