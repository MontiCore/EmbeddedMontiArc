/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller;

import java.util.ArrayList;
import java.util.List;

import javax.json.*;
import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;

public class ScenarioConfig {


    public enum Entries {
        TYPE("type", true, "scenario"),
        NAME("name", false, "\"String\""),
        MAX_SIM_DURATION("max_sim_duration", false, "Max duration in seconds."),
        SIMULATION_FREQUENCY("simulation_frequency", false, "Defaults to 30 Hz"),
        TIME("time", false, "HH:mm"),
        PEDESTRIAN_DENSITY("pedestrian_density", false, ""),
        CARS("cars", true, "List of simulation cars (See VehicleConfig)"),
        MAP_NAME("map_name", true, "Name of map in \"maps\" folder without file extension.");

        public String key_name;
        public boolean required;
        public String format;
        Entries(String key_name, boolean required, String format){
            this.key_name = key_name;
            this.required = required;
            this.format = format;
        }
    }

    public double pedestrian_density;
    public int max_sim_duration;
    public String scenario_name;
    public double sim_frequ;
    public DateTime time;
    public String map_name;

    public List<VehicleConfig> vehicles;

    public ScenarioConfig(JsonObject config) throws Exception {
        for (Entries entry : Entries.values()){
            if (entry.required){
                if(!config.containsKey(entry.key_name)){
                    throw new Exception("Scenario configuration missing entry: " + entry.key_name + ": " + entry.format);
                }
            }
        }
        if (!config.getString(Entries.TYPE.key_name, "").equalsIgnoreCase("scenario")){
            throw new Exception("File loaded as scenario not of scenario type.");
        }

        JsonNumber p_d = config.getJsonNumber(Entries.PEDESTRIAN_DENSITY.key_name);
        pedestrian_density = p_d != null ? p_d.doubleValue() : 0;
        max_sim_duration = config.getInt(Entries.MAX_SIM_DURATION.key_name, 60);
        scenario_name = config.getString(Entries.NAME.key_name, "default");
        JsonNumber s_f = config.getJsonNumber(Entries.SIMULATION_FREQUENCY.key_name);
        sim_frequ = s_f != null ? s_f.doubleValue() : 30;
        org.joda.time.format.DateTimeFormatter formatter = DateTimeFormat.forPattern("HH:mm");
        time = formatter.parseDateTime(config.getString(Entries.TIME.key_name, "12:00"));
        map_name = config.getString(Entries.MAP_NAME.key_name);

        //Check cars
        JsonArray cars = config.getJsonArray(Entries.CARS.key_name);
        if (cars.size() == 0){
            throw new Exception("No cars specified in scenario " + scenario_name);
        }
        vehicles = new ArrayList<VehicleConfig>(cars.size());

        for (JsonValue car : cars){
            if (car.getValueType() != JsonValue.ValueType.OBJECT)
                throw new Exception("Expected Json object as car in scenario " + scenario_name);
            JsonObject car_config = (JsonObject) car;
            vehicles.add(new VehicleConfig(car_config));
        }
    }
}