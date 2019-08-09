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

import java.util.Map;

import javax.json.*;

public class VehicleConfig {
    public enum Entries {
        NAME("name", false, "\"String\""),
        PHYSICS_MODEL("physics_model", false, "(masspoint | modelica)"),
        START_COORDS("start_coords", true, "[lat, lon]"),
        TARGET_COORDS("target_coords", true, "[lat, lon]"),
        AUTOPILOT("autopilot", true, "{ \"autopilot\": \"AutopilotFileName\", \"autopilot config\": \"See HardwareEmulator\"}");
        public String key_name;
        public boolean required;
        public String format;
        Entries(String key_name, boolean required, String format){
            this.key_name = key_name;
            this.required = required;
            this.format = format;
        }
    }

    public enum PhysicsModel {
        MODELICA("modelica"),
        MASSPOINT("masspoint");
        public String name;
        PhysicsModel(String name){
            this.name = name;
        }
    }


    public String name;
    public String physics_model;
    public double[] start_coords = new double[2];
    public double[] target_coords = new double[2];
    public String autopilot_config = "";
    public JsonObject config;

    public VehicleConfig(JsonObject config) throws Exception {
        this.config = config;
        for (Entries entry : Entries.values()){
            if (entry.required){
                if(!config.containsKey(entry.key_name)){
                    throw new Exception("Car configuration missing entry: " + entry.key_name + ": " + entry.format);
                }
            }
        }

        name = config.getString(Entries.NAME.key_name, "unknown");
        physics_model = config.getString(Entries.PHYSICS_MODEL.key_name);
        JsonObject auto = config.getJsonObject(Entries.AUTOPILOT.key_name);
        for (Map.Entry<String, JsonValue> entry : auto.entrySet()){
            if (entry.getValue().getValueType() == JsonValue.ValueType.STRING){
                autopilot_config += entry.getKey() + "=" + ((JsonString) entry.getValue()).getString() + "\n";
            } else {
                autopilot_config += entry.getKey() + "\n";
            }
        }

        parse_coords(config, Entries.START_COORDS.key_name, start_coords );
        parse_coords(config, Entries.TARGET_COORDS.key_name, target_coords );
    }

    public static void parse_coords(JsonObject config, String entry_name, double[] coords) throws Exception {
        int c = 0;
        JsonArray coords_entry = config.getJsonArray(entry_name);
        if (coords_entry.size() != coords.length){
            throw new Exception("Expected " + coords.length + " entries in the " + entry_name + " coords.");
        }
        for (JsonValue val : coords_entry){
            if (val.getValueType() != JsonValue.ValueType.NUMBER)
                throw new Exception("Expected number as entry in the " + entry_name + " coords.");
                coords[c] = ((JsonNumber)val).doubleValue();
            c++;
        }
    }

    
}