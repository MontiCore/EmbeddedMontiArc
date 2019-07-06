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
package de.rwth_aachen.se.montisim.simulators.basic_simulator.controller;

import commons.controller.commons.Vertex;
import functionBlock.ConnectionEntry;
import functionBlock.FunctionBlock;
import navigationBlock.components.FindPath;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import sensors.StaticPlannedTrajectoryXSensor;
import sensors.StaticPlannedTrajectoryYSensor;
import sensors.util.SensorUtil;
import simulation.environment.World;
import simulation.environment.WorldModel;
import simulation.environment.osm.ApproximateConverter;
import simulation.simulator.Simulator;
import simulation.vehicle.*;
import simulator.integration.HardwareEmulatorInterface;
import structures.Graph;

import javax.json.*;
import java.io.FileInputStream;
import java.util.*;

public class VehicleBuilder {
    public static class VehicleTrajectory {
        List<Vertex> trajectory;
        RealVector start;
        RealVector target;
        boolean notified;
        public VehicleTrajectory(JsonArray start_coords, JsonArray target_coords) throws Exception {
            notified = false;
            start = jsonCoordToRealVector(start_coords);
            target = jsonCoordToRealVector(target_coords);


            //Get nearest nodes from coordinates
            World world = WorldModel.getInstance();
            Graph graph = new Graph(world.getControllerMap().getAdjacencies(), true);

            double best_start = Double.MAX_VALUE, best_target = Double.MAX_VALUE;
            Vertex nearest_start = null, nearest_target = null;

            for (Vertex v: graph.getVertices()){
                double dist_start = v.getPosition().getDistance(start);
                if (dist_start < best_start){
                    best_start = dist_start;
                    nearest_start = v;
                }
                double dist_target = v.getPosition().getDistance(target);
                if (dist_target < best_target){
                    best_target = dist_target;
                    nearest_target = v;
                }
            }



            //Compute trajectory
            FunctionBlock findPath = new FindPath();
            Map<String, Object> inputs = new HashMap<String, Object>();
            inputs.put(ConnectionEntry.FIND_PATH_graph.toString(), graph);
            inputs.put(ConnectionEntry.FIND_PATH_start_vertex.toString(), nearest_start);
            inputs.put(ConnectionEntry.FIND_PATH_target_vertex.toString(), nearest_target);
            findPath.setInputs(inputs);
            findPath.execute(0);

            trajectory = (List<Vertex>) findPath.getOutputs().get(ConnectionEntry.FIND_PATH_path.toString());

        }
    }

    public enum VehicleSettings {
        NAME("name"),
        PHYSICS_MODEL("physics_model"),
        START_COORDS("start_coords"),
        TARGET_COORDS("target_coords"),
        AUTOPILOT("autopilot");
        private String key_name;
        VehicleSettings(String key_name){
            this.key_name = key_name;
        }
        public String get_key_name(){
            return this.key_name;
        }
    }

    public enum VehiclePhysicsModel {
        MODELICA("modelica"),
        MASSPOINT("masspoint");
        private String name;
        VehiclePhysicsModel(String name){
            this.name = name;
        }
        public String get_name(){
            return this.name;
        }
    }


    HardwareEmulatorInterface model_server;

    public VehicleBuilder(HardwareEmulatorInterface model_server){
        this.model_server = model_server;
    }
    /*
        TODOS:
        - Seperate PhysicalVehicleBuilder and the parts the create the "Vehicle" itself.
        - Setup from config/simlang
     */
    public void createVehicle(JsonObject config, HashMap<Long, VehicleBuilder.VehicleTrajectory> vehicleTrajectories) throws Exception {
        String name = config.getString(VehicleSettings.NAME.get_key_name(), "unknown");
        String physics_model = config.getString(VehicleSettings.PHYSICS_MODEL.get_key_name());
        JsonObject auto = config.getJsonObject(VehicleSettings.AUTOPILOT.get_key_name());
        JsonArray start_coords = config.getJsonArray(VehicleSettings.START_COORDS.get_key_name());
        if (start_coords == null || start_coords.size() != 3){
            throw new Exception("Simulation car: " + name + " missing start_coords ([x,y,z])");
        }
        JsonArray target_coords = config.getJsonArray(VehicleSettings.TARGET_COORDS.get_key_name());
        if (target_coords == null || target_coords.size() != 3){
            throw new Exception("Simulation car: " + name + " missing target_coords ([x,y,z])");
        }


        PhysicalVehicleBuilder vehicleBuilder;

        //Set physical model
        if (physics_model.equalsIgnoreCase(VehiclePhysicsModel.MODELICA.get_name())){
            vehicleBuilder = new ModelicaPhysicalVehicleBuilder();
        } else if (physics_model.equalsIgnoreCase(VehiclePhysicsModel.MASSPOINT.get_name())){
            vehicleBuilder = new MassPointPhysicalVehicleBuilder();
        } else {
            vehicleBuilder = new MassPointPhysicalVehicleBuilder();
        }

        //Get the autopilot configuration
        String autopilot_config = "";
        for (Map.Entry<String, JsonValue> entry : auto.entrySet()){
            if (entry.getValue().getValueType() == JsonValue.ValueType.STRING){
                autopilot_config += entry.getKey() + "=" + ((JsonString) entry.getValue()).getString() + "\n";
            } else {
                autopilot_config += entry.getKey() + "\n";
            }
        }

        DirectModelAsFunctionBlock controller = new DirectModelAsFunctionBlock(model_server, autopilot_config);
        vehicleBuilder.setController(Optional.of(controller));
        vehicleBuilder.setControllerBus(Optional.of(new TempBus()));
        PhysicalVehicle physicalVehicle = vehicleBuilder.buildPhysicalVehicle();
        controller.set_vehicle(physicalVehicle);
        SensorUtil.sensorAdder(physicalVehicle);


        // compute trajectory for the vehicle
        VehicleTrajectory trajectory = new VehicleTrajectory(start_coords, target_coords);
        // put vehicle on its start point
        Simulator.getSharedInstance().registerAndPutObject(physicalVehicle,
                trajectory.start.getEntry(0),
                trajectory.start.getEntry(1),
                getRotation(trajectory.trajectory)
        );

        //  setup the vehicle's trajectory
        Map<String, List<Double>> trajectoryCoordinates = getTrajectoryCoordinates(trajectory.trajectory);
        Vehicle simVehicle = physicalVehicle.getSimulationVehicle();
        simVehicle.addSensor(new StaticPlannedTrajectoryXSensor(trajectoryCoordinates.get("x")));
        simVehicle.addSensor(new StaticPlannedTrajectoryYSensor(trajectoryCoordinates.get("y")));

        vehicleTrajectories.put(physicalVehicle.getId(), trajectory);
        // add the vehicle to observer
        //this.simulatonObserver.addVehicle(vehicle.getId(), trajectory);
    }

    public static double getRotation(List<Vertex> trajectory) {
        if (trajectory.size() < 2) return 0d;
        //compare first two vectors, to get correct start rotation
        double[] v1pos = trajectory.get(0).getPosition().toArray();
        double[] v2pos = trajectory.get(1).getPosition().toArray();

        //slightly change the rotation from perfect straight line,
        //otherwise the simulator calculates strange starting trajectory
        return Math.atan2(v1pos[0] - v2pos[0], v2pos[1] - v1pos[1]) -0.35d;
    }

    public static Map<String, List<Double>> getTrajectoryCoordinates(List<Vertex> trajectory) {
        Map<String, List<Double>> result = new HashMap<String, List<Double>>();

        List<Double> x = new ArrayList<Double>();
        List<Double> y = new ArrayList<Double>();
        for (Vertex v : trajectory) {
            double[] pos = v.getPosition().toArray();
            x.add(pos[0]);
            y.add(pos[1]);
        }
        result.put("x", x);
        result.put("y", y);

        return result;
    }

    private static RealVector jsonCoordToRealVector(JsonArray coords) throws Exception {
        //Json coords as [Latitude, Longitude]
        double[] res = new double[3];
        int c = 0;
        for (JsonValue val : coords){
            if (val.getValueType() != JsonValue.ValueType.NUMBER)
                throw new Exception("Expected number as start_coords entry.");
            res[c] = ((JsonNumber)val).doubleValue();
            c++;
        }
        ApproximateConverter cvt = WorldModel.getInstance().getParser().getConverter().getApproximateConverter();
        return new ArrayRealVector(new double[]{
                cvt.convertLongToMeters(res[1], res[0]),
                cvt.convertLatToMeters(res[0]),
                0
        }, false);
    }



}
