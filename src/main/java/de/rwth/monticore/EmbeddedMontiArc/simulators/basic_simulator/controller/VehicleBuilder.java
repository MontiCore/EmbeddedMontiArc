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

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;
import sensors.StaticPlannedTrajectoryXSensor;
import sensors.StaticPlannedTrajectoryYSensor;
import sensors.util.SensorUtil;
import simulation.simulator.Simulator;
import simulation.vehicle.*;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.HardwareEmulatorInterface;

import java.util.*;

public class VehicleBuilder {
    public static class VehicleTrajectory {
        Point2D[] trajectory;
        Point2D start;
        Point2D target;
        public VehicleTrajectory(double[] start_coords, double[] target_coords, MapData map) throws Exception {
            start = map.conv.latlonToMeters(new Point2D(start_coords[0], start_coords[1]));
            target = map.conv.latlonToMeters(new Point2D(target_coords[0], target_coords[1]));

            trajectory = new Pathfinding(map).find_shortest_path(start, target);
            start = trajectory[0];
            target = trajectory[trajectory.length-1];
        }
    }

    

    


    HardwareEmulatorInterface model_server;
    SimulationResult result;

    public VehicleBuilder(HardwareEmulatorInterface model_server, SimulationResult result){
        this.model_server = model_server;
        this.result = result;
    }
    /*
        TODOS:
        - Seperate PhysicalVehicleBuilder and the parts the create the "Vehicle" itself.
        - Setup from config/simlang
     */
    public void createVehicle(VehicleConfig config, MapData map) throws Exception {

        PhysicalVehicleBuilder vehicleBuilder = getVehicleBuilder(config.physics_model);

        DirectModelAsFunctionBlock controller = new DirectModelAsFunctionBlock(model_server, config.autopilot_config);
        vehicleBuilder.setController(Optional.of(controller));
        vehicleBuilder.setControllerBus(Optional.of(new TempBus()));
        PhysicalVehicle physicalVehicle = vehicleBuilder.buildPhysicalVehicle();
        controller.set_vehicle(physicalVehicle);
        SensorUtil.sensorAdder(physicalVehicle);


        // compute trajectory for the vehicle
        VehicleTrajectory trajectory = new VehicleTrajectory(config.start_coords, config.target_coords, map);
        // put vehicle on its start point
        Simulator.getSharedInstance().registerAndPutObject(physicalVehicle,
                trajectory.start.getX(),
                trajectory.start.getY(),
                getRotation(trajectory.trajectory)
        );

        //  setup the vehicle's trajectory
        Map<String, List<Double>> trajectoryCoordinates = getTrajectoryCoordinates(trajectory.trajectory);
        Vehicle simVehicle = physicalVehicle.getSimulationVehicle();
        simVehicle.addSensor(new StaticPlannedTrajectoryXSensor(trajectoryCoordinates.get("x")));
        simVehicle.addSensor(new StaticPlannedTrajectoryYSensor(trajectoryCoordinates.get("y")));

        result.register_car(physicalVehicle.getId(), config.name, config.config, trajectory);
    }

    public static PhysicalVehicleBuilder getVehicleBuilder(String physics_model){
        if (physics_model.equalsIgnoreCase(VehicleConfig.PhysicsModel.MODELICA.name)){
            return new ModelicaPhysicalVehicleBuilder();
        } else if (physics_model.equalsIgnoreCase(VehicleConfig.PhysicsModel.MODELICA.name)){
            return new MassPointPhysicalVehicleBuilder();
        }
        //Allow the masspoint physical vehicle as DEFAULT
        return new MassPointPhysicalVehicleBuilder();
    }

    public static double getRotation(Point2D[] trajectory) {
        if (trajectory.length < 2) return 0d;
        //compare first two vectors, to get correct start rotation
        Point2D v1pos = trajectory[0];
        Point2D v2pos = trajectory[1];

        //slightly change the rotation from perfect straight line,
        //otherwise the simulator calculates strange starting trajectory
        return Math.atan2(v1pos.getX() - v2pos.getX(), v2pos.getY() - v1pos.getY()) -0.35d;
    }

    public static Map<String, List<Double>> getTrajectoryCoordinates(Point2D[] path) {
        Map<String, List<Double>> result = new HashMap<String, List<Double>>();

        List<Double> x = new ArrayList<Double>();
        List<Double> y = new ArrayList<Double>();
        for (Point2D p : path) {
            x.add(p.getX());
            y.add(p.getY());
        }
        result.put("x", x);
        result.put("y", y);
        return result;
    }



}
