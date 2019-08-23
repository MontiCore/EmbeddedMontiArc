/* (c) https://github.com/MontiCore/monticore */
package de.rwth_aachen.se.montisim.simulators.basic_simulator.controller;

import commons.controller.commons.BusEntry;
import commons.controller.interfaces.FunctionBlockInterface;
import org.apache.commons.math3.linear.RealVector;
import simulation.vehicle.PhysicalVehicle;
import simulator.integration.HardwareEmulatorInterface;
import simulation.vehicle.VehicleActuatorType;

import java.io.Serializable;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class DirectModelAsFunctionBlock implements FunctionBlockInterface {
    private static final int MAX_TRAJECTORY_LENGTH = 100;

    HardwareEmulatorInterface model_server;
    int model_id;
    HashMap<String, Serializable> inputs = new HashMap<String, Serializable>();
    HashMap<String, Serializable>  outputs = new HashMap<String, Serializable>();
    PhysicalVehicle vehicle;


    public DirectModelAsFunctionBlock(HardwareEmulatorInterface model_server, String autopilot_config) throws Exception {
        this.model_server = model_server;
        this.model_id = model_server.alloc_autopilot(autopilot_config);
        if (this.model_id < 0){
            String error_msg = model_server.query("get_error_msg");
            throw new Exception("Error allocating autopilot. Config:\n"+autopilot_config+"\n"+error_msg);
        }
    }

    public void set_vehicle(PhysicalVehicle vehicle){
        this.vehicle = vehicle;
    }

    public void free(){
        if (model_id >= 0){
            model_server.free_autopilot(model_id);
            model_id = -1;
        }
    }

    @Override
    protected void finalize(){
        free();
    }

    @Override
    public void execute(double timeDelta) {
        this.outputs = model_server.old_execute(model_id, (long)(timeDelta*1000000), inputs);
    }
    /*
        TODOS:
        - Let the sensors interact with the bus
        - Let the actuators send their updated state
        - Use the same names for the messages and Autopilot ports


     */
    @Override
    public void setInputs(Map<String, Object> inputs) {
        this.inputs.clear();
        double timeIncrement = (Double) inputs.get(BusEntry.SIMULATION_DELTA_TIME.toString());
        double currentVelocity = (Double) inputs.get(BusEntry.SENSOR_VELOCITY.toString());
        RealVector gps = (RealVector) inputs.get(BusEntry.SENSOR_GPS_COORDINATES.toString());
        double x = gps.getEntry(0);
        double y = gps.getEntry(1);
        double compass = (Double) inputs.get(BusEntry.SENSOR_COMPASS.toString());
        double engine = getCurrentActuatorValue(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        double steering = getCurrentActuatorValue(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        double brakes = getCurrentActuatorValue(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        this.inputs.put("timeIncrement", timeIncrement);
        this.inputs.put("currentVelocity", currentVelocity);
        this.inputs.put("x", x);
        this.inputs.put("y", y);
        this.inputs.put("compass", compass);
        this.inputs.put("currentEngine", engine);
        this.inputs.put("currentSteering", steering);
        this.inputs.put("currentBrakes", brakes);
        Object ptx = inputs.get(BusEntry.PLANNED_TRAJECTORY_X.toString());
        Object pty = inputs.get(BusEntry.PLANNED_TRAJECTORY_Y.toString());
        if (ptx instanceof List<?> && pty instanceof List<?>) {
            int trajectoryLength = processTrajectory((List<Double>) ptx, (List<Double>) pty);
            this.inputs.put("trajectory_length", trajectoryLength);
        }
    }

    private double getCurrentActuatorValue(VehicleActuatorType type) {
        return vehicle.getSimulationVehicle().getVehicleActuator(type).getActuatorValueCurrent();
    }

    private int processTrajectory(List<Double> xCoords, List<Double> yCoords) {
        if (xCoords.size() != yCoords.size()) {
            return 0;
        }
        int len = xCoords.size();
        if (len > MAX_TRAJECTORY_LENGTH) {
            len = MAX_TRAJECTORY_LENGTH;
        }
        double[] trajectoryX = new double[len];
        double[] trajectoryY = new double[len];
        for (int i = 0; i < len; i++) {
            Double x = xCoords.get(i);
            Double y = yCoords.get(i);
            if (x == null || y == null) {
                return 0;
            }
            trajectoryX[i] = x;
            trajectoryY[i] = y;
        }
        inputs.put("trajectory_x", trajectoryX);
        inputs.put("trajectory_y", trajectoryY);
        return len;
    }

    @Override
    public Map<String, Object> getOutputs() {
        Map<String, Object> result = new HashMap<>();
        Object engine = outputs.get("engine");
        if (engine != null)
            result.put(BusEntry.ACTUATOR_ENGINE.toString(), ((double)engine) / 2.5); //TODO REMOVE SUPER JANKY TEMP TEST

        Object brakes = outputs.get("brakes");
        if (brakes != null)
            result.put(BusEntry.ACTUATOR_BRAKE.toString(), brakes);

        Object  steering  = outputs.get("steering");
        if ( steering  != null)
            result.put(BusEntry.ACTUATOR_STEERING.toString(),  steering );

        outputs.clear();
        return result;
    }

    @Override
    public String[] getImportNames() {
        return new String[0];
    }
}
