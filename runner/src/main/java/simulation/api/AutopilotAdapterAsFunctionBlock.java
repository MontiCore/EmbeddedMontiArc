package simulation.api;

import commons.controller.commons.BusEntry;
import commons.controller.interfaces.FunctionBlockInterface;
import org.apache.commons.math3.linear.RealVector;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import simulation.vehicle.PhysicalVehicle;
import simulation.vehicle.VehicleActuatorType;

import java.io.Serializable;
import java.util.HashMap;
import java.util.List;
import java.util.Map;


public class AutopilotAdapterAsFunctionBlock implements commons.controller.interfaces.FunctionBlockInterface {

	private static final Logger LOG = LoggerFactory.getLogger(server.adapters.AutoPilot.AutopilotAdapterAsFunctionBlock.class);
    private static final int MAX_TRAJECTORY_LENGTH = 100;

    private final RMIClient client;
    private final double[] trajectoryX = new double[MAX_TRAJECTORY_LENGTH];
    private final double[] trajectoryY = new double[MAX_TRAJECTORY_LENGTH];
    private PhysicalVehicle vehicle;

    private HashMap<String, Serializable> modelInputs = new HashMap<String, Serializable>();
    private HashMap<String, Serializable>  modelOutputs = new HashMap<String, Serializable>();

    public AutopilotAdapterAsFunctionBlock(RMIClient client) {
    	this.client = client;
    }

    public void setVehicle(PhysicalVehicle vehicle) {
        this.vehicle = vehicle;
    }

    public PhysicalVehicle getVehicle() {
        return vehicle;
    }

    @Override
    public void execute(double timeDelta) {
        //PutAll in because the execute now only returns UPDATES: keep the old values if no update.
    	modelOutputs.putAll(client.oldExecute(modelInputs, timeDelta));
//        modelOutputs.putAll(client.oldExecute(modelInputs, 0));
    }

    @Override
    public void setInputs(Map<String, Object> inputs) {
        double timeIncrement = (Double) inputs.get(BusEntry.SIMULATION_DELTA_TIME.toString());
        double currentVelocity = (Double) inputs.get(BusEntry.SENSOR_VELOCITY.toString());
        RealVector gps = (RealVector) inputs.get(BusEntry.SENSOR_GPS_COORDINATES.toString());
        double x = gps.getEntry(0);
        double y = gps.getEntry(1);
        double compass = (Double) inputs.get(BusEntry.SENSOR_COMPASS.toString());
        double engine = getCurrentActuatorValue(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_MOTOR);
        double steering = getCurrentActuatorValue(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_STEERING);
        double brakes = getCurrentActuatorValue(VehicleActuatorType.VEHICLE_ACTUATOR_TYPE_BRAKES_BACK_LEFT);
        LOG.debug("velocity " + currentVelocity);
        LOG.debug("position " + x + " " + y);
        LOG.debug("actuation " + brakes + " " + engine + " " + steering);
        modelInputs.put("timeIncrement", timeIncrement);
        modelInputs.put("currentVelocity", currentVelocity);
        modelInputs.put("x", x);
        modelInputs.put("y", y);
        modelInputs.put("compass", compass);
        modelInputs.put("currentEngine", engine);
        modelInputs.put("currentSteering", steering);
        modelInputs.put("currentBrakes", brakes);
        Object ptx = inputs.get(BusEntry.PLANNED_TRAJECTORY_X.toString());
        Object pty = inputs.get(BusEntry.PLANNED_TRAJECTORY_Y.toString());
        if (ptx instanceof List<?> && pty instanceof List<?>) {
            int trajectoryLength = processTrajectory((List<Double>) ptx, (List<Double>) pty);
            modelInputs.put("trajectory_length", trajectoryLength);
        } else {
            LOG.error("planned trajectory is not provided");
            modelInputs.put("trajectory_length", 0);
        }
    }

    @Override
    public Map<String, Object> getOutputs() {
        Map<String, Object> result = new HashMap<>();
        Object engine = modelOutputs.get("engine");
        if (engine != null)
            result.put(BusEntry.ACTUATOR_ENGINE.toString(), engine);

        Object brakes = modelOutputs.get("brakes");
        if (brakes != null)
            result.put(BusEntry.ACTUATOR_BRAKE.toString(), brakes);

        Object  steering  = modelOutputs.get("steering");
        if ( steering  != null)
            result.put(BusEntry.ACTUATOR_STEERING.toString(),  steering );

        LOG.debug("control commands " + brakes + " " + engine + " " + steering);
        return result;
    }

    @Override
    public String[] getImportNames() {
        return new String[0];
    }

    private double getCurrentActuatorValue(VehicleActuatorType type) {
        return vehicle.getSimulationVehicle().getVehicleActuator(type).getActuatorValueCurrent();
    }

    private int processTrajectory(List<Double> xCoords, List<Double> yCoords) {
        if (xCoords.size() != yCoords.size()) {
            LOG.error(
                    "corrupt trajectory data: x has length "
                            + xCoords.size()
                            + " and y has length "
                            + yCoords.size()
            );
            return 0;
        }
        int len = xCoords.size();
        if (len > MAX_TRAJECTORY_LENGTH) {
            LOG.warn(
                    "trajectory is too long ("
                            + len
                            + " coordinates), only first "
                            + MAX_TRAJECTORY_LENGTH
                            + " coordinates will be processed"
            );
            len = MAX_TRAJECTORY_LENGTH;
        }
        for (int i = 0; i < len; i++) {
            Double x = xCoords.get(i);
            Double y = yCoords.get(i);
            if (x == null || y == null) {
                LOG.error(
                        "corrupt trajectory data: x = " + x
                                + " and y = " + y
                                + " at index i = " + i
                );
                return 0;
            }
            trajectoryX[i] = x;
            trajectoryY[i] = y;
        }
        logTrajectory(len);
        modelInputs.put("trajectory_x", trajectoryX);
        modelInputs.put("trajectory_y", trajectoryY);
        return len;
    }

    private void logTrajectory(int len) {
        StringBuilder sb = new StringBuilder("planned trajectory:");
        for (int i = 0; i < len; i++) {
            sb.append("   (");
            sb.append(trajectoryX[i]);
            sb.append(';');
            sb.append(trajectoryY[i]);
            sb.append(')');
        }
        LOG.debug(sb.toString());
    }
}