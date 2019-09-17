/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.HardwareEmulatorInterface;

import org.apache.commons.math3.analysis.function.Add;
import org.apache.commons.math3.linear.RealVector;
import simulation.bus.BusMessage;

import java.io.Serializable;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;



public class DirectModelAsEEComponent extends ImmutableEEComponent {
    private static final int MAX_TRAJECTORY_LENGTH = 100;

    private static final ArrayList<BusEntry> STANDARD_SUBSCRIBED_MESSAGES = new ArrayList<BusEntry>() {{
    	add(BusEntry.SENSOR_VELOCITY);
    	add(BusEntry.SENSOR_GPS_COORDINATES);
        add(BusEntry.SENSOR_COMPASS);
        add(BusEntry.ACTUATOR_ENGINE_CURRENT);
        add(BusEntry.ACTUATOR_BRAKE_CURRENT);
        add(BusEntry.ACTUATOR_STEERING_CURRENT);
        add(BusEntry.PLANNED_TRAJECTORY_X);
        add(BusEntry.PLANNED_TRAJECTORY_Y);
    }};
    
    public static final ArrayList<BusEntry> MASSPOINT_OUTPUT_MESSAGES = new ArrayList<BusEntry>() {{
    	add(BusEntry.ACTUATOR_STEERING);
    	add(BusEntry.ACTUATOR_BRAKE);
        add(BusEntry.ACTUATOR_ENGINE);
    }};
    
    HardwareEmulatorInterface model_server;
    int model_id;
    HashMap<String, Serializable> inputs = new HashMap<String, Serializable>();
    HashMap<String, Serializable>  outputs = new HashMap<String, Serializable>();
    Optional<Object> trajectoryX = Optional.empty();
    Optional<Object> trajectoryY = Optional.empty();
    Instant lastTime = Instant.EPOCH;


    public DirectModelAsEEComponent(HardwareEmulatorInterface model_server, String autopilot_config, EESimulator simulator, HashMap<BusEntry, List<EEComponent>> targetsByMessageId) throws Exception {
        super(simulator, EEComponentType.AUTOPILOT, STANDARD_SUBSCRIBED_MESSAGES, targetsByMessageId);
        this.model_server = model_server;
        this.model_id = model_server.alloc_autopilot(autopilot_config);
        if (this.model_id < 0){
            String error_msg = model_server.query("get_error_msg");
            throw new Exception("Error allocating autopilot. Config:\n"+autopilot_config+"\n"+error_msg);
        }
        EEDiscreteEvent nextExecute = new ControllerExecuteEvent(simulator.getSimulationTime().plusMillis(30), this);
        simulator.addEvent(nextExecute);
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

    /*
    TODO: change time of nextExecute event
     */


    @Override
    public void processEvent(EEDiscreteEvent event) {
        if (event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
            BusMessage currentMessage = (BusMessage) event;

            switch (currentMessage.getMessageID()) {
                case SENSOR_VELOCITY:
                    double currentVelocity = (double) currentMessage.getMessage();
                    this.inputs.put("currentVelocity", currentVelocity);
                    break;
                case SENSOR_GPS_COORDINATES:
                    double x = ((RealVector) currentMessage.getMessage()).getEntry(0);
                    double y = ((RealVector) currentMessage.getMessage()).getEntry(1);
                    this.inputs.put("x", x);
                    this.inputs.put("y", y);
                    break;
                case SENSOR_COMPASS:
                    double compass = (double) currentMessage.getMessage();
                    this.inputs.put("compass", compass);
                    break;
                case ACTUATOR_ENGINE_CURRENT:
                    double engine = (double) currentMessage.getMessage();
                    this.inputs.put("currentEngine", engine);
                    break;
                case ACTUATOR_BRAKE_CURRENT:
                    double brakes = (double) currentMessage.getMessage();
                    this.inputs.put("currentBrakes", brakes);
                    break;
                case ACTUATOR_STEERING_CURRENT:
                    double steering = (double) currentMessage.getMessage();
                    this.inputs.put("currentSteering", steering);
                    break;
                case PLANNED_TRAJECTORY_X:
                    this.trajectoryX = Optional.of(currentMessage.getMessage());
                    break;
                case PLANNED_TRAJECTORY_Y:
                    this.trajectoryY = Optional.of(currentMessage.getMessage());
                    break;
                default:
                    //noop;
            }
            if (trajectoryX.isPresent() && trajectoryY.isPresent()) {
                if (trajectoryX.get() instanceof List<?> && trajectoryY.get() instanceof List<?>) {
                    int trajectoryLength = processTrajectory((List<Double>) trajectoryX.get(), (List<Double>) trajectoryY.get());
                    this.inputs.put("trajectory_length", trajectoryLength);
                }
            }

        } else if (event.getEventType() == EEDiscreteEventTypeEnum.CONTROLLER_EXECUTE_EVENT) {
            double timeIncrement = Duration.between(event.getEventTime(), lastTime).toMillis();
            lastTime = event.getEventTime();
            this.inputs.put("timeIncrement", timeIncrement);
            
            this.outputs = model_server.old_execute(model_id, (Duration.between(lastTime, event.getEventTime()).toMillis() * 1000000), inputs);

            Object engine = outputs.get("engine");
            if (engine != null) {
                this.sendMessage(engine, 6, BusEntry.ACTUATOR_ENGINE, event.getEventTime());
            }

            Object brakes = outputs.get("brakes");
            if (brakes != null) {
                this.sendMessage(brakes, 6, BusEntry.ACTUATOR_BRAKE, event.getEventTime());
            }

            Object steering = outputs.get("steering");
            if (steering != null) {
                this.sendMessage(steering, 6, BusEntry.ACTUATOR_STEERING, event.getEventTime());
            }
            EEDiscreteEvent nextExecute = new ControllerExecuteEvent(event.getEventTime().plusMillis(30), this);
            this.getSimulator().addEvent(nextExecute);
        } else {
            throw new IllegalArgumentException("DirectModelAsEEComponent expect BusMessage or ControllerExecuteEvent as event. Event was: " + event.getEventType());
        }
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


}
