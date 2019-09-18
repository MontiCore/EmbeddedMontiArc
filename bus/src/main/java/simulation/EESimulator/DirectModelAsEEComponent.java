/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator;
import org.apache.commons.math3.analysis.function.Add;
import org.apache.commons.math3.linear.RealVector;
import simulation.bus.Bus;
import simulation.bus.BusMessage;

import java.io.Serializable;
import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static com.vividsolutions.jts.util.Memory.free;


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
    
    boolean newInputs = false;
    
    boolean wakeUpNeeded = false;
    
    Duration cycleTime;
    
    HardwareEmulatorInterface modelServer;
    int modelId;
    HashMap<String, Serializable> inputs = new HashMap<String, Serializable>();
    HashMap<String, Serializable>  outputs = new HashMap<String, Serializable>();
    Optional<Object> trajectoryX = Optional.empty();
    Optional<Object> trajectoryY = Optional.empty();
    Instant lastTime = Instant.EPOCH;



    public static DirectModelAsEEComponent createDirectModelAsEEComponent(Bus bus) {
        return createDirectModelAsEEComponent(Collections.singletonList(bus));
    }

    public static DirectModelAsEEComponent createDirectModelAsEEComponent(List<Bus> buses) {
        HashMap<BusEntry, List<EEComponent>> targetsByMessageId = new HashMap<>();
        targetsByMessageId.put(BusEntry.ACTUATOR_BRAKE, new LinkedList<>());
        targetsByMessageId.put(BusEntry.ACTUATOR_STEERING, new LinkedList<>());
        targetsByMessageId.put(BusEntry.ACTUATOR_ENGINE, new LinkedList<>());
        for (Bus bus : buses) {
            if (bus.subscribedMessages.contains(BusEntry.ACTUATOR_ENGINE)) {
                targetsByMessageId.get(BusEntry.ACTUATOR_ENGINE).add(bus);
            }
            if (bus.subscribedMessages.contains(BusEntry.ACTUATOR_STEERING)) {
                targetsByMessageId.get(BusEntry.ACTUATOR_STEERING).add(bus);
            }
            if (bus.subscribedMessages.contains(BusEntry.ACTUATOR_BRAKE)) {
                targetsByMessageId.get(BusEntry.ACTUATOR_BRAKE).add(bus);
            }
        }
        return new DirectModelAsEEComponent(buses.get(0).getSimulator(), targetsByMessageId);
    }


    public DirectModelAsEEComponent(EESimulator simulator, HashMap<BusEntry, List<EEComponent>> targetsByMessageId, Duration cycleTime) {
        super(simulator, EEComponentType.AUTOPILOT, STANDARD_SUBSCRIBED_MESSAGES, targetsByMessageId);
        this.cycleTime = cycleTime;
        //add controller execute event
        simulator.addEvent(new ControllerExecuteEvent(simulator.getSimulationTime().plus(cycleTime), this));

    
    public DirectModelAsEEComponent(HardwareEmulatorInterface modelServer, String autopilotConfig, EESimulator simulator, HashMap<BusEntry, List<EEComponent>> targetsByMessageId) throws Exception {
    	this(modelServer, autopilotConfig, Duration.ofMillis(30), simulator, targetsByMessageId);
    }


    public void setCycleTime(Duration cycleTime) {
    	this.cycleTime = cycleTime;
    }
    
    public Duration getCycleTime() {
    	return cycleTime;
    }
    
    
    public void free(){
        if (modelId >= 0){
            modelServer.free_autopilot(modelId);
            modelId = -1;
        }
    }

    public void initializeController(HardwareEmulatorInterface model_server, String autopilot_config) throws Exception {
        this.model_server = model_server;
        this.model_id = model_server.alloc_autopilot(autopilot_config);
        if (this.model_id < 0){
            String error_msg = model_server.query("get_error_msg");
            throw new Exception("Error allocating autopilot. Config:\n"+autopilot_config+"\n"+error_msg);
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
        //buffer new values arrived by busMessage
        if (event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
            BusMessage currentMessage = (BusMessage) event;
            newInputs = true;
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
                    throw new IllegalArgumentException("Received unsubscribed Message. Message type was: " + currentMessage.getMessageID().toString());
            }
            if (trajectoryX.isPresent() && trajectoryY.isPresent()) {
                if (trajectoryX.get() instanceof List<?> && trajectoryY.get() instanceof List<?>) {
                    int trajectoryLength = processTrajectory((List<Double>) trajectoryX.get(), (List<Double>) trajectoryY.get());
                    this.inputs.put("trajectory_length", trajectoryLength);
                }
            }

            if(wakeUpNeeded) {
            	executeController(event);
            }
        } else if (event.getEventType() == EEDiscreteEventTypeEnum.CONTROLLER_EXECUTE_EVENT) {
            if(newInputs) {
            	executeController(event);
            } 
            else {
            	wakeUpNeeded = true;
            }
        } else {
            throw new IllegalArgumentException("DirectModelAsEEComponent expect BusMessage or ControllerExecuteEvent as event. Event was: " + event.getEventType());
        }
    }

	private void executeController(EEDiscreteEvent event) {
		double timeIncrement = Duration.between(event.getEventTime(), lastTime).toMillis();
		lastTime = event.getEventTime();
		this.inputs.put("timeIncrement", timeIncrement);
		
		Duration delay = modelServer.time_execute(modelId, Duration.between(lastTime, event.getEventTime()));
		this.outputs = modelServer.get_outputs(modelId);
		Instant finishTime = event.getEventTime().plus(delay);
		
		Object engine = outputs.get("engine");
		if (engine != null) {
		    this.sendMessage(engine, 6, BusEntry.ACTUATOR_ENGINE, finishTime);
		}

		Object brakes = outputs.get("brakes");
		if (brakes != null) {
		    this.sendMessage(brakes, 6, BusEntry.ACTUATOR_BRAKE, finishTime);
		}

		Object steering = outputs.get("steering");
		if (steering != null) {
		    this.sendMessage(steering, 6, BusEntry.ACTUATOR_STEERING, finishTime);
		}
		
		//set next execute event
		Instant nextExecuteTime = event.getEventTime().plus(cycleTime);
		if(finishTime.isBefore(nextExecuteTime)) {
			this.getSimulator().addEvent(new ControllerExecuteEvent(nextExecuteTime, this));
		}
		else {
			this.getSimulator().addEvent(new ControllerExecuteEvent(finishTime, this));
		}
		wakeUpNeeded = false;
		newInputs = false;
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
