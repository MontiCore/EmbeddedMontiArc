/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.interfaces.*;
import de.rwth.monticore.EmbeddedMontiArc.simulators.hardware_emulator.config.ControllerConfig;
import de.rwth.montisim.commons.utils.Vec3;
import simulation.bus.Bus;
import simulation.bus.BusMessageEvent;

import java.io.Serializable;
import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static com.vividsolutions.jts.util.Memory.free;

/**
 * Autopilot that communicate directly with the HardwareEmulatorServer.
 * Controls the velocity and path of the vehicle.
 * Calculates actuator values for brakes, engine, steering.
 */
public class ControllerAsEEComponent extends ImmutableEEComponent {
    private static final int MAX_TRAJECTORY_LENGTH = 100;

    /**
     * Last values that were send indexed by the bus entry
     */
    private Map<Object, BusEntry> lastValueByMessageId = new HashMap<>();

    /**
     * Necessary messages that the controller needs
     */
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

    /**
     * Outputs when created for a MassPointVehicle
     */
    public static final ArrayList<BusEntry> MASSPOINT_OUTPUT_MESSAGES = new ArrayList<BusEntry>() {{
        add(BusEntry.ACTUATOR_STEERING);
        add(BusEntry.ACTUATOR_BRAKE);
        add(BusEntry.ACTUATOR_ENGINE);
    }};

    /**
     * Determines if the controller has received new inputs
     */
    boolean newInputs = false;

    /**
     * Determines if the controller needs to start the calculation again afer receiving the next message.
     */
    boolean wakeUpNeeded = false;

    /**
     * The duration of the an update cycle of this controller
     */
    Duration cycleTime;

    /**
     * Direct reference to the hardware emulator
     */
    SoftwareSimulator softwareSimulator;

    int modelId;

    HashMap<String, Serializable> inputs = new HashMap<String, Serializable>();
    HashMap<String, Serializable> outputs = new HashMap<String, Serializable>();

    Optional<Object> trajectoryX = Optional.empty();
    Optional<Object> trajectoryY = Optional.empty();

    /**
     * Time when the controller was executed last
     */
    Instant lastFinishTime = Instant.EPOCH;


    /**
     * Create a ControllerAsEEComponent that is connected to bus.
     * @param bus Bus that the ControllerAsEEComponent is connected to
     * @return ControllerAsEEComponent with default configuration
     */
    public static ControllerAsEEComponent createControllerAsEEComponent(Bus bus) {
        return createControllerAsEEComponent(Collections.singletonList(bus));
    }

    /**
     * Create a ControllerAsEEComponent that is connected to bus.
     * @param buses Buses that the ControllerAsEEComponent is connected to
     * @return ControllerAsEEComponent with default configuration
     */
    public static ControllerAsEEComponent createControllerAsEEComponent(List<Bus> buses) {
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
        return new ControllerAsEEComponent(buses.get(0).getSimulator(), targetsByMessageId);
    }


    public ControllerAsEEComponent(EESimulator simulator, HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
        super(simulator, EEComponentType.AUTOPILOT, STANDARD_SUBSCRIBED_MESSAGES, targetsByMessageId);
    }


    public void setCycleTime(Duration cycleTime) {
        this.cycleTime = cycleTime;
    }

    public Duration getCycleTime() {
        return cycleTime;
    }


    public void free() {
        try {
            softwareSimulator.free();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Set the SoftwareSimulator interface, create the autopilot and set the cycle time.
     * @param softwareSimManager
     * @param controllerConfig
     * @param cycleTime
     * @throws Exception
     */
    public void initializeController(SoftwareSimulatorManager softwareSimManager, ControllerConfig controllerConfig, Duration cycleTime) throws Exception {
        this.softwareSimulator = softwareSimManager.newSimulator(controllerConfig);
        this.cycleTime = cycleTime;
        //add controller execute event
        this.getSimulator().addEvent(new ControllerExecuteEvent(this.getSimulator().getSimulationTime().plus(cycleTime), this));
    }

    @Override
    protected void finalize() {
        free();
    }

    @Override
    public void processEvent(EEDiscreteEvent event) {
        //buffer new values arrived by busMessage
        if (event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
            BusMessageEvent currentMessage = (BusMessageEvent) event;
            newInputs = true;
            switch (currentMessage.getMessageID()) {
                case SENSOR_VELOCITY:
                    double currentVelocity = (double) currentMessage.getMessage();
                    this.inputs.put("currentVelocity", currentVelocity);
                    break;
                case SENSOR_GPS_COORDINATES:
                    double x = ((Vec3) currentMessage.getMessage()).getEntry(0);
                    double y = ((Vec3) currentMessage.getMessage()).getEntry(1);
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

            if (wakeUpNeeded) {
                try {
                    executeController(event);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        } else if (event.getEventType() == EEDiscreteEventTypeEnum.CONTROLLER_EXECUTE_EVENT) {
            if (newInputs) {
                try {
                    executeController(event);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            } else {
                wakeUpNeeded = true;
            }
        } else {
            throw new IllegalArgumentException("ControllerAsEEComponent expect BusMessageEvent or ControllerExecuteEvent as event. Event was: " + event.getEventType());
        }
    }

    /**
     * Execute the controller once. Add a ControllerExecuteEvent for the next cycle.
     * @param event
     */
    private void executeController(EEDiscreteEvent event) throws Exception {
        double timeIncrement = Duration.between(event.getEventTime(), lastFinishTime).toMillis();
        this.inputs.put("timeIncrement", timeIncrement);
        this.softwareSimulator.setInputs(inputs);
        Duration delay = this.softwareSimulator.runCycle();
        this.outputs = this.softwareSimulator.getOutputs();
        lastFinishTime = event.getEventTime().plus(delay);

        Object engine = outputs.get("engine");
        if (engine != null) {
            if (!lastValueByMessageId.containsKey(BusEntry.ACTUATOR_ENGINE) || !lastValueByMessageId.get(BusEntry.ACTUATOR_ENGINE).equals(engine)) {
                this.sendMessage(engine, 8, BusEntry.ACTUATOR_ENGINE, lastFinishTime);
            }
        }

        Object brakes = outputs.get("brakes");
        if (brakes != null) {
            if (!lastValueByMessageId.containsKey(BusEntry.ACTUATOR_BRAKE) || !lastValueByMessageId.get(BusEntry.ACTUATOR_BRAKE).equals(brakes)) {
                this.sendMessage(brakes, 8, BusEntry.ACTUATOR_BRAKE, lastFinishTime);
            }

        }

        Object steering = outputs.get("steering");
        if (steering != null) {
            if (!lastValueByMessageId.containsKey(BusEntry.ACTUATOR_STEERING) || !lastValueByMessageId.get(BusEntry.ACTUATOR_STEERING).equals(steering)) {
                this.sendMessage(steering, 8, BusEntry.ACTUATOR_STEERING, lastFinishTime);
            }
        }

        //set next execute event
        Instant nextExecuteTime = event.getEventTime().plus(cycleTime);
        if (lastFinishTime.isBefore(nextExecuteTime)) {
            this.getSimulator().addEvent(new ControllerExecuteEvent(nextExecuteTime, this));
        } else {
            this.getSimulator().addEvent(new ControllerExecuteEvent(lastFinishTime, this));
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
