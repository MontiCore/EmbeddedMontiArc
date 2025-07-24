/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.simulation.commons.Inspectable;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueCompass;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class RLAutopilot extends EEComponent implements Inspectable {
    transient final RLAutopilotProperties properties;

    transient int velocityMsg;
    transient int positionMsg;
    transient int compassMsg;

    transient int trajLengthMsg;
    transient int trajXMsg;
    transient int trajYMsg;

    transient int steeringMsg;
    transient int accelMsg;
    transient int brakeMsg;

    public double currentVelocity = 0;
    public Vec2 currentPosition = null;
    public double currentCompass = Double.NaN;

    double newTrajX[] = null;
    public int newTrajLength = 0;
    public int trajLength = 0;
    public double trajX[] = null;
    public double trajY[] = null;

    public double currentGas = 0;
    public double currentSteering = 0;
    public double currentBrakes = 0;


    //these values get set by RLSimulationHandler
    public double brakeOutput = 0;
    public double turnOutput = 0;
    public double speedOutput = 0;

    //action and state as arrays for more flexibility
    public float[] state = null;
    public float[] action = null;

    //array with all state values that get sent
    public float[] statePacket = new float[25];

    public RLAutopilot(RLAutopilotProperties properties, EESystem eeSystem) {
        super(properties, eeSystem);
        this.properties = properties;

        this.velocityMsg = addPort(PortInformation.newRequiredInputDataPort(TrueVelocity.VALUE_NAME, TrueVelocity.TYPE, false));
        this.positionMsg = addPort(PortInformation.newRequiredInputDataPort(TruePosition.VALUE_NAME, TruePosition.TYPE, false));
        this.compassMsg = addPort(PortInformation.newRequiredInputDataPort(TrueCompass.VALUE_NAME, TrueCompass.TYPE, false));

        this.trajLengthMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_LENGTH_MSG, BasicType.N, false));
        this.trajXMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_X_MSG, Navigation.TRAJECTORY_X_TYPE, false));
        this.trajYMsg = addPort(PortInformation.newRequiredInputDataPort(Navigation.TRAJECTORY_Y_MSG, Navigation.TRAJECTORY_Y_TYPE, false));

        this.steeringMsg = addPort(PortInformation.newRequiredOutputDataPort(Actuator.SETTER_PREFIX + PowerTrainProperties.STEERING_VALUE_NAME,
                BasicType.DOUBLE));
        this.accelMsg = addPort(PortInformation.newRequiredOutputDataPort(Actuator.SETTER_PREFIX + PowerTrainProperties.GAS_VALUE_NAME, BasicType.DOUBLE));
        this.brakeMsg = addPort(PortInformation.newRequiredOutputDataPort(Actuator.SETTER_PREFIX + PowerTrainProperties.BRAKING_VALUE_NAME, BasicType.DOUBLE));

    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        if (msg.isMsg(velocityMsg)) {
            currentVelocity = (Double) msg.message;
        } else if (msg.isMsg(positionMsg)) {
            currentPosition = (Vec2) msg.message;
            // Trigger computation
            compute(msgRecvEvent.getEventTime());
        } else if (msg.isMsg(compassMsg)) {
            currentCompass = (Double) msg.message;
        } else if (msg.isMsg(trajLengthMsg)) {
            newTrajLength = (int) msg.message;
        } else if (msg.isMsg(trajXMsg)) {
            newTrajX = (double[]) msg.message;
        } else if (msg.isMsg(trajYMsg)) {
            // Assumes the y positions array of a new trajectory always arrives last
            trajY = (double[]) msg.message;
            trajLength = newTrajLength;
            trajX = newTrajX;
        }
    }

    public void updateStatePacket() {
		for (int i = 0; i < this.statePacket.length; i++) {
			this.statePacket[i] = 0.f;
		}
		for (int i = 0; i < this.trajLength; i++) {
			this.statePacket[i] = (float) this.trajX[i];
			this.statePacket[10 + i] = (float) this.trajY[i];
		}
		this.statePacket[20] = (float) this.trajLength;
        this.statePacket[21] = (float) currentPosition.at(0);
        this.statePacket[22] = (float) currentPosition.at(1);
        this.statePacket[23] = (float) currentCompass;
        this.statePacket[24] = (float) currentVelocity;
    }

    public float[] getStatePacket() {
        return this.statePacket;
    }

    public double getCurrentSteering() {
        return this.currentSteering;
    }

    public double getCurrentGas() {
        return this.currentGas;
    }

    public double getCurrentBrakes() {
        return this.currentBrakes;
    }

    void setGas(Instant sendTime, double val) {
        this.currentGas = val;
        sendMessage(sendTime, accelMsg, val);
    }

    void setBrakes(Instant sendTime, double val) {
        this.currentBrakes = val;
        sendMessage(sendTime, brakeMsg, val);
    }

    void setSteering(Instant sendTime, double val) {
        this.currentSteering = val;
        sendMessage(sendTime, steeringMsg, val);
    }

    void compute(Instant startTime) {
        //get action values from action array
        Instant sendTime = startTime.plus(properties.compute_time);
        if (action != null) {
            speedOutput = (double) (action[0] + 1) / 2;
            brakeOutput = (double) (action[1] + 1) / 2;

            turnOutput = (double) action[2] * 30;
        }

        setState();
        setSteering(sendTime, turnOutput);
        setGas(sendTime, speedOutput);
        setBrakes(sendTime, brakeOutput);
    }

    private void setState() {
        state = new float[25];
        if (trajX != null) {
            for (int i = 0; i < trajLength; i++) {
                state[i] = (float) trajX[i];
            }
        } else return;
        for (int i = 0; i < 10 - trajLength; i++) {
            state[trajLength + i] = 0f;
        }
        if (trajY != null) {
            for (int i = 10; i < 10 + trajLength; i++) {
                state[i] = (float) trajY[i - 10];
            }
        }
        for (int i = 10 + trajLength; i < 20; i++) {
            state[i] = 0f;
        }

        state[20] = (float) trajLength;
        state[21] = (float) currentPosition.at(0);
        state[22] = (float) currentPosition.at(1);
        state[23] = (float) currentCompass;
        state[24] = (float) currentVelocity;

        return;
    }

    @Override
    public String getType() {
        return "autopilot";
    }

    void addEntry(List<String> entries, boolean output, PortInformation portInf, Object val) {
        String res = output ? "output: " : "input: ";
        res += portInf.name + ": ";
        if (val == null) entries.add(res + "null");
        else {
            List<String> toStr = portInf.data_type.toString(val);
            if (toStr.size() == 0) entries.add(res + "No toString()");
            if (toStr.size() == 1) entries.add(res + toStr.get(0));
            else {
                entries.add(res);
                for (String s : toStr) {
                    entries.add("  " + s);
                }
            }
        }
    }

    @Override
    public List<String> getEntries() {
        List<String> entries = new ArrayList<>();
        addEntry(entries, false, ports.elementAt(0), currentVelocity);
        addEntry(entries, false, ports.elementAt(1), currentPosition);
        addEntry(entries, false, ports.elementAt(2), currentCompass);
        addEntry(entries, false, ports.elementAt(3), trajLength);
        addEntry(entries, false, ports.elementAt(4), trajX);
        addEntry(entries, false, ports.elementAt(5), trajY);
        addEntry(entries, true, ports.elementAt(6), currentSteering);
        addEntry(entries, true, ports.elementAt(7), currentGas);
        addEntry(entries, true, ports.elementAt(8), currentBrakes);
        return entries;
    }

    @Override
    public String getName() {
        return properties.name;
    }

}