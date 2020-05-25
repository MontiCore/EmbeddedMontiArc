/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.autopilots;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.vehicle.autopilots.TestAutopilotProperties.Mode;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class TestAutopilot extends EEComponent {
    final TestAutopilotProperties properties;

    MessageInformation velocityMsg;
    MessageInformation positionMsg;
    MessageInformation steeringMsg;
    MessageInformation accelMsg;
    MessageInformation brakeMsg;


    double currentVelocity = 0;
    Vec2 currentPosition = new Vec2(0,0);
    double previous_error = 0;
    double integral = 0;

    Instant lastTime = null;

    boolean braking = false;

    PID pid;

    public TestAutopilot(TestAutopilotProperties properties) {
        super(properties);
        this.properties = properties;
        this.pid = new PID(1,0,0.2);
    }
    @Override
    protected void init() throws EEMessageTypeException {
        this.velocityMsg = addInput(TrueVelocity.VALUE_NAME, TrueVelocity.TYPE);
        this.positionMsg = addInput(TruePosition.VALUE_NAME, TruePosition.TYPE);
        this.steeringMsg = addOutput(Actuator.SETTER_PREFIX+PowerTrainProperties.STEERING_VALUE_NAME, DataType.DOUBLE);
        this.accelMsg = addOutput(Actuator.SETTER_PREFIX+PowerTrainProperties.GAS_VALUE_NAME, DataType.DOUBLE);
        this.brakeMsg = addOutput(Actuator.SETTER_PREFIX+PowerTrainProperties.BRAKING_VALUE_NAME, DataType.DOUBLE);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        if (msg.msgId == velocityMsg.messageId) {
            currentVelocity = (Double) msg.message;
            // Trigger computation
            compute(msgRecvEvent.getEventTime());
        } else if (msg.msgId == positionMsg.messageId) {
            currentPosition = (Vec2) msg.message;
        }
    }

    void compute(Instant startTime) {
        Instant sendTime = startTime.plus(properties.computeTime);
        if (properties.mode == Mode.START_STOP) {
            if (currentVelocity > properties.targetVelocity * 0.9 && !braking) {
                braking = true;
                System.out.println("Braking");
            }
        }
        if (braking) {
            send(sendTime, new Message(accelMsg, 0.0, accelMsg.type.dataSize, this.id));
            send(sendTime, new Message(brakeMsg, 1.0, brakeMsg.type.dataSize, this.id));
            return;
        }
        double dt = 0;
        if (lastTime == null) {
            lastTime = startTime;
        } else {
            dt = Time.secondsFromDuration(Duration.between(lastTime, startTime));
        }
        double output = pid.compute(dt, currentVelocity, properties.targetVelocity);
        output /= 3.6; // Convert to m/s related space
        double accel = output / properties.maxVehicleAccel; // Convert to [0:1] actuator range
        
        send(sendTime, new Message(steeringMsg, properties.turnAngle, steeringMsg.type.dataSize, this.id));
        send(sendTime, new Message(accelMsg, accel, accelMsg.type.dataSize, this.id));
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.COMPUTER;
    }

    
}