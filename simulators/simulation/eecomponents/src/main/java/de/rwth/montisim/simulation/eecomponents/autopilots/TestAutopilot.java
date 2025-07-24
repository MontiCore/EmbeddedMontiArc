/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.*;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;
import de.rwth.montisim.simulation.eesimulator.*;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.*;
import de.rwth.montisim.simulation.eecomponents.autopilots.TestAutopilotProperties.Mode;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class TestAutopilot extends EEComponent {
    transient final TestAutopilotProperties properties;

    transient int velocityMsg;
    transient int steeringMsg;
    transient int accelMsg;
    transient int brakeMsg;

    double currentVelocity = 0;
    double previous_error = 0;
    double integral = 0;

    Instant lastTime = null;

    boolean braking = false;

    transient final PID pid;

    public TestAutopilot(TestAutopilotProperties properties, EESystem eeSystem) {
        super(properties, eeSystem);
        this.properties = properties;
        this.pid = new PID(1, 0, 0.2);

        this.velocityMsg = addPort(PortInformation.newRequiredInputDataPort(TrueVelocity.VALUE_NAME, TrueVelocity.TYPE, false));
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
            // Trigger computation
            compute(msgRecvEvent.getEventTime());
        }
    }

    void compute(Instant startTime) {
        Instant sendTime = startTime.plus(properties.compute_time);
        if (properties.mode == Mode.START_STOP) {
            if (currentVelocity > properties.target_velocity * 0.9 && !braking) {
                braking = true;
                System.out.println("Braking");
            }
        }
        if (braking) {
            sendMessage(sendTime, accelMsg, 0.0);
            sendMessage(sendTime, brakeMsg, 1.0);
            return;
        }
        double dt = 0;
        if (lastTime == null) {
            lastTime = startTime;
        } else {
            dt = Time.secondsFromDuration(Duration.between(lastTime, startTime));
        }
        double output = pid.compute(dt, currentVelocity, properties.target_velocity);
        output /= 3.6; // Convert to m/s related space
        double accel = output / properties.maxVehicleAccel; // Convert to [0:1] actuator range

        sendMessage(sendTime, steeringMsg, properties.turn_angle);
        sendMessage(sendTime, accelMsg, accel);
    }

}