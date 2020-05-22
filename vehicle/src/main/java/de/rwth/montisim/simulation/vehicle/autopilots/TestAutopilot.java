package de.rwth.montisim.simulation.vehicle.autopilots;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class TestAutopilot extends EEComponent {

    final MessageInformation velocityMsg;
    final MessageInformation steeringMsg;
    final MessageInformation accelMsg;
    final MessageInformation brakeMsg;

    final Duration computeTime;

    double currentVelocity = 0;
    final double turnAngle;
    final double targetVelocity; // km/h

    double previous_error = 0;
    double integral = 0;

    Instant lastTime = null;

    final double maxVehicleAccel;
    public static enum Mode {
        CIRCLE,
        START_STOP
    }
    boolean braking = false;

    final Mode mode;

    public TestAutopilot(Mode mode, EESimulator simulator, Duration computeTime,double targetVelocity, double turnAngle, double maxVehicleAccel) {
        super(simulator, "TestAutopilot", 0);
        this.velocityMsg = addInput("velocity", DataType.DOUBLE);
        this.steeringMsg = addOutput("set_steering", DataType.DOUBLE);
        this.accelMsg = addOutput("set_gas", DataType.DOUBLE);
        this.brakeMsg = addOutput("set_braking", DataType.DOUBLE);
        this.computeTime = computeTime;
        this.targetVelocity = targetVelocity;
        this.turnAngle = turnAngle;
        this.maxVehicleAccel = maxVehicleAccel;
        this.mode = mode;
    }

    public static TestAutopilot newCircleAutopilot(
        EESimulator simulator, Duration computeTime, double maxVehicleAccel, 
        double targetVelocity, double turnAngle){
            return new TestAutopilot(Mode.CIRCLE, simulator, computeTime, targetVelocity, turnAngle, maxVehicleAccel);
    }

    public static TestAutopilot newStartStopAutopilot(
        EESimulator simulator, Duration computeTime, double maxVehicleAccel, 
        double targetVelocity
    ) {
        return new TestAutopilot(Mode.START_STOP, simulator, computeTime, targetVelocity, 0, maxVehicleAccel);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        if (msg.msgId == velocityMsg.messageId) {
            currentVelocity = (Double)msg.message;
            // Trigger computation
            compute(msgRecvEvent.getEventTime());
        }
    }

    void compute(Instant startTime){
        Instant sendTime = startTime.plus(computeTime);
        if (mode == Mode.START_STOP){
            if (currentVelocity > targetVelocity*0.9 && !braking) {
                braking = true;
                System.out.println("Braking");
            }
        }
        if (braking){
            send(sendTime, new Message(accelMsg, 0.0, 8, this.id));
            send(sendTime, new Message(brakeMsg, 1.0, 8, this.id));
            return;
        }
        double P = 1;
        double I = 0;
        double D = 0.2;


        double dt = 0;
        if (lastTime == null){
            lastTime = startTime;
        } else {
            dt = Time.secondsFromDuration(Duration.between(lastTime, startTime));
        }
        // Simple PID for speed
        double error = targetVelocity - currentVelocity;
        integral += error * dt;
        double derivative = dt > 0 ? (error - previous_error) / dt : 0;
        double output = P*error + I*integral + D*derivative;
        output /= 3.6; // Convert to m/s related space
        previous_error = error;
        /*
        error := setpoint − measured_value
        integral := integral + error × dt
        derivative := (error − previous_error) / dt
        output := Kp × error + Ki × integral + Kd × derivative
        previous_error := error
        */
        double accel = output/maxVehicleAccel; // Convert to m/s related space + to [0:1] actuator range

        send(sendTime, new Message(steeringMsg, turnAngle, 8, this.id));
        send(sendTime, new Message(accelMsg, accel, 8, this.id));
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.COMPUTER;
    }
    
}