/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.simulator.autopilots;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.eecomponents.navigation.Navigation;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueCompass;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class JavaAutopilot extends EEComponent {
    final JavaAutopilotProperties properties;
    
    MessageInformation velocityMsg;
    MessageInformation positionMsg;
    MessageInformation compassMsg;

    
    MessageInformation trajXMsg;
    MessageInformation trajYMsg;

    MessageInformation steeringMsg;
    MessageInformation accelMsg;
    MessageInformation brakeMsg;
    
    double currentVelocity = 0;
    Vec2 currentPosition = null;
    double currentCompass = 0;

    double newTrajX[] = null;
    double trajX[] = null;
    double trajY[] = null;
    
    Instant lastTime = null;

    public JavaAutopilot(JavaAutopilotProperties properties) {
        super(properties);
        this.properties = properties;
    }
    
    @Override
    protected void init() throws EEMessageTypeException {
        this.velocityMsg = addInput(TrueVelocity.VALUE_NAME, TrueVelocity.TYPE);
        this.positionMsg = addInput(TruePosition.VALUE_NAME, TruePosition.TYPE);
        this.compassMsg = addInput(TrueCompass.VALUE_NAME, TrueCompass.TYPE);

        this.trajXMsg = addInput(Navigation.TRAJECTORY_X_MSG, Navigation.TRAJECTORY_X_TYPE);
        this.trajYMsg = addInput(Navigation.TRAJECTORY_Y_MSG, Navigation.TRAJECTORY_Y_TYPE);

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
        } else if (msg.msgId == positionMsg.messageId) {
            currentPosition = (Vec2) msg.message;
            compute(msgRecvEvent.getEventTime());
        } else if (msg.msgId == compassMsg.messageId) {
            currentCompass = (Double) msg.message;
        } else if (msg.msgId == trajXMsg.messageId){
            newTrajX = (double[]) msg.message;
        } else if (msg.msgId == trajYMsg.messageId){
            trajY = (double[]) msg.message;
            trajX = newTrajX;
        }
    }


    void compute(Instant startTime) {
        if (currentPosition == null) return;
        Instant sendTime = startTime.plus(properties.computeTime);

        int index = getNearestSegment(currentPosition);
        if (index < 0){
            // No trajectory -> Stay in place
            send(sendTime, new Message(accelMsg, 0.0, accelMsg.type.dataSize, this.id));
            send(sendTime, new Message(brakeMsg, 1.0, brakeMsg.type.dataSize, this.id));
            return;
        }
        
        if (trajX.length == 1){
            // Only one point -> orient towards it
            return;
        }



        // double dt = 0;
        // if (lastTime == null) {
        //     lastTime = startTime;
        // } else {
        //     dt = Time.secondsFromDuration(Duration.between(lastTime, startTime));
        // }
        // double output = pid.compute(dt, currentVelocity, properties.targetVelocity);
        // output /= 3.6; // Convert to m/s related space
        // double accel = output / properties.maxVehicleAccel; // Convert to [0:1] actuator range
        
        // send(sendTime, new Message(steeringMsg, properties.turnAngle, 8, this.id));
        // send(sendTime, new Message(accelMsg, accel, 8, this.id));
    }

    
    private int getNearestSegment(Vec2 pos) {
        if (trajX == null || trajY == null) return -1;
        double currentNearestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        int count = trajX.length;
        Vec2 normal = new Vec2();
        Vec2 dir = new Vec2();
        Vec2 delta = new Vec2();
        Vec2 point = new Vec2();
        double dist;
        Vec2 lastPoint = new Vec2();
        boolean hasLastPoint = false;
        
        for (int i = 0; i < count; i++){
            point.set(trajX[i], trajY[i]);

            if (hasLastPoint){
                // Check segment

                // 1) Get segment "normal"
                IPM.subtractToVec(point, lastPoint, dir);
                // Manual normalization to keep the length
                double length = dir.magnitude();
                if (length > 0.001){
                    IPM.multiply(dir, 1/length);
                } else {
                    dir.set(Double.NaN,Double.NaN);
                }

                // 2) check if in segment bounds
                IPM.subtractToVec(pos, lastPoint, delta);
                double projPos = IPM.dot(dir, delta);
                if (projPos > 0 && projPos < length) {
                
                    // 3) get distance
                    normal.set(-dir.y, dir.x);
                    dist = Math.abs(IPM.dot(normal, delta));
                    if (dist < currentNearestDistance){
                        currentNearestDistance = dist;
                        closestIndex = i-1;
                    }
                }
            }

            //Check point
            dist = point.distance(pos);
            if (dist < currentNearestDistance){
                currentNearestDistance = dist;
                closestIndex = i;
            }

            lastPoint.set(point);
            hasLastPoint = true;
        }
        return closestIndex;
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.COMPUTER;
    }
    
}