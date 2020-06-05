/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.Geometry;
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

    public static final double MAX_DEVIATION = 3; // max allowed deviation from the trajectory "corners" (in meters)
    public static final double ORTHO_DIST = MAX_DEVIATION/(Math.sqrt(2)-1); // max allowed deviation from the trajectory "corners" (in meters)
    
    MessageInformation velocityMsg;
    MessageInformation positionMsg;
    MessageInformation compassMsg;

    
    MessageInformation trajXMsg;
    MessageInformation trajYMsg;

    MessageInformation steeringMsg;
    MessageInformation accelMsg;
    MessageInformation brakeMsg;
    
    public double currentVelocity = 0;
    public Vec2 currentPosition = null;
    public double currentCompass = Double.NaN;

    double newTrajX[] = null;
    public double trajX[] = null;
    public double trajY[] = null;
    
    Instant lastTime = null;
    final PID speedPid;
    final PID turnPid;

    public JavaAutopilot(JavaAutopilotProperties properties) {
        super(properties);
        this.properties = properties;
        this.speedPid = new PID(1,0,0.2);
        this.turnPid = new PID(1,0,0.2);
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
            // Assumes the x positions array of a new trajectory always arrives first
            newTrajX = (double[]) msg.message;
        } else if (msg.msgId == trajYMsg.messageId){
            trajY = (double[]) msg.message;
            trajX = newTrajX;
        }
    }

    // TODO set in commons, use in getNearestSegment (autopilot, navigation, ...)
    private class SegmentPos {
        final Vec2 posStart = new Vec2();
        final Vec2 posEnd = new Vec2();
        final Vec2 dir = new Vec2();
        final Vec2 normal = new Vec2();
        final Vec2 relPos = new Vec2();

        double length;
        double projPos;
        double orthoPos;
        double dist;
        double projDistToEnd;

        void initFromTraj(int index){
            getTrajPoint(index, posStart);
            getTrajPoint(index+1, posEnd);
            init();
        }

        /** Assumes posStart & posEnd are set */
        void init() {
            IPM.subtractToVec(posEnd, posStart, dir);
            length = dir.magnitude();
            if (length > 0.001){
                IPM.multiply(dir, 1/length);
            } else {
                dir.set(Double.NaN,Double.NaN);
            }
            normal.set(-dir.y, dir.x);

            IPM.subtractToVec(currentPosition, posStart, relPos);
            projPos = IPM.dot(dir, relPos);
            orthoPos = IPM.dot(normal, relPos);
            dist = Math.abs(orthoPos);
            projDistToEnd = length - projPos;
        }
    }

    final SegmentPos currSeg = new SegmentPos();
    final SegmentPos nextSeg = new SegmentPos();
    SegmentPos targetSeg;

    final Vec2 relPos = new Vec2();
    final Vec2 targetDir = new Vec2();
    final Vec2 carDir = new Vec2();

    // TEMP
    int mode = 0;
    int target = 0;

    void compute(Instant startTime) {
        if (currentPosition == null || Double.isNaN(currentCompass)) return;
        double carAngle = currentCompass * Geometry.DEG_TO_RAD;
        Instant sendTime = startTime.plus(properties.computeTime);

        int index = getNearestSegment(currentPosition);
        if (index < 0){
            // No trajectory -> Stay in place
            sendMessage(sendTime, accelMsg, 0.0);
            sendMessage(sendTime, brakeMsg, 1.0);
            return;
        }
        
        carDir.set(Math.cos(carAngle), Math.sin(carAngle));

        if (trajX.length == 1 || index + 1 >= trajX.length){
            // Only one point -> orient towards it
            // If "behind" -> just stop
            // Orient and try to stop at position
            return;
        }

        double dt = getDeltaTime(startTime);

        SegmentPos targetSeg = findTargetSegment(index);
        
        // Now that we have a target segment, we follow it
        double turnOutput = 0;
        double speedOutput = 0;

        boolean carTowardsSeg = targetSeg.orthoPos * IPM.dot(targetSeg.normal, carDir) < 0;
        boolean carTowardsEnd = IPM.dot(targetSeg.dir, carDir) > 0;

        double segmentAngle = Math.acos(targetSeg.dir.x) * Math.signum(targetSeg.dir.y);

        // Define TURNING to align with target (depending on distance, orientation & speed)


        // Target angle (relative to segment)
        double margin = 0.2;
        double targetAngleRel = targetSeg.dist < margin ? 
            0 : 
            targetSeg.dist < ORTHO_DIST ? 
                Math.acos(1+(margin-targetSeg.dist)/ORTHO_DIST) : 
                Math.PI*0.5;

        // If targetDir attained: check if "tangent turn" is possible
        // targetDir attained = car pointing between orthogonal to segment & targetDir (CHECK symmetric +/- versions)
        if (carTowardsSeg && carTowardsEnd && Math.abs(carAngle - segmentAngle) < targetAngleRel) {
            if (mode == 0) {
                //System.out.println("Tangent turn");
                mode = 1;
            }

            // Find the radius of a circle tangent to the segment and the car position
            double junctionAngle = Math.acos(IPM.dot(targetSeg.dir, carDir));
            double radius = Math.tan( (Math.PI - junctionAngle)*0.5) * targetSeg.dist / IPM.dot(carDir, targetSeg.normal);

            turnOutput = 452/radius * Math.signum(-targetSeg.orthoPos);

        } else {
            if (mode == 1) {
                //System.out.println("Follow angle");
                mode = 0;
            }
            double targetAngle = segmentAngle;
            targetAngle += targetAngleRel * Math.signum(-targetSeg.orthoPos);
            targetDir.set(Math.cos(targetAngleRel), Math.sin(targetAngleRel));

            while (targetAngle > carAngle+Math.PI) targetAngle -= 2*Math.PI;
            while (targetAngle < carAngle-Math.PI) targetAngle += 2*Math.PI;
            turnOutput = turnPid.compute(dt, carAngle, targetAngle);
            turnOutput *= Geometry.RAD_TO_DEG;
        }

        // Eval required speed -> in order to arrive at next turn with desired max speed or arrive at speed 0 at end
        // Respect max speed


        double finalTargetSpeed = 30;
        speedOutput = speedPid.compute(dt, currentVelocity, finalTargetSpeed);
        speedOutput /= 3.6; // Convert to m/s related space
        speedOutput /= properties.maxVehicleAccel; // Convert to [0:1] actuator range


        sendMessage(sendTime, steeringMsg, turnOutput);
        sendMessage(sendTime, accelMsg, speedOutput);
    }

    private double getDeltaTime(Instant time) {
        if (lastTime == null) {
            lastTime = time;
            return 0;
        } else {
            return Time.secondsFromDuration(Duration.between(lastTime, time));
        }
    }

    SegmentPos findTargetSegment(int index) {
        currSeg.initFromTraj(index);

        boolean hasNext = index + 2 < trajX.length;
        if (hasNext){
            nextSeg.initFromTraj(index+1);

            // Eval junction turn
            double junctionAngle = Math.acos(IPM.dot(currSeg.dir, nextSeg.dir));
            double junctionSign = Math.signum(IPM.dot(normal, nextSeg.dir));
            double maxRadius = junctionAngle == 0 ? Double.POSITIVE_INFINITY : MAX_DEVIATION / ((1/Math.cos(junctionAngle*0.5)) - 1);
            double maxDistToCorner = Math.sqrt(MAX_DEVIATION * (MAX_DEVIATION + maxRadius*2));

            // Check if "in turn"
            if (currSeg.projDistToEnd <= maxDistToCorner){
                if (target == 0){
                    target = 1;
                    //System.out.println("Following NEXT segment");
                }
                return nextSeg;
            }
        }

        target = 0;
        return currSeg;
    }

    final Vec2 normal = new Vec2();
    final Vec2 dir = new Vec2();
    final Vec2 delta = new Vec2();
    final Vec2 point = new Vec2();
    final Vec2 lastPoint = new Vec2();
    
    /**
     * Returns the index of the closest point in "traj" 
     * or the index of the start point of the closest segment
     * or -1 if there is no trajectory
     */
    private int getNearestSegment(Vec2 pos) {
        if (trajX == null || trajY == null) return -1;
        double currentNearestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        int count = trajX.length;
        double dist;
        boolean hasLastPoint = false;
        
        for (int i = 0; i < count; i++){
            getTrajPoint(i, point);

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

    void getTrajPoint(int index, Vec2 target){
        target.set(trajX[index], trajY[index]);
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.COMPUTER;
    }
    
}