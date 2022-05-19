/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.autopilots;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.simulation.commons.Inspectable;
import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;
import de.rwth.montisim.simulation.eecomponents.lidar.Lidar;
import de.rwth.montisim.simulation.eecomponents.speed_limit.SpeedLimitService;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.physicalvalues.BatteryLevel;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueCompass;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class JavaAutopilot extends EEComponent implements Inspectable {
    transient final JavaAutopilotProperties properties;

    public static final double MAX_DEVIATION = 3; // max allowed deviation from the trajectory "corners" (in meters)
    public static final double ORTHO_DIST = MAX_DEVIATION / (Math.sqrt(2) - 1); // max allowed deviation from the
    // trajectory "corners" (in meters)

    transient int velocityMsg;
    transient int positionMsg;
    transient int compassMsg;

    transient int trajLengthMsg;
    transient int trajXMsg;
    transient int trajYMsg;

    transient int steeringMsg;
    transient int accelMsg;
    transient int brakeMsg;

    transient int batteryMsg;

    transient List<Integer> sensorMsg = Arrays.asList(0, 0, 0, 0, 0, 0, 0, 0);

    transient int upperSpeedLimitMsg;

    public double currentVelocity = 0;
    public Vec2 currentPosition = null;
    public double currentCompass = Double.NaN;
    public double batteryLevel = 0; // Unused
    public List<Double> lidarSensors = Arrays.asList(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // Unused

    double newTrajX[] = null;
    public int newTrajLength = 0;
    public int trajLength = 0;
    public double trajX[] = null;
    public double trajY[] = null;

    public double upperSpeedLimitArr[] = null; // Unused

    public double currentGas = 0;
    public double currentSteering = 0;
    public double currentBrakes = 0;

    Instant lastTime = null;
    transient final PID speedPid;
    transient final PID turnPid;

    public JavaAutopilot(JavaAutopilotProperties properties, EESystem eeSystem) {
        super(properties, eeSystem);
        this.properties = properties;
        this.speedPid = new PID(1, 0, 0.2);
        this.turnPid = new PID(1, 0, 0.2);

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

        this.batteryMsg = addPort(PortInformation.newOptionalInputDataPort(BatteryLevel.VALUE_NAME, BatteryLevel.TYPE, false));

        for (int i = 0; i < sensorMsg.size(); i++) {
            this.sensorMsg.set(i, addPort(PortInformation.newOptionalInputDataPort(Lidar.LIDAR_MSG.get(i), PhysicalValueDouble.TYPE, false)));
        }

        this.upperSpeedLimitMsg = addPort(PortInformation.newOptionalInputDataPort(SpeedLimitService.UPPER_SPEED_LIMIT_MSG, SpeedLimitService.SPEED_LIMIT_TYPE, false));

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
        } else if (msg.isMsg(batteryMsg)) {
            batteryLevel = (double) msg.message;
        } else if (msg.isMsg(upperSpeedLimitMsg)) {
            upperSpeedLimitArr = (double[]) msg.message;
        } else {
            for (int i = 0; i < sensorMsg.size(); i++) {
                if (msg.isMsg(sensorMsg.get(i))) {
                    this.lidarSensors.set(i, (double) msg.message);
                }
            }
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

        void initFromTraj(int index) {
            getTrajPoint(index, posStart);
            getTrajPoint(index + 1, posEnd);
            init();
        }

        /**
         * Assumes posStart & posEnd are set
         */
        void init() {
            IPM.subtractTo(dir, posEnd, posStart);
            length = dir.magnitude();
            if (length > 0.001) {
                IPM.multiply(dir, 1 / length);
            } else {
                dir.set(Double.NaN, Double.NaN);
            }
            normal.set(-dir.y, dir.x);

            IPM.subtractTo(relPos, currentPosition, posStart);
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
        if (currentPosition == null || Double.isNaN(currentCompass))
            return;
        double carAngle = currentCompass * Geometry.DEG_TO_RAD;
        Instant sendTime = startTime.plus(properties.compute_time);

        int index = getNearestSegment(currentPosition);
        if (index < 0) {
            // No trajectory -> Stay in place
            setGas(sendTime, 0.0);
            setBrakes(sendTime, 1.0);
            return;
        }

        carDir.set(Math.cos(carAngle), Math.sin(carAngle));

        if (trajLength == 1 || index + 1 >= trajLength) {
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

        // Define TURNING to align with target (depending on distance, orientation &
        // speed)

        // Target angle (relative to segment)
        double margin = 0.2;
        double targetAngleRel = targetSeg.dist < margin ? 0
                : targetSeg.dist < ORTHO_DIST ? Math.acos(1 + (margin - targetSeg.dist) / ORTHO_DIST) : Math.PI * 0.5;

        // If targetDir attained: check if "tangent turn" is possible
        // targetDir attained = car pointing between orthogonal to segment & targetDir
        // (CHECK symmetric +/- versions)
        if (carTowardsSeg && carTowardsEnd && Math.abs(carAngle - segmentAngle) < targetAngleRel) {
            if (mode == 0) {
                // System.out.println("Tangent turn");
                mode = 1;
            }

            // Find the radius of a circle tangent to the segment and the car position
            double junctionAngle = Math.acos(IPM.dot(targetSeg.dir, carDir));
            double radius = Math.tan((Math.PI - junctionAngle) * 0.5) * targetSeg.dist
                    / IPM.dot(carDir, targetSeg.normal);

            turnOutput = 452 / radius * Math.signum(-targetSeg.orthoPos);

        } else {
            if (mode == 1) {
                // System.out.println("Follow angle");
                mode = 0;
            }
            double targetAngle = segmentAngle;
            targetAngle += targetAngleRel * Math.signum(-targetSeg.orthoPos);
            targetDir.set(Math.cos(targetAngleRel), Math.sin(targetAngleRel));

            while (targetAngle > carAngle + Math.PI)
                targetAngle -= 2 * Math.PI;
            while (targetAngle < carAngle - Math.PI)
                targetAngle += 2 * Math.PI;
            turnOutput = turnPid.compute(dt, carAngle, targetAngle);
            turnOutput *= Geometry.RAD_TO_DEG;
        }

        // Eval required speed -> in order to arrive at next turn with desired max speed
        // or arrive at speed 0 at end
        // Respect max speed

        double finalTargetSpeed = 30;
        speedOutput = speedPid.compute(dt, currentVelocity, finalTargetSpeed);
        speedOutput /= 3.6; // Convert to m/s related space
        speedOutput /= properties.maxVehicleAccel; // Convert to [0:1] actuator range

        setSteering(sendTime, turnOutput);
        setGas(sendTime, speedOutput);
        setBrakes(sendTime, 0);
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

        boolean hasNext = index + 2 < trajLength;
        if (hasNext) {
            nextSeg.initFromTraj(index + 1);

            // Eval junction turn
            double junctionAngle = Math.acos(IPM.dot(currSeg.dir, nextSeg.dir));
            //double junctionSign = Math.signum(IPM.dot(normal, nextSeg.dir));
            double maxRadius = junctionAngle == 0 ? Double.POSITIVE_INFINITY
                    : MAX_DEVIATION / ((1 / Math.cos(junctionAngle * 0.5)) - 1);
            double maxDistToCorner = Math.sqrt(MAX_DEVIATION * (MAX_DEVIATION + maxRadius * 2));

            // Check if "in turn"
            if (currSeg.projDistToEnd <= maxDistToCorner) {
                if (target == 0) {
                    target = 1;
                    // System.out.println("Following NEXT segment");
                }
                return nextSeg;
            }
        }

        target = 0;
        return currSeg;
    }

    transient final Vec2 normal = new Vec2();
    transient final Vec2 dir = new Vec2();
    transient final Vec2 delta = new Vec2();
    transient final Vec2 point = new Vec2();
    transient final Vec2 lastPoint = new Vec2();

    /**
     * Returns the index of the closest point in "traj" or the index of the start
     * point of the closest segment or -1 if there is no trajectory
     */
    private int getNearestSegment(Vec2 pos) {
        if (trajX == null || trajY == null)
            return -1;
        double currentNearestDistance = Double.POSITIVE_INFINITY;
        int closestIndex = -1;
        int count = trajLength;
        double dist;
        boolean hasLastPoint = false;

        for (int i = 0; i < count; i++) {
            getTrajPoint(i, point);

            if (hasLastPoint) {
                // Check segment

                // 1) Get segment "normal"
                IPM.subtractTo(dir, point, lastPoint);
                // Manual normalization to keep the length
                double length = dir.magnitude();
                if (length > 0.001) {
                    IPM.multiply(dir, 1 / length);
                } else {
                    dir.set(Double.NaN, Double.NaN);
                }

                // 2) check if in segment bounds
                IPM.subtractTo(delta, pos, lastPoint);
                double projPos = IPM.dot(dir, delta);
                if (projPos > 0 && projPos < length) {

                    // 3) get distance
                    normal.set(-dir.y, dir.x);
                    dist = Math.abs(IPM.dot(normal, delta));
                    if (dist < currentNearestDistance) {
                        currentNearestDistance = dist;
                        closestIndex = i - 1;
                    }
                }
            }

            // Check point
            dist = point.distance(pos);
            if (dist < currentNearestDistance) {
                currentNearestDistance = dist;
                closestIndex = i;
            }

            lastPoint.set(point);
            hasLastPoint = true;
        }
        return closestIndex;
    }

    void getTrajPoint(int index, Vec2 target) {
        target.set(trajX[index], trajY[index]);
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
        addEntry(entries, false, ports.elementAt(9), batteryLevel);
        addEntry(entries, false, ports.elementAt(10), lidarSensors.get(0)); // frontRightSensor
        addEntry(entries, false, ports.elementAt(11), lidarSensors.get(1)); // frontLeftSensor
        addEntry(entries, false, ports.elementAt(12), lidarSensors.get(2)); // rightFrontSensor
        addEntry(entries, false, ports.elementAt(13), lidarSensors.get(3)); // rightBackSensor
        addEntry(entries, false, ports.elementAt(14), lidarSensors.get(4)); // leftFrontSensor
        addEntry(entries, false, ports.elementAt(15), lidarSensors.get(5)); // leftBackSensor
        addEntry(entries, false, ports.elementAt(16), lidarSensors.get(6)); // backRightSensor
        addEntry(entries, false, ports.elementAt(17), lidarSensors.get(7)); // backLeftSensor
        addEntry(entries, false, ports.elementAt(18), upperSpeedLimitArr);
        return entries;
    }

    @Override
    public String getName() {
        return properties.name;
    }

}