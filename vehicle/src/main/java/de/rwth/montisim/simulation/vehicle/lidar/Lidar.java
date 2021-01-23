package de.rwth.montisim.simulation.vehicle.lidar;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Building;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueCompass;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;

import java.time.Instant;
import java.util.Vector;

public class Lidar extends EEComponent {

    public World world;

    private final Vector<Edge> edges = new Vector<>();

    private transient int truePositionMsg;
    private transient int trueCompassMsg;

    // define sensors massages variables
    private transient int frontRightSensorMsg;
    private transient int frontLeftSensorMsg;
    private transient int rightFrontSensorMsg;
    private transient int rightBackSensorMsg;
    private transient int leftFrontSensorMsg;
    private transient int leftBackSensorMsg;
    private transient int backRightSensorMsg;
    private transient int backLeftSensorMsg;



    private double angleDeg;

    public static final String FRONT_RIGHT_SENSOR_MSG = "front_right_lidar";
    public static final String FRONT_LEFT_SENSOR_MSG = "front_left_lidar";
    public static final String RIGHT_FRONT_SENSOR_MSG = "right_front_lidar";
    public static final String RIGHT_BACK_SENSOR_MSG = "right_back_lidar";
    public static final String LEFT_FRONT_SENSOR_MSG = "left_front_lidar";
    public static final String LEFT_BACK_SENSOR_MSG = "left_back_lidar";
    public static final String BACK_RIGHT_SENSOR_MSG = "back_right_lidar";
    public static final String BACK_LEFT_SENSOR_MSG = "back_left_lidar";

    private Vec2 truePosition = new Vec2(0,0);
    private Vec2 orientation = new Vec2(0,0);
    public static final double DOUBLE_TOLERANCE = 0.000001d;

    public Lidar(LidarProperties properties, EESystem eeSystem, World world){
        super(properties,eeSystem);

        for (Building building : world.buildings) {
            int corners = building.boundary.size();
            for (int i=0; i < corners; i++) {
                //Line line = getLineBetweenPoints(building.boundary.elementAt(i), building.boundary.elementAt(i%corners));
                edges.add(new Edge(building.boundary.elementAt(i), building.boundary.elementAt((i+1)%corners)));
            }
        }

        this.truePositionMsg = addPort(PortInformation.newOptionalInputDataPort(TruePosition.VALUE_NAME, TruePosition.TYPE, false));
        this.trueCompassMsg = addPort(PortInformation.newOptionalInputDataPort(TrueCompass.VALUE_NAME, TrueCompass.TYPE, false));

        this.frontRightSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(FRONT_RIGHT_SENSOR_MSG, PhysicalValueDouble.TYPE));
        this.frontLeftSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(FRONT_LEFT_SENSOR_MSG, PhysicalValueDouble.TYPE));
        this.rightFrontSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(RIGHT_FRONT_SENSOR_MSG, PhysicalValueDouble.TYPE));
        this.rightBackSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(RIGHT_BACK_SENSOR_MSG, PhysicalValueDouble.TYPE));
        this.leftFrontSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(LEFT_FRONT_SENSOR_MSG, PhysicalValueDouble.TYPE));
        this.leftBackSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(LEFT_BACK_SENSOR_MSG, PhysicalValueDouble.TYPE));
        this.backRightSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(BACK_RIGHT_SENSOR_MSG, PhysicalValueDouble.TYPE));
        this.backLeftSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(BACK_LEFT_SENSOR_MSG, PhysicalValueDouble.TYPE));
        
    }
    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        Instant time = msgRecvEvent.getEventTime();
        if (msg.isMsg(truePositionMsg)){
            truePosition = (Vec2) msg.message;
            compute(time);
        } else if (msg.isMsg(trueCompassMsg)){
            this.angleDeg = (double) msg.message;
            double angleRad = angleDeg * Geometry.DEG_TO_RAD;
            orientation = new Vec2(Math.cos(angleRad), Math.sin(angleRad));
        }
    }

    private void compute(Instant time) {

        double length = 4.971;
        double width = 1.87;
        double height = 1.383;

        Vec2 frontRightPosition = truePosition.add(length/2,width/4);    //in front
        Vec2 frontLeftPosition = truePosition.add(length/2,width/-4);     //in front
        Vec2 rightFrontPosition = truePosition.add(length/4,width/2);     //on the right
        Vec2 rightBackPosition = truePosition.add(length/-4,width/2);     //on the right
        Vec2 leftFrontPosition = truePosition.add(length/4,width/-2);     //on the left
        Vec2 leftBackPosition = truePosition.add(length/-4,width/-2);     //on the left
        Vec2 backRightPosition = truePosition.add(length/-2,width/4);     //in the back
        Vec2 backLeftPosition = truePosition.add(length/-2,width/-4);     //in the back


        double angleRad = (angleDeg + 90) * Geometry.DEG_TO_RAD;
        Vec2 rightOrientation = new Vec2(Math.cos(angleRad), Math.sin(angleRad));

        angleRad = (angleDeg - 90) * Geometry.DEG_TO_RAD;
        Vec2 leftOrientation = new Vec2(Math.cos(angleRad), Math.sin(angleRad));

        angleRad = (angleDeg + 180) * Geometry.DEG_TO_RAD;
        Vec2 backOrientation = new Vec2(Math.cos(angleRad), Math.sin(angleRad));

        double distanceFrontRight = computeShortestDistance(frontRightPosition, orientation);
        double distanceFrontLeft = computeShortestDistance(frontLeftPosition, orientation);
        double distanceRightFront = computeShortestDistance(rightFrontPosition, rightOrientation);
        double distanceRightBack = computeShortestDistance(rightBackPosition, rightOrientation);
        double distanceLeftFront = computeShortestDistance(leftFrontPosition, leftOrientation);
        double distanceLeftBack = computeShortestDistance(leftBackPosition, leftOrientation);
        double distanceBackRight = computeShortestDistance(backRightPosition, backOrientation);
        double distanceBackLeft = computeShortestDistance(backLeftPosition, backOrientation);

        sendMessage(time, frontRightSensorMsg, distanceFrontRight);
        sendMessage(time, frontLeftSensorMsg, distanceFrontLeft);
        sendMessage(time, rightFrontSensorMsg, distanceRightFront);
        sendMessage(time, rightBackSensorMsg, distanceRightBack);
        sendMessage(time, leftFrontSensorMsg, distanceLeftFront);
        sendMessage(time, leftBackSensorMsg, distanceLeftBack);
        sendMessage(time, backRightSensorMsg, distanceBackRight);
        sendMessage(time, backLeftSensorMsg, distanceBackLeft);
        
    }



    public double computeShortestDistance(Vec2 rayStart, Vec2 rayDirection) {
        double shortestDistance = Double.MAX_VALUE, currentDistance = 0;
        Vec2 currentIntersection = null, nearestIntersection = null;
        double edgeScalar;

        for (Edge edge : edges) {
            Vec2 edgeStart = edge.start.asVec2();
            Vec2 edgeDirection = edge.end.subtract(edge.start).asVec2();
            double directionsDeterminant = determinant2x2(edgeDirection, rayDirection);

            if (!UMath.equalsThreshold(directionsDeterminant, 0, DOUBLE_TOLERANCE)) {
                edgeScalar = (determinant2x2(rayDirection, edgeStart) + determinant2x2(rayStart, rayDirection)) / directionsDeterminant;
                if (edgeScalar >= 0 && edgeScalar <= 1) {
                    // ray hits the edge
                    currentIntersection = edgeStart.add(edgeDirection.multiply(edgeScalar));
                    currentDistance = currentIntersection.distance(rayStart);
                    if (currentDistance < shortestDistance) {
                        shortestDistance = currentDistance;
                        nearestIntersection = currentIntersection;
                    }
                }
            }
        }
        return shortestDistance;
    }

    private double determinant2x2(Vec2 a, Vec2 b){
        return a.x*b.y - a.y*b.x;
    }

    private static class Edge {
        public Vec3 start;
        public Vec3 end;

        public Edge(Vec3 start, Vec3 end) {
            this.start = start;
            this.end = end;
        }
    }
}
