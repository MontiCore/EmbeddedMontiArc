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
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueCompass;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;

import java.time.Instant;
import java.util.Vector;

public class Lidar extends EEComponent {

    public World world;

    private final Vector<Edge> edges = new Vector<>();

    private transient int truePositionMsg;
    private transient int trueCompassMsg;

    private transient int frontSensorMsg;

    public static final String FRONT_SENSOR_MSG = "front_sensor";

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

        this.frontSensorMsg = addPort(PortInformation.newOptionalOutputDataPort(FRONT_SENSOR_MSG, PhysicalValueDouble.TYPE));
    }
    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        Instant time = msgRecvEvent.getEventTime();
        if (msg.isMsg(truePositionMsg)){
            truePosition = (Vec2) msg.message;
            compute(time);
        } else if (msg.isMsg(trueCompassMsg)){
            Double angleDeg = (Double) msg.message;
            Double angleRad = angleDeg * Geometry.DEG_TO_RAD;
            orientation = new Vec2(Math.cos(angleRad), Math.sin(angleRad));
        }
    }

    private void compute(Instant time) {
        double distance = computeShortestDistance(truePosition, orientation);
        sendMessage(time, frontSensorMsg, distance);
    }



    public double computeShortestDistance(Vec2 rayStart, Vec2 rayDirection) {
//        Vec2 rayStart = position.asVec2();
//        Vec2 rayDirection = direction.asVec2();

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
