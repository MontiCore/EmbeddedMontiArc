/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.car;

import java.awt.*;
import java.text.*;
import java.util.*;
import java.util.List;
import javax.swing.JMenuItem;

import de.rwth.montisim.simulation.commons.boundingbox.AABB;
import de.rwth.montisim.simulation.commons.boundingbox.OBB;
import de.rwth.montisim.simulation.commons.physicalvalue.*;
import de.rwth.montisim.commons.map.Path;
import de.rwth.montisim.simulation.commons.DynamicObject;
import de.rwth.montisim.simulation.commons.Inspectable;
import de.rwth.montisim.simulation.commons.StaticObject;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.simulator.visualization.ui.Renderer;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;
import de.rwth.montisim.simulation.vehicle.*;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;
import de.rwth.montisim.simulation.vehicle.navigation.NavigationProperties;

public class CarRenderer extends Renderer {
    private static final DecimalFormat posFormat = new DecimalFormat("##0.00",
            DecimalFormatSymbols.getInstance(Locale.ENGLISH));
    public static final Color CAR_COLOR = new Color(47, 141, 235);//= new Color(204,163,122); // new Color(239, 81, 38);
    public static final Color CAR_COLLISION_COLOR = new Color(237, 126, 42);//= new Color(204,163,122); // new Color(239, 81, 38);
    public static final Color LINE_COLOR = Color.BLACK; // new Color(122,130,129); // new Color(50, 50, 255);
    public static final Color FULL_GAS_COLOR = new Color(25, 255, 0);
    public static final Color FULL_REVERSE_COLOR = new Color(180, 200, 255);
    public static final Color FULL_BRAKE_COLOR = Color.RED;
    public static final BasicStroke LINE_STROKE = new BasicStroke(2);
    public static final Color BOTTOM_COLOR = new Color(147, 47, 19);

    public static final Color PATH_COLOR = new Color(255, 100, 100, 100);
    public static final Color TRAJECTORY_COLOR = new Color(50, 255, 50, 255);
    public static final BasicStroke PATH_STROKE = new BasicStroke(3);
    public static final BasicStroke TRAJECTORY_STROKE = new BasicStroke(1);
    public static final Color PAST_TRAJECTORY_COLOR = new Color(51, 153, 255);

    private static final int[] faceIndices = new int[]{
            0, 1, 3, 2, // -X face
            4, 5, 7, 6, // +X face
            0, 1, 5, 4, // -Y face
            2, 3, 7, 6, // +Y face
            0, 2, 6, 4, // -Z face
            1, 3, 7, 5, // +Z face
    };
    Optional<Vehicle> car = Optional.empty();
    Optional<PhysicalValue> truePosition = Optional.empty();
    final Vec3 half_size = new Vec3();
    final Vec3 points[] = new Vec3[8];
    final boolean visible[] = new boolean[6];

    final Polygonn[] faces = new Polygonn[]{
            new Polygonn(4, CAR_COLOR),
            new Polygonn(4, CAR_COLOR),
            new Polygonn(4, CAR_COLOR),
            new Polygonn(4, CAR_COLOR),
            new Polygonn(4, BOTTOM_COLOR),
            new Polygonn(4, CAR_COLOR)
    };
    final Polyline steeringLine = new Polyline(2, LINE_COLOR, LINE_STROKE);
    final Vec2[] gasCircle = new Vec2[2];
    Color gasColor = Color.BLACK;
    final Vec2[] brakeCircle = new Vec2[2];
    Color brakeColor = Color.BLACK;


    final List<Polyline> pathLines = new ArrayList<>();
    final List<Polyline> trajectoryLines = new ArrayList<>();
    final List<Polyline> pastTrajectoryLines = new ArrayList<>();
    private final List<Polyline> aabbLines = new ArrayList<>();
    private final Polyline aabbContour = new Polyline(5, UIInfo.AABB_COLOR);
    private final Polyline obbLines[] = new Polyline[]{new Polyline(2, UIInfo.OOB_X_COLOR), new Polyline(2, UIInfo.OOB_Y_COLOR), new Polyline(2, UIInfo.OOB_Z_COLOR)};


    Vec3 light1 = new Vec3(10, 5, 10);
    Vec3 light2 = new Vec3(-10, -5, 5);

    // Vec for in-place computations
    Vec3 v1 = new Vec3();
    Vec3 v2 = new Vec3();
    Vec3 v3 = new Vec3();
    Mat3 m1 = new Mat3();
    Vec3 color = new Vec3(CAR_COLOR.getRed() / 255d, CAR_COLOR.getGreen() / 255d, CAR_COLOR.getBlue() / 255d);

    private Vec3 lastPosition = null;

    public CarRenderer() {
        initGeom();
    }

    public CarRenderer(Vehicle car, Vec3 size) {
        setCar(car, size);
        initGeom();
    }

    public void initGeom() {
        aabbLines.add(aabbContour);
        for (int i = 0; i < 5; ++i) {
            aabbContour.points[i] = new Vec3();
        }
        for (int i = 0; i < 3; ++i) {
            aabbLines.add(obbLines[i]);
            obbLines[i].points[0] = new Vec3();
            obbLines[i].points[1] = new Vec3();
        }
    }

    public void setCar(Vehicle vehicle) {
        VehicleProperties p = vehicle.properties;
        this.car = Optional.of(vehicle);
        this.truePosition = Optional.of(vehicle.physicalValues.getPhysicalValue("true_position"));
        IPM.multiplyTo(half_size, new Vec3(p.body.length, p.body.width, p.body.height), 0.5);
        for (int i = 0; i < points.length; ++i) {
            points[i] = new Vec3();
        }
        IPM.normalize(light1);
        IPM.normalize(light2);
        dirty = true;
        gasCircle[0] = new Vec2();
        gasCircle[1] = new Vec2();
        brakeCircle[0] = new Vec2();
        brakeCircle[1] = new Vec2();
    }

    public void setCar(Vehicle car, Vec3 size) {
        this.car = Optional.of(car);
        this.truePosition = Optional.of(car.physicalValues.getPhysicalValue("true_position"));
        IPM.multiplyTo(half_size, size, 0.5);
        for (int i = 0; i < points.length; ++i) {
            points[i] = new Vec3();
        }
        IPM.normalize(light1);
        IPM.normalize(light2);
        dirty = true;
    }

    @Override
    public void draw(Graphics2D g) {
        if (!car.isPresent()) return;
        if (UIInfo.drawPlannedPath) {
            drawLines(g, pathLines);
        }
        if (UIInfo.drawPlannedTrajectory) {
            drawLines(g, trajectoryLines);
        }
        faces[5].color = car.get().staticCollisions.size() > 0 ? CAR_COLLISION_COLOR : CAR_COLOR;
        for (int i = 0; i < 6; ++i) {
            if (visible[i]) {
                Polygonn p = faces[i];
                g.setColor(p.color);
                g.fill(p.p);
            }
        }
        if (UIInfo.drawActuators) {
            g.setColor(steeringLine.color);
            g.setStroke(steeringLine.stroke);
            g.drawPolyline(steeringLine.x, steeringLine.y, steeringLine.x.length);

            int width = (int) Math.round(gasCircle[1].x - gasCircle[0].x);
            g.setColor(gasColor);
            g.fillOval((int) gasCircle[0].x, (int) gasCircle[0].y, width, width);
            width = (int) Math.round(brakeCircle[1].x - brakeCircle[0].x);
            g.setColor(brakeColor);
            g.fillOval((int) brakeCircle[0].x, (int) brakeCircle[0].y, width, width);
        }
        if (UIInfo.showAABBs) {
            drawLines(g, aabbLines);
        }

        if (UIInfo.drawDrivenTrajectory) {
            drawLines(g, pastTrajectoryLines);
        }
    }

    @Override
    public void computeGeometry(Mat3 viewMatrix) {
        if (!car.isPresent()) return;
        Vehicle vehicle = car.get();
        DynamicObject c = vehicle.physicalObject;
        int i = 0;
        // Generate and project Box geometry in one go
        for (int dx = -1; dx <= 1; dx += 2) {
            for (int dy = -1; dy <= 1; dy += 2) {
                for (int dz = -1; dz <= 1; dz += 2) {
                    // Local pos
                    Vec3 p = points[i];
                    p.x = half_size.x * dx;
                    p.y = half_size.y * dy;
                    p.z = half_size.z * dz;
                    // To world pos
                    IPM.multiply(c.rotation, p);
                    IPM.add(p, c.pos);
                    p.z = 1; // "Project" to 2D homogenous coordinates
                    // To screen pos
                    IPM.multiply(viewMatrix, p);
                    ++i;
                }
            }
        }
        // Check face visibility
        // -X, +X, -Y, +Y, -Z, +Z
        // Here the normals are the 3 local axes => rotated axes = entries in the rotation matrix
        // The "viewMatrix" is in 2D => doesn't affect Z => ignore
        checkFace(0, c.rotation.col1, true);
        checkFace(1, c.rotation.col1, false);
        checkFace(2, c.rotation.col2, true);
        checkFace(3, c.rotation.col2, false);
        checkFace(4, c.rotation.col3, true);
        checkFace(5, c.rotation.col3, false);

        // Fill faces
        for (i = 0; i < 6; i++) {
            if (!visible[i]) continue;
            int offset = i * 4;
            Polygonn p = faces[i];
            for (int j = 0; j < 4; ++j) {
                Vec3 res = points[faceIndices[offset + j]];
                p.p.xpoints[j] = (int) Math.round(res.x);
                p.p.ypoints[j] = (int) Math.round(res.y);
            }
        }

        if (UIInfo.drawActuators) {
            computeSteeringLine(viewMatrix);
            computeGasCircle(viewMatrix);
            computeBrakeCircle(viewMatrix);
        }

        if (true) {
            computeTrajectoryHistory(viewMatrix);
        } //else {
        //    lastPosition = null;
        //}

        if (UIInfo.showAABBs) {
            AABB b = vehicle.physicalObject.worldSpaceAABB.get();
            aabbContour.points[0].set(b.min.x, b.min.y, 1.0);
            aabbContour.points[1].set(b.max.x, b.min.y, 1.0);
            aabbContour.points[2].set(b.max.x, b.max.y, 1.0);
            aabbContour.points[3].set(b.min.x, b.max.y, 1.0);
            aabbContour.points[4].set(b.min.x, b.min.y, 1.0);

            OBB b2 = (OBB) vehicle.physicalObject.bbox.get();
            double x = b2.pos.x;
            double y = b2.pos.y;
            for (i = 0; i < 3; ++i) {
                obbLines[i].points[0].set(x, y, 1.0);
                obbLines[i].points[1].set(
                        x + b2.world_space_half_axes.at(0, i),
                        y + b2.world_space_half_axes.at(1, i),
                        1.0
                );

            }
            computeLineGeometry(viewMatrix, aabbLines);
        }

        if (UIInfo.drawPlannedPath || UIInfo.drawPlannedTrajectory) {
            Optional<EEComponent> res = vehicle.eesystem.getComponent(NavigationProperties.NAME);
            if (res.isPresent()) {
                Navigation nav = (Navigation) res.get();

                if (UIInfo.drawPlannedPath) {
                    pathLines.clear();
                    Optional<Path> opath = nav.getCurrentPath();
                    if (opath.isPresent()) {
                        Path path = opath.get();
                        // Fill path
                        int count = path.getLength();
                        if (count > 1) {
                            Polyline poly = new Polyline(count, PATH_COLOR, PATH_STROKE);
                            pathLines.add(poly);
                            for (i = 0; i < count; ++i) {
                                poly.points[i] = new Vec3(path.trajectoryX[i], path.trajectoryY[i], 1);
                            }
                        }
                    }
                    computeLineGeometry(viewMatrix, pathLines);
                }
                if (UIInfo.drawPlannedTrajectory) {
                    trajectoryLines.clear();
                    // Fill path
                    int count = nav.getCurrentTrajSize();
                    if (count > 1) {
                        Vec2 traj[] = nav.getCurrentTraj();
                        Polyline poly = new Polyline(count, TRAJECTORY_COLOR, TRAJECTORY_STROKE);
                        trajectoryLines.add(poly);
                        for (i = 0; i < count; ++i) {
                            poly.points[i] = new Vec3(traj[i], 1);
                        }
                    } else if (count == 1) {
                        // TODO as POINT
                    }
                    computeLineGeometry(viewMatrix, trajectoryLines);
                }
            }
        }


    }

    void computeSteeringLine(Mat3 viewMatrix) {
        Vehicle vehicle = car.get();
        DynamicObject c = vehicle.physicalObject;
        // Center of steering line = (Car pos + (local pos (front axle) -> rotated local pos))
        v1.set(half_size.x * 0.6, 0, 0); // Local pos in car
        IPM.multiply(c.rotation, v1); // Relative global pos
        IPM.add(v1, c.pos); // Global front axle pos

        // Steering direction = (Car Z rot) * steering angle
        v2.set(1, 0, 0);
        IPM.multiply(c.rotation, v2); // Get vehicle front vector => to 2D homogeneous coordinates
        v2.z = 0;
        IPM.normalize(v2);
        IPM.rotationMatrix(m1, (Double) vehicle.powerTrain.steeringValue.get() * Geometry.DEG_TO_RAD);
        IPM.multiply(m1, v2); // Rotate "Vehicle Front vector" to Steering direction
        IPM.multiply(v2, half_size.x * 0.3);

        // Point 1
        IPM.addTo(v3, v1, v2);
        v3.z = 1; // homogenous coordinates
        IPM.multiply(viewMatrix, v3);
        steeringLine.x[0] = (int) Math.round(v3.x);
        steeringLine.y[0] = (int) Math.round(v3.y);
        // Point 2
        IPM.subtractTo(v3, v1, v2);
        v3.z = 1; // homogenous coordinates
        IPM.multiply(viewMatrix, v3);
        steeringLine.x[1] = (int) Math.round(v3.x);
        steeringLine.y[1] = (int) Math.round(v3.y);
    }

    void computeGasCircle(Mat3 viewMatrix) {
        Vehicle vehicle = car.get();
        DynamicObject c = vehicle.physicalObject;
        v1.set(half_size.x * -0.5, half_size.y * -0.5, 0); // Local pos in car
        IPM.multiply(c.rotation, v1); // Relative global pos
        IPM.add(v1, c.pos); // Global pos
        v2.set(half_size.y * -0.3, half_size.y * 0.3, 0);
        // Upper left corner
        IPM.addTo(v3, v1, v2);
        v3.z = 1; // homogenous coordinates
        IPM.multiply(viewMatrix, v3);
        gasCircle[0].x = (int) Math.round(v3.x);
        gasCircle[0].y = (int) Math.round(v3.y);
        // Lower right corner
        IPM.subtractTo(v3, v1, v2);
        v3.z = 1; // homogenous coordinates
        IPM.multiply(viewMatrix, v3);
        gasCircle[1].x = (int) Math.round(v3.x);
        gasCircle[1].y = (int) Math.round(v3.y);

        double gas = (Double) vehicle.powerTrain.gasValue.get();
        if (gas < 0) {
            gas *= -2;
            if (gas > 1) gas = 1;
            gasColor = new Color((int) (FULL_REVERSE_COLOR.getRed() * gas), (int) (FULL_REVERSE_COLOR.getGreen() * gas), (int) (FULL_REVERSE_COLOR.getBlue() * gas));
        } else {
            gasColor = new Color((int) (FULL_GAS_COLOR.getRed() * gas), (int) (FULL_GAS_COLOR.getGreen() * gas), (int) (FULL_GAS_COLOR.getBlue() * gas));
        }
    }

    void computeBrakeCircle(Mat3 viewMatrix) {
        Vehicle vehicle = car.get();
        DynamicObject c = vehicle.physicalObject;
        v1.set(half_size.x * -0.5, half_size.y * 0.5, 0); // Local pos in car
        IPM.multiply(c.rotation, v1); // Relative global pos
        IPM.add(v1, c.pos); // Global pos
        v2.set(half_size.y * -0.3, half_size.y * 0.3, 0);
        // Upper left corner
        IPM.addTo(v3, v1, v2);
        v3.z = 1; // homogenous coordinates
        IPM.multiply(viewMatrix, v3);
        brakeCircle[0].x = (int) Math.round(v3.x);
        brakeCircle[0].y = (int) Math.round(v3.y);
        // Lower right corner
        IPM.subtractTo(v3, v1, v2);
        v3.z = 1; // homogenous coordinates
        IPM.multiply(viewMatrix, v3);
        brakeCircle[1].x = (int) Math.round(v3.x);
        brakeCircle[1].y = (int) Math.round(v3.y);

        double brakes = (Double) vehicle.powerTrain.brakingValue.get();
        brakeColor = new Color((int) (FULL_BRAKE_COLOR.getRed() * brakes), (int) (FULL_BRAKE_COLOR.getGreen() * brakes), (int) (FULL_BRAKE_COLOR.getBlue() * brakes));
    }

    void checkFace(int i, Vec3 vec, boolean neg) {
        visible[i] = neg ? -vec.z > 0 : vec.z > 0;
        if (!visible[i]) return;
        // Perform crude illumination (diffuse from 2 lights)
    }

    void computeTrajectoryHistory(Mat3 viewMatrix) {
        PhysicalValue position1 = truePosition.get();
        Vec2 currentPosition = (Vec2) position1.get();
        Vec3 position3D = new Vec3(currentPosition, 1d);
        if (lastPosition != null) {
            Polyline path = new Polyline(2, PAST_TRAJECTORY_COLOR);
            path.points[0] = lastPosition;
            path.points[1] = position3D;
            pastTrajectoryLines.add(path);
        }
        lastPosition = position3D;
        computeLineGeometry(viewMatrix, pastTrajectoryLines);
    }

    @Override
    public List<String> getInfo() {
        if (!car.isPresent()) return null;
        DynamicObject rb = car.get().physicalObject;
        List<String> baseInfo = new ArrayList<>();
        baseInfo.add("Car '" + rb.name + "' velocity: " + posFormat.format(rb.velocity.magnitude() * 3.6));
        baseInfo.add("  pos: " + posFormat.format(rb.pos.x) + " : " + posFormat.format(rb.pos.y));

        for (Vehicle col : car.get().vehicleCollisions) {
            baseInfo.add("Vehicle Collision: " + col.properties.vehicleName);
        }
        for (StaticObject col : car.get().staticCollisions) {
            baseInfo.add("Static Collision: " + col.name);
        }

        if (UIInfo.inspectAutopilots) {
            car.get().eesystem.componentTable.stream()
                    .filter(c -> (c instanceof Inspectable))
                    .map(c -> ((Inspectable) c))
                    .filter(c -> c.getType().equals("autopilot"))
                    .forEach(i -> {
                        baseInfo.add("  Autopilot: " + i.getName());
                        List<String> entries = i.getEntries();
                        for (String e : entries) {
                            baseInfo.add("    " + e);
                        }
                    });
        }
        return baseInfo;
    }

    @Override
    public String[] getHoverInfo(Vec2 worldPos) {
        return null;
    }

    @Override
    public JMenuItem[] getClicMenuItem(Vec2 worldPos) {
        return null;
    }

}