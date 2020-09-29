/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.car;

import java.awt.*;
import java.text.*;
import java.util.*;

import javax.swing.JMenuItem;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.Renderer;
import de.rwth.montisim.simulation.vehicle.*;

public class CarRenderer extends Renderer {
    private static final DecimalFormat posFormat = new DecimalFormat("##0.00",
            DecimalFormatSymbols.getInstance(Locale.ENGLISH));
    public static final Color CAR_COLOR = new Color(239, 81, 38);
    public static final Color LINE_COLOR = new Color(50, 50, 255);
    public static final Color FULL_GAS_COLOR = new Color(25, 255, 0);
    public static final Color FULL_REVERSE_COLOR = new Color(180, 200, 255);
    public static final Color FULL_BRAKE_COLOR = Color.ORANGE;
    public static final BasicStroke LINE_STROKE = new BasicStroke(2);
    public static final Color BOTTOM_COLOR = new Color(147, 47, 19);
    private static final int[] faceIndices = new int[] {
        0,1,3,2, // -X face
        4,5,7,6, // +X face
        0,1,5,4, // -Y face
        2,3,7,6, // +Y face
        0,2,6,4, // -Z face
        1,3,7,5, // +Z face
    };
    Optional<Vehicle> car = Optional.empty();
    final Vec3 half_size = new Vec3();
    final Vec3 points[] = new Vec3[8];
    final boolean visible[] = new boolean[6];

    final Polygonn[] faces = new Polygonn[] {
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


    Vec3 light1 = new Vec3(10,5,10);
    Vec3 light2 = new Vec3(-10,-5,5);

    // Vec for in-place computations
    Vec3 v1 = new Vec3();
    Vec3 v2 = new Vec3();
    Vec3 v3 = new Vec3();
    Mat3 m1 = new Mat3();
    Vec3 color = new Vec3(CAR_COLOR.getRed()/255d,CAR_COLOR.getGreen()/255d,CAR_COLOR.getBlue()/255d);

    boolean drawActuators = true;

    public CarRenderer() {}
    public CarRenderer(Vehicle car, Vec3 size){
        setCar(car, size);
    }

    public void setCar(Vehicle vehicle){
        VehicleProperties p = vehicle.properties;
        this.car = Optional.of(vehicle);
        IPM.multiplyTo(half_size, new Vec3(p.body.length, p.body.width, p.body.height), 0.5);
        for (int i = 0; i < points.length; ++i){
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

    public void setCar(Vehicle car, Vec3 size){
        this.car = Optional.of(car);
        IPM.multiplyTo(half_size, size, 0.5);
        for (int i = 0; i < points.length; ++i){
            points[i] = new Vec3();
        }
        IPM.normalize(light1);
        IPM.normalize(light2);
        dirty = true;
    }

    @Override
    public void draw(Graphics2D g) {
        if (!car.isPresent()) return;
        for (int i = 0; i < 6; ++i){
            if (visible[i]){
                Polygonn p = faces[i];
                g.setColor(p.color);
                g.fill(p.p);
            }
        }
        if (drawActuators){
            g.setColor(steeringLine.color);
            g.setStroke(steeringLine.stroke);
            g.drawPolyline(steeringLine.x,steeringLine.y,steeringLine.x.length);

            int width = (int) Math.round(gasCircle[1].x - gasCircle[0].x);
            g.setColor(gasColor);
            g.fillOval((int) gasCircle[0].x, (int) gasCircle[0].y, width, width);
            width = (int) Math.round(brakeCircle[1].x - brakeCircle[0].x);
            g.setColor(brakeColor);
            g.fillOval((int) brakeCircle[0].x, (int) brakeCircle[0].y, width, width);
        }
    }

    @Override
    public void computeGeometry(Mat3 viewMatrix) {
        if (!car.isPresent()) return;
        Vehicle vehicle = car.get();
        DynamicObject c = vehicle.physicalObject;
        int i = 0;
        // Generate and project Box geometry in one go
        for (int dx = -1; dx <= 1; dx +=2){
            for (int dy = -1; dy <= 1; dy +=2){
                for (int dz = -1; dz <= 1; dz +=2){
                    // Local pos
                    Vec3 p = points[i];
                    p.x = half_size.x *dx;
                    p.y = half_size.y *dy;
                    p.z = half_size.z *dz;
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
        for (i = 0; i < 6; i++){
            if (!visible[i]) continue;
            int offset = i*4;
            Polygonn p = faces[i];
            for (int j = 0; j <4; ++j){
                Vec3 res = points[faceIndices[offset+j]];
                p.p.xpoints[j] = (int) Math.round(res.x);
                p.p.ypoints[j] = (int) Math.round(res.y);
            }
        }

        // Compute steering line

        // Center of steering line = (Car pos + (local pos (front axle) -> rotated local pos))
        v1.set(half_size.x*0.6, 0,0); // Local pos in car
        IPM.multiply(c.rotation, v1); // Relative global pos
        IPM.add(v1, c.pos); // Global front axle pos

        // Steering direction = (Car Z rot) * steering angle
        v2.set(1,0,0);
        IPM.multiply(c.rotation, v2); // Get vehicle front vector => to 2D homogeneous coordinates
        v2.z = 0;
        IPM.normalize(v2);
        IPM.rotationMatrix(m1, (Double)vehicle.powerTrain.steeringValue.get() * Geometry.DEG_TO_RAD);
        IPM.multiply(m1, v2); // Rotate "Vehicle Front vector" to Steering direction
        IPM.multiply(v2, half_size.x*0.3);

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

        // Show Gas
        v1.set(half_size.x*-0.5, half_size.y*-0.5,0); // Local pos in car
        IPM.multiply(c.rotation, v1); // Relative global pos
        IPM.add(v1, c.pos); // Global pos
        v2.set(half_size.y*-0.3, half_size.y*0.3,0);
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

        double gas = (Double)vehicle.powerTrain.gasValue.get();
        if (gas < 0){
            gas *= -2;
            if (gas > 1) gas = 1;
            gasColor = new Color((int)(FULL_REVERSE_COLOR.getRed()*gas), (int)(FULL_REVERSE_COLOR.getGreen()*gas), (int)(FULL_REVERSE_COLOR.getBlue()*gas));
        } else {
            gasColor = new Color((int)(FULL_GAS_COLOR.getRed()*gas), (int)(FULL_GAS_COLOR.getGreen()*gas), (int)(FULL_GAS_COLOR.getBlue()*gas));
        }

        // Show Braking
        v1.set(half_size.x*-0.5, half_size.y*0.5,0); // Local pos in car
        IPM.multiply(c.rotation, v1); // Relative global pos
        IPM.add(v1, c.pos); // Global pos
        v2.set(half_size.y*-0.3, half_size.y*0.3,0);
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

        double brakes = (Double)vehicle.powerTrain.brakingValue.get();
        brakeColor = new Color((int)(FULL_GAS_COLOR.getRed()*brakes), (int)(FULL_GAS_COLOR.getGreen()*brakes), (int)(FULL_GAS_COLOR.getBlue()*brakes));
    }

    void checkFace(int i, Vec3 vec, boolean neg){
        visible[i] =  neg ? -vec.z > 0 : vec.z > 0;
        if (!visible[i]) return;
        // Perform crude illumination (diffuse from 2 lights)
    }

    @Override
    public String[] getInfo() {
        if (!car.isPresent()) return null;
        DynamicObject rb = car.get().physicalObject;
        return new String[]{
            "Car '"+rb.name+"' velocity: " + posFormat.format(rb.velocity.magnitude()*3.6),
            "    pos: "+posFormat.format(rb.pos.x) + " : "+posFormat.format(rb.pos.y)
        };
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