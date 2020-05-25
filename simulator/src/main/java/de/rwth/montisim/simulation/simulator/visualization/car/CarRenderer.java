/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.simulator.visualization.car;

import java.awt.Color;
import java.awt.Graphics2D;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;
import java.util.Optional;

import javax.swing.JMenuItem;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.simulator.visualization.ui.Renderer;

public class CarRenderer extends Renderer {
    private static final DecimalFormat posFormat = new DecimalFormat("##0.00",
            DecimalFormatSymbols.getInstance(Locale.ENGLISH));
    public static final Color CAR_COLOR = new Color(239, 81, 38);
    public static final Color BOTTOM_COLOR = new Color(147, 47, 19);
    private static final int[] faceIndices = new int[] {
        0,1,3,2, // -X face
        4,5,7,6, // +X face
        0,1,5,4, // -Y face
        2,3,7,6, // +Y face
        0,2,6,4, // -Z face
        1,3,7,5, // +Z face
    };
    Optional<DynamicObject> car = Optional.empty();
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

    Vec3 light1 = new Vec3(10,5,10);
    Vec3 light2 = new Vec3(-10,-5,5);

    // Vec for in-place computations
    Vec3 a = new Vec3();
    Vec3 color = new Vec3(CAR_COLOR.getRed()/255d,CAR_COLOR.getGreen()/255d,CAR_COLOR.getBlue()/255d);

    public CarRenderer() {}
    public CarRenderer(DynamicObject car, Vec3 size){
        setCar(car, size);
    }

    public void setCar(DynamicObject car, Vec3 size){
        this.car = Optional.of(car);
        IPM.multiplyToVec(size, 0.5, half_size);
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
    }

    @Override
    public void computeGeometry(Mat3 viewMatrix) {
        if (!car.isPresent()) return;
        DynamicObject c = car.get();
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
    }

    void checkFace(int i, Vec3 vec, boolean neg){
        visible[i] =  neg ? -vec.z > 0 : vec.z > 0;
        if (!visible[i]) return;
        // Perform crude illumination (diffuse from 2 lights)
    }

    @Override
    public String[] getInfo() {
        if (!car.isPresent()) return null;
        DynamicObject rb = car.get();
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