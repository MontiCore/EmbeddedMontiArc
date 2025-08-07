/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import org.junit.*;
import de.rwth.montisim.commons.utils.Vec3;

import java.util.ArrayList;

import static org.junit.Assert.*;

/**
 * Created by lukas on 26.01.17.
 */
public class ApproximateConverterTest {


    @Test
    public void testApp() throws Exception {
        ApproximateConverter converter = new ApproximateConverter(6.06132, 50.78026);
        ArrayList<Vec3> points = new ArrayList<>();

        points.add(new Vec3(6.06146, 50.78059, 0));
        points.add(new Vec3(6.06132, 50.78055, 0));
        points.add(new Vec3(6.06166, 50.78033, 0));
        points.add(new Vec3(6.06155, 50.78026, 0));

        ArrayList<Vec3> kPoints = new ArrayList<>();

        for (Vec3 p : points) {
            kPoints.add(converter.convertLongLatPoint(p));
        }

//        out.println(kPoints.get(0));
        //       out.println(kPoints.get(1));


        Vec3 origin = new Vec3(converter.getMinLat(), converter.getMinLong(), 0);

        for (int i = 0; i < points.size(); i++) {
            Vec3 p = points.get(i);
            Vec3 k = kPoints.get(i);
            //           out.println(k);
            //           out.println(p);
            //           out.println(Math.acos(Math.sin(converter.getMinLat()) * Math.sin(p.getX()) + Math.cos(converter.getMinLat()) * Math.cos(p.getX()) * Math.cos(p.getY() - converter.getMinLong())));
            double dist = 6378.388 * Math.acos(Math.sin(converter.getMinLat()) * Math.sin(p.getX()) + Math.cos(converter.getMinLat()) * Math.cos(p.getX()) * Math.cos(p.getY() - converter.getMinLong()));
            //assertEquals(dist, new Vec3(0, 0, 0).distance(k));

        }

        //distance ~90.69 m in Aachen
        Vec3 point1 = new Vec3(6.058674, 50.776730, 0);
        Vec3 point2 = new Vec3(6.058921, 50.777524, 0);

        converter = new ApproximateConverter(point1.getX(), point1.getY());
        Vec3 k1 = converter.convertLongLatPoint(point1);
        Vec3 k2 = converter.convertLongLatPoint(point2);

        assertEquals(90.69, k2.distance(k1), 1.2);

        //distance Cologne to Aachen is about 63.95
        Vec3 point3 = new Vec3(6.084934, 50.772430, 0);
        Vec3 point4 = new Vec3(6.958025, 50.936766, 0);


        converter = new ApproximateConverter(point3.getX(), point3.getY());
        Vec3 k3 = converter.convertLongLatPoint(point3);
        Vec3 k4 = converter.convertLongLatPoint(point4);

        //larger error for larger distances
        assertEquals(63950, k3.distance(k4), 70);
    }

    @Test
    public void testConvertXYToLonLat() {
        // First try to convert lon lat points into xy points
        // Then try to convert them back to check if the data are consistent
        ApproximateConverter converter = new ApproximateConverter(6.06132, 50.78026);
        ArrayList<Vec3> points = new ArrayList<>();

        points.add(new Vec3(6.06146, 50.78059, 0));
        points.add(new Vec3(6.06132, 50.78055, 0));
        points.add(new Vec3(6.06166, 50.78033, 0));
        points.add(new Vec3(6.06155, 50.78026, 0));

        ArrayList<Vec3> kPoints = new ArrayList<>();

        for (Vec3 p : points) {
            kPoints.add(converter.convertLongLatPoint(p));
        }

        for (int i = 0; i < points.size(); i++) {
            Vec3 p = points.get(i);
            Vec3 k = kPoints.get(i);

            assertEquals(p.getY(), converter.convertYToLat(k.getY()), 0.0001);
            assertEquals(p.getX(), converter.convertXToLong(k.getX(), k.getY()), 0.0001);

            Vec3 lonLatPointConvertedFromXYPoint = converter.convertXYPoint(k);
            assertEquals(p.getX(), lonLatPointConvertedFromXYPoint.getX(), 0.0001);
            assertEquals(p.getY(), lonLatPointConvertedFromXYPoint.getY(), 0.0001);
        }
    }
}
