/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.environment.osm;

import org.junit.*;
import javafx.geometry.Point3D;
import java.util.ArrayList;
import static  org.junit.Assert.*;

/**
 * Created by lukas on 26.01.17.
 */
public class ApproximateConverterTest {


	@Test
    public void testApp() throws Exception {
        ApproximateConverter converter = new ApproximateConverter(6.06132, 50.78026);
        ArrayList<Point3D> points = new ArrayList<>();

        points.add(new Point3D(6.06146,50.78059, 0));
        points.add(new Point3D(6.06132,50.78055, 0));
        points.add(new Point3D(6.06166, 50.78033,0));
        points.add(new Point3D(6.06155, 50.78026, 0));

        ArrayList<Point3D> kPoints = new ArrayList<>();

        for(Point3D p : points) {
            kPoints.add(converter.convertLongLatPoint(p));
        }

//        out.println(kPoints.get(0));
 //       out.println(kPoints.get(1));


        Point3D origin = new Point3D(converter.getMinLat(), converter.getMinLong(), 0);

        for(int i = 0; i < points.size(); i++) {
            Point3D p = points.get(i);
            Point3D k = kPoints.get(i);
 //           out.println(k);
 //           out.println(p);
 //           out.println(Math.acos(Math.sin(converter.getMinLat()) * Math.sin(p.getX()) + Math.cos(converter.getMinLat()) * Math.cos(p.getX()) * Math.cos(p.getY() - converter.getMinLong())));
            double dist = 6378.388 * Math.acos(Math.sin(converter.getMinLat()) * Math.sin(p.getX()) + Math.cos(converter.getMinLat()) * Math.cos(p.getX()) * Math.cos(p.getY() - converter.getMinLong()));
            //assertEquals(dist, new Point3D(0, 0, 0).distance(k));

        }

        //distance ~90.69 m in Aachen
        Point3D point1 = new Point3D(6.058674, 50.776730, 0);
        Point3D point2 = new Point3D(6.058921, 50.777524, 0);

        converter = new ApproximateConverter(point1.getX(), point1.getY());
        Point3D k1 = converter.convertLongLatPoint(point1);
        Point3D k2 = converter.convertLongLatPoint(point2);

        assertEquals(90.69, k2.distance(k1), 1.2);

        //distance Cologne to Aachen is about 63.95
        Point3D point3 = new Point3D(6.084934, 50.772430, 0);
        Point3D point4 = new Point3D(6.958025, 50.936766, 0);


        converter = new ApproximateConverter(point3.getX(), point3.getY());
        Point3D k3 = converter.convertLongLatPoint(point3);
        Point3D k4 = converter.convertLongLatPoint(point4);

        //larger error for larger distances
        assertEquals(63950, k3.distance(k4), 70);
    }

	@Test
    public void testConvertXYToLonLat() {
        // First try to convert lon lat points into xy points
        // Then try to convert them back to check if the data are consistent
        ApproximateConverter converter = new ApproximateConverter(6.06132, 50.78026);
        ArrayList<Point3D> points = new ArrayList<>();

        points.add(new Point3D(6.06146,50.78059, 0));
        points.add(new Point3D(6.06132,50.78055, 0));
        points.add(new Point3D(6.06166, 50.78033,0));
        points.add(new Point3D(6.06155, 50.78026, 0));

        ArrayList<Point3D> kPoints = new ArrayList<>();

        for(Point3D p : points) {
            kPoints.add(converter.convertLongLatPoint(p));
        }

        for(int i = 0; i < points.size(); i++) {
            Point3D p = points.get(i);
            Point3D k = kPoints.get(i);

            assertEquals(p.getY(), converter.convertYToLat(k.getY()), 0.0001);
            assertEquals(p.getX(), converter.convertXToLong(k.getX(), k.getY()), 0.0001);

            Point3D lonLatPointConvertedFromXYPoint = converter.convertXYPoint(k);
            assertEquals(p.getX(), lonLatPointConvertedFromXYPoint.getX(), 0.0001);
            assertEquals(p.getY(), lonLatPointConvertedFromXYPoint.getY(), 0.0001);
        }
    }
}