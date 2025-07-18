/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry;

import de.rwth.montisim.commons.map.IControllerNode;
import de.rwth.montisim.commons.utils.Vec3;
import org.junit.*;

import static org.junit.Assert.*;

import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.environment.geometry.splines.LinearInterpolator;
import de.rwth.montisim.simulation.environment.pedestrians.PedestrianStreetParameters;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvStreet;

import java.util.ArrayList;

/**
 * Created by lukas on 21.01.17.
 */
public class LinearInterpolatorTest {

    @Test
    public void testApp() throws Exception {
        LinearInterpolator interpol = new LinearInterpolator(new Vec3(3, 3, 0), new Vec3(1, 1, 0), EnvStreet.STREET_WIDTH, -1l, -1l, true);

        double dist = interpol.computeDistanceToMiddle(new Vec3(2, 1, 0));

        assertEquals(Math.sqrt(0.5), dist, 0.0001);


        Vec3 p = interpol.computePoint(8);
        assertEquals(0d, interpol.computeDistanceToMiddle(p), 0.0001);

        assertEquals(EnvStreet.STREET_WIDTH, interpol.computeDistanceToLeft(p) + interpol.computeDistanceToRight(p), LinearInterpolator.precision);

        interpol = new LinearInterpolator(new Vec3(-5, -1, 0), new Vec3(3, 0, 0), 0.006, -1l, -1l, true);

        assertFalse(interpol.isOnStreet(new Vec3(6, 0, 0)));

        assertTrue(interpol.isOnStreet(new Vec3(3, 0, 0)));

        interpol = new LinearInterpolator(new Vec3(0, 0, 0), new Vec3(1, 2, 0), 2 * Math.sqrt(5), -1l, -1l, true);

        assertTrue(interpol.isOnStreet(new Vec3(2, 0, 0)));

        assertFalse(interpol.isOnStreet(new Vec3(3, -2, 0)));

        interpol = new LinearInterpolator(new Vec3(-5, -1, 0), new Vec3(3, 0, 0), 0.006, -1l, -1l, true);


        assertEquals(-0d, interpol.computeT(new Vec3(-5, -1, 0)), 0.0001);

        assertEquals(0.5, interpol.computeT(new Vec3(-1, -0.5, 0)), 0.0001);

        assertEquals(1d, interpol.computeT(new Vec3(3, 0, 0)), 0.0001);

        p = new Vec3(2, 3, 4);

        assertEquals(interpol.computeDistanceToMiddle(p), interpol.computePoint(interpol.computeT(p)).distance(p), LinearInterpolator.precision);

        interpol = new LinearInterpolator(new Vec3(1, 1, 0), new Vec3(2, 2, 0), EnvStreet.STREET_WIDTH, -1, -1, true);
        ArrayList<IControllerNode> cNodes = interpol.convertToControllerList();
        for (int i = 0; i < interpol.convertToControllerList().size() - 1; i++) {
            IControllerNode c1 = interpol.convertToControllerList().get(i);
            IControllerNode c2 = interpol.convertToControllerList().get(i + 1);
            double cDist = c1.getPoint().distance(c2.getPoint());
            assertTrue(cDist <= IControllerNode.INTERPOLATION_DISTANCE + Math.pow(10, -10));
        }


        interpol = new LinearInterpolator(new Vec3(0, 0, 0), new Vec3(4, 4, 4), 3, -1, -1, true, 1.d);

        assertTrue(interpol.isOnStreet(new Vec3(3, 1, 2)));

        assertFalse(interpol.isOnStreet(new Vec3(4.5, 1.5, 3)));

        assertTrue(interpol.isOnPavement(new Vec3(4.5, 1.5, 3)));

        assertFalse(interpol.isOnPavement(new Vec3(4.5, 1.5, 2)));

        assertFalse(interpol.isOnPavement(new Vec3(4, 0, 2)));

        assertFalse(interpol.isOnStreet(new Vec3(4, 0, 2)));

        World world = (World) (World.getInstance());

        interpol = new LinearInterpolator(new Vec3(0, 0, 0), new Vec3(0, 4, 0), 3, -1, -1, true, 1.d);

        assertEquals(new Vec3(0.75, 1, 0), interpol.spawnCar(true, new Vec3(1, 1, 1)));

        interpol = new LinearInterpolator(new Vec3(0, 0, 0), new Vec3(8, 8, 0), 3, -1, -1, true, 1.d);

        Vec3 spawningPoint = interpol.spawnCar(true, new Vec3(1.5, 1.5, 0));
        Vec3 expectedPoint = new Vec3(2, 1, 0);

        assertEquals(expectedPoint.getX(), spawningPoint.getX(), 0.04);
        assertEquals(expectedPoint.getY(), spawningPoint.getY(), 0.04);
        assertEquals(expectedPoint.getZ(), spawningPoint.getZ(), 0.04);

        assertEquals(0.75, new Vec3(1.5, 1.5, 0).distance(spawningPoint), 0.04);


        spawningPoint = interpol.spawnCar(true, new Vec3(2, 2, 0));
        expectedPoint = new Vec3(2.5, 1.5, 0);

        assertEquals(expectedPoint.getX(), spawningPoint.getX(), 0.04);
        assertEquals(expectedPoint.getY(), spawningPoint.getY(), 0.04);
        assertEquals(expectedPoint.getZ(), spawningPoint.getZ(), 0.04);

        assertEquals(0.75, new Vec3(2, 2, 0).distance(spawningPoint), 0.04);

        spawningPoint = interpol.spawnCar(false, new Vec3(2, 2, 0));
        expectedPoint = new Vec3(1.5, 2.5, 0);

        assertEquals(expectedPoint.getX(), spawningPoint.getX(), 0.04);
        assertEquals(expectedPoint.getY(), spawningPoint.getY(), 0.04);
        assertEquals(expectedPoint.getZ(), spawningPoint.getZ(), 0.04);

        assertEquals(0.75, new Vec3(2, 2, 0).distance(spawningPoint), 0.04);

        spawningPoint = interpol.spawnCar(false, new Vec3(1.5, 1.5, 0));
        expectedPoint = new Vec3(1, 2, 0);

        assertEquals(expectedPoint.getX(), spawningPoint.getX(), 0.04);
        assertEquals(expectedPoint.getY(), spawningPoint.getY(), 0.04);
        assertEquals(expectedPoint.getZ(), spawningPoint.getZ(), 0.04);

        assertEquals(0.75, new Vec3(1.5, 1.5, 0).distance(spawningPoint), 0.04);

        boolean isLeft = true;
        LinearInterpolator pavement = interpol.getPavement(isLeft);
        Vec3 initPoint = pavement.computePoint(0.5);

        assertEquals(initPoint, interpol.computePointForPedestrian(new PedestrianStreetParameters(false, initPoint, true, isLeft), Math.sqrt(128) + 1).getPosition());

        assertTrue(initPoint.distance(pavement.getP1()) < interpol.computePointForPedestrian(new PedestrianStreetParameters(false, initPoint, true, isLeft), 1).getPosition().distance(pavement.getP1()));

        Vec3 newPoint = interpol.computePointForPedestrian(new PedestrianStreetParameters(true, initPoint, true, isLeft), 1).getPosition();
        assertTrue(interpol.computeDistanceToMiddle(initPoint) > interpol.computeDistanceToMiddle(newPoint));

        initPoint = interpol.computePoint(0.5);

        newPoint = interpol.computePointForPedestrian(new PedestrianStreetParameters(true, initPoint, true, isLeft), 2).getPosition();
        assertTrue(interpol.isOnPavement(newPoint));


        Vec3 p1 = new Vec3(-1, 1, 0);

        //System.out.println(p1.angle(0,1,0));
        //System.out.println(Math.toRadians(p1.angle(0,1,0)));

    }
}
