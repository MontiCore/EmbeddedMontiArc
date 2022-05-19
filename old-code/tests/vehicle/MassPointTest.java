/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.utils.Vec3;
import org.junit.*;
import de.rwth.montisim.simulation.util.Log;
import de.rwth.montisim.simulation.util.MathHelper;

import static org.junit.Assert.*;

/**
 * Class that tests the MassPoint class
 */
public class MassPointTest {

    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void constructorTest() {
        // Create test values
        Vec3 zeroVector = new Vec3(3);
        Vec3 localPos = new Vec3(new double[]{0.2346, 0.3678, 0.2486});
        double mass = 500.0;

        // Construct mass point
        MassPoint mp = new MassPoint(MassPointType.MASS_POINT_TYPE_WHEEL_FRONT_LEFT, localPos, mass);

        // Ensure that constructor works
        assertTrue(MathHelper.vectorEquals(localPos, mp.getLocalPosition(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector, mp.getLocalCenterDiff(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector, mp.getPosition(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector, mp.getCenterDiff(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector, mp.getVelocity(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector, mp.getForce(), 0.00000001));
        assertEquals(mass, mp.getMass(), 0);
        assertEquals(0.0, mp.getGroundZ(), 0);
        assertEquals(0.0, mp.getPressure(), 0);
    }

    @Test
    public void getterAndSetterTest() {
        // Create test values
        Vec3 zeroVector = new Vec3(3);
        Vec3 localPosition = new Vec3(new double[]{0.2346, 0.3678, 0.2486});
        Vec3 localCenterDiff = new Vec3(new double[]{5.2346, 0.678, 1.2486});
        Vec3 position = new Vec3(new double[]{249.2346, 10.3678, 3.2486});
        Vec3 centerDiff = new Vec3(new double[]{240.2346, 11.3678, 2.2486});
        Vec3 velocity = new Vec3(new double[]{0.0, 50.3, -0.25});
        double mass = 500.0;
        double groundZ = 2.1;
        double pressure = 1.9;

        // Construct mass point
        MassPoint mp = new MassPoint(MassPointType.MASS_POINT_TYPE_WHEEL_FRONT_LEFT, zeroVector, 0.0);

        // Set remaining values
        mp.setLocalPosition(localPosition);
        mp.setLocalCenterDiff(localCenterDiff);
        mp.setPosition(position);
        mp.setCenterDiff(centerDiff);
        mp.setVelocity(velocity);
        mp.setMass(mass);
        mp.setGroundZ(groundZ);
        mp.setPressure(pressure);

        // Test if setters work
        assertTrue(MathHelper.vectorEquals(localPosition, mp.getLocalPosition(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(localCenterDiff, mp.getLocalCenterDiff(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(position, mp.getPosition(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(centerDiff, mp.getCenterDiff(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(velocity, mp.getVelocity(), 0.00000001));
        assertEquals(mass, mp.getMass(), 0);
        assertEquals(groundZ, mp.getGroundZ(), 0);
        assertEquals(pressure, mp.getPressure(), 0);

        // Modify local copies of vectors, mass point values should not change!
        localPosition.setEntry(0, 0.3256);
        localCenterDiff.setEntry(0, 0.3256);
        position.setEntry(0, 0.3256);
        centerDiff.setEntry(0, 0.3256);
        velocity.setEntry(0, 0.3256);

        // Now the vectors should not be the same anymore
        assertFalse(MathHelper.vectorEquals(localPosition, mp.getLocalPosition(), 0.00000001));
        assertFalse(MathHelper.vectorEquals(localCenterDiff, mp.getLocalCenterDiff(), 0.00000001));
        assertFalse(MathHelper.vectorEquals(position, mp.getPosition(), 0.00000001));
        assertFalse(MathHelper.vectorEquals(centerDiff, mp.getCenterDiff(), 0.00000001));
        assertFalse(MathHelper.vectorEquals(velocity, mp.getVelocity(), 0.00000001));
    }

    @Test
    public void forceAdderAndResetterTest() {
        // Create test values
        Vec3 zeroVector = new Vec3(3);
        Vec3 force = new Vec3(new double[]{5.02, 0.1, -39.4});

        // Construct mass point
        MassPoint mp = new MassPoint(MassPointType.MASS_POINT_TYPE_WHEEL_BACK_LEFT, zeroVector, 0.0);

        // Add force
        mp.addForce(force);

        // Test adder
        assertTrue(MathHelper.vectorEquals(force, mp.getForce(), 0.00000001));

        // Add force again
        mp.addForce(force);

        // Test adder
        assertTrue(MathHelper.vectorEquals(force.add(force), mp.getForce(), 0.00000001));

        // Reset force
        mp.resetForce();

        // Test re setter
        assertTrue(MathHelper.vectorEquals(zeroVector, mp.getForce(), 0.00000001));
    }
}
