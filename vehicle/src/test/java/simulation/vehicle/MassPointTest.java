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
package simulation.vehicle;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.junit.*;
import simulation.util.Log;
import simulation.util.MathHelper;
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
    public void constructorTest(){
        // Create test values
        RealVector zeroVector = new ArrayRealVector(3);
        RealVector localPos = new ArrayRealVector(new double[] {0.2346, 0.3678, 0.2486});
        double mass = 500.0;

        // Construct mass point
        MassPoint mp = new MassPoint(MassPointType.MASS_POINT_TYPE_WHEEL_FRONT_LEFT, localPos, mass);

        // Ensure that constructor works
        assertTrue(MathHelper.vectorEquals(localPos, mp.getLocalPosition(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector,mp.getLocalCenterDiff(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector,mp.getPosition(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector,mp.getCenterDiff(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector,mp.getVelocity(), 0.00000001));
        assertTrue(MathHelper.vectorEquals(zeroVector,mp.getForce(), 0.00000001));
        assertEquals(mass, mp.getMass(), 0);
        assertEquals(0.0, mp.getGroundZ(), 0);
        assertEquals(0.0, mp.getPressure(), 0);
    }

    @Test
    public void getterAndSetterTest() {
        // Create test values
        RealVector zeroVector = new ArrayRealVector(3);
        RealVector localPosition = new ArrayRealVector(new double[] {0.2346, 0.3678, 0.2486});
        RealVector localCenterDiff = new ArrayRealVector(new double[] {5.2346, 0.678, 1.2486});
        RealVector position = new ArrayRealVector(new double[] {249.2346, 10.3678, 3.2486});
        RealVector centerDiff = new ArrayRealVector(new double[] {240.2346, 11.3678, 2.2486});
        RealVector velocity = new ArrayRealVector(new double[] {0.0, 50.3, -0.25});
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
    public void forceAdderAndResetterTest(){
        // Create test values
        RealVector zeroVector = new ArrayRealVector(3);
        RealVector force = new ArrayRealVector(new double[] {5.02, 0.1, -39.4});

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