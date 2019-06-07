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
package simulation.environment.geometry;

import javafx.geometry.Point2D;
import junit.framework.TestCase;
import junit.framework.TestSuite;
import org.junit.Test;
import simulation.environment.geometry.height.ConcentricCircleGenerator;
import simulation.environment.visualisationadapter.implementation.Bounds2D;


/**
 * Created by lukas on 16.02.17.
 */
public class ConcentricCircleGeneratorTest extends TestCase {
    public ConcentricCircleGeneratorTest(String testName) {
        super(testName);
    }

    public static junit.framework.Test suite() {
        return new TestSuite(ConcentricCircleGeneratorTest.class);
    }

    @Ignore
    @Test
    public void testApp() {
        ConcentricCircleGenerator.init(new Bounds2D(0, 2*Math.sqrt(2000), 0, 2*Math.sqrt(2000), 0, 0), true);
        ConcentricCircleGenerator generator = ConcentricCircleGenerator.getInstance();


        assertEquals(8, generator.getNumberOfIntervals());

        assertEquals(0, generator.getCircleIndex(Math.sqrt(2000) + 1, Math.sqrt(2000) + 5));
        assertEquals(1000.0, generator.getGround(Math.sqrt(2000) + 1, Math.sqrt(2000) + 5));

        assertEquals(1, generator.getCircleIndex(Math.sqrt(2000) + 11, Math.sqrt(2000) + 15));
        assertEquals(generator.getGround(Math.sqrt(2) + 0.015, Math.sqrt(2) + 0.011), generator.getGround(Math.sqrt(2) + 0.011, Math.sqrt(2) + 0.015));

        assertEquals(generator.getGround(0,0), generator.getGround(2*Math.sqrt(2000),2*Math.sqrt(2000)));


        ConcentricCircleGenerator.init(new Bounds2D(0, 4000, 0, 4000, 0, 0), true);
        generator = ConcentricCircleGenerator.getInstance();

        assertEquals(generator.getGround(2000,2000 + new Point2D(2000,2000).distance(4000,4000)), generator.getGround(4000,4000), Math.pow(10, -12));

        for(int i = 0; i < generator.getNumberOfIntervals(); i++) {
            assertTrue(generator.getGround(2000, 2000+(i*10)+1) < (1000 + (i+1)*1));
            if(i < 2) {
                assertTrue(generator.getGround(2000, 2000+(i*10)+1) >= 1000);
            } else {
                assertTrue(generator.getGround(2000, 2000+(i*10)+1) >= (1000 + (i-1)*ConcentricCircleGenerator.fixedSlope));
            }
            assertEquals(generator.getCircleIndex(2000, 2000+(i*10)+1),i);

        }
    }
}