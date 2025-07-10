/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry;

import de.rwth.montisim.commons.utils.Vec2;
import org.junit.*;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;

import de.rwth.montisim.simulation.environment.geometry.height.ConcentricCircleGenerator;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.Bounds2D;


/**
 * Created by lukas on 16.02.17.
 */
public class ConcentricCircleGeneratorTest {

    @Test
    public void TestApp() {
        ConcentricCircleGenerator.init(new Bounds2D(0, 2 * Math.sqrt(2000), 0, 2 * Math.sqrt(2000), 0, 0), true);
        ConcentricCircleGenerator generator = ConcentricCircleGenerator.getInstance();


        assertEquals(8, generator.getNumberOfIntervals(), 0.0001);

        assertEquals(0, generator.getCircleIndex(Math.sqrt(2000) + 1, Math.sqrt(2000) + 5), 0.0001);
        assertEquals(1000.0, generator.getGround(Math.sqrt(2000) + 1, Math.sqrt(2000) + 5), 0.0001);

        assertEquals(1, generator.getCircleIndex(Math.sqrt(2000) + 11, Math.sqrt(2000) + 15));
        assertEquals(generator.getGround(Math.sqrt(2) + 0.015, Math.sqrt(2) + 0.011), generator.getGround(Math.sqrt(2) + 0.011, Math.sqrt(2) + 0.015), 0.0001);

        assertEquals(generator.getGround(0, 0), generator.getGround(2 * Math.sqrt(2000), 2 * Math.sqrt(2000)), 0.0001);


        ConcentricCircleGenerator.init(new Bounds2D(0, 4000, 0, 4000, 0, 0), true);
        generator = ConcentricCircleGenerator.getInstance();

        assertEquals(generator.getGround(2000, 2000 + new Vec2(2000, 2000).distance(4000, 4000)), generator.getGround(4000, 4000), Math.pow(10, -12));

        for (int i = 0; i < generator.getNumberOfIntervals(); i++) {
            assertTrue(generator.getGround(2000, 2000 + (i * 10) + 1) < (1000 + (i + 1) * 1));
            if (i < 2) {
                assertTrue(generator.getGround(2000, 2000 + (i * 10) + 1) >= 1000);
            } else {
                assertTrue(generator.getGround(2000, 2000 + (i * 10) + 1) >= (1000 + (i - 1) * ConcentricCircleGenerator.fixedSlope));
            }
            assertEquals(generator.getCircleIndex(2000, 2000 + (i * 10) + 1), i, 0.0001);

        }
    }
}
