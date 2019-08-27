/**
 *
 * /* (c) https://github.com/MontiCore/monticore */
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package simulation.runner;

import org.junit.Test;

public class RunnerTest {

    @Test
    public void run() {
        double finalDistanceToTarget = new Runner(
                "/straight.osm",
                5170132468L,
                4188028027L,
                false,
                "autopilot",
                10101
        ) .run();
        assert  finalDistanceToTarget < 1;

        finalDistanceToTarget = new Runner(
                "/straight.osm",
                5170132468L,
                4188028027L,
                true,
                "autopilot",
                10101
        ) .run();
        assert  finalDistanceToTarget < 1;
    }
}
