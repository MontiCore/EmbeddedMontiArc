package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.monticar.generator.middleware.cli.DistributedTargetGeneratorCli;
import org.junit.Ignore;
import org.junit.Test;

@Ignore("Used for evaluation, nothing gets asserted")
public class EvaluationTest {

    @Test
    public void testPacman() {
        DistributedTargetGeneratorCli.main(new String[]{"./src/test/resources/config/evaluation/pacman.json"});
    }

    @Test
    public void testPacmanSilhouette() {
        DistributedTargetGeneratorCli.main(new String[]{"./src/test/resources/config/evaluation/pacmanSilhouette.json"});
    }

    @Test
    public void testAutopilot() {
        DistributedTargetGeneratorCli.main(new String[]{"./src/test/resources/config/evaluation/autopilot.json"});
    }

    @Test
    public void testAutopilotSilhouette() {
        DistributedTargetGeneratorCli.main(new String[]{"./src/test/resources/config/evaluation/autopilotSilhouette.json"});
    }

    @Test
    public void testSupermario() {
        DistributedTargetGeneratorCli.main(new String[]{"./src/test/resources/config/evaluation/supermario.json"});
    }

    @Test
    public void testSupermarioSilhouette() {
        DistributedTargetGeneratorCli.main(new String[]{"./src/test/resources/config/evaluation/supermarioSilhouette.json"});
    }

    @Ignore("Very long runtime(30min+)")
    @Test
    public void testDaimlerModel() {
        DistributedTargetGeneratorCli.main(new String[]{"./src/test/resources/config/evaluation/daimler.json"});
    }
}
