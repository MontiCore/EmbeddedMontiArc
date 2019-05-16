package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.RewardFunctionSourceGenerator;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.stream.Collectors;

import static junit.framework.TestCase.assertTrue;
import static org.mockito.Mockito.mock;

public class IntegrationPythonWrapperTest extends AbstractSymtabTest{
    private RewardFunctionSourceGenerator rewardFunctionSourceGenerator;

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
        rewardFunctionSourceGenerator = mock(RewardFunctionSourceGenerator.class);
    }

    @Test
    public void testReinforcementConfigWithRewardGeneration() {
        Log.getFindings().clear();
        Path modelPath = Paths.get("src/test/resources/valid_tests");
        CNNTrain2Gluon trainGenerator = new CNNTrain2Gluon(rewardFunctionSourceGenerator);

        trainGenerator.generate(modelPath, "ReinforcementConfig1");

        assertTrue(Log.getFindings().stream().filter(Finding::isError).collect(Collectors.toList()).isEmpty());
        checkFilesAreEqual(
                Paths.get("./target/generated-sources-cnnarch"),
                Paths.get("./src/test/resources/target_code/ReinforcementConfig1"),
                Arrays.asList(
                        "CNNTrainer_reinforcementConfig1.py",
                        "start_training.sh",
                        "reinforcement_learning/__init__.py",
                        "reinforcement_learning/action_policy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py"
                        )
        );
        assertTrue(Paths.get("./target/generated-sources-cnnarch/reward/pylib").toFile().isDirectory());
    }
}
