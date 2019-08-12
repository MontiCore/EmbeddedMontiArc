package de.monticore.lang.monticar.emadl;

import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Ignore;
import org.junit.Test;

import java.nio.file.Paths;
import java.util.Arrays;
import java.util.stream.Collectors;

import static junit.framework.TestCase.assertTrue;

/**
 *
 */
public class IntegrationPythonWrapperTest extends AbstractSymtabTest {
    @Test
    public void testGluonReinforcementModelRosEnvironment() {
        Log.getFindings().clear();
        String[] args = {"-m", "src/test/resources/models/reinforcementModel", "-r", "torcs.agent.TorcsAgent", "-b", "GLUON", "-f", "n", "-c", "n"};
        EMADLGeneratorCli.main(args);
        assertTrue(Log.getFindings().stream().filter(Finding::isError).collect(Collectors.toList()).isEmpty());

        checkFilesAreEqual(
                Paths.get("./target/generated-sources-emadl"),
                Paths.get("./src/test/resources/target_code/gluon/reinforcementModel/torcs"),
                Arrays.asList(
                        "CMakeLists.txt",
                        "CNNBufferFile.h",
                        "torcs_agent_torcsAgent.cpp",
                        "torcs_agent_torcsAgent.h",
                        "torcs_agent_torcsAgent_dqn.h",
                        "torcs_agent_torcsAgent_policy.h",
                        "CNNCreator_torcs_agent_torcsAgent_dqn.py",
                        "CNNNet_torcs_agent_torcsAgent_dqn.py",
                        "CNNPredictor_torcs_agent_torcsAgent_dqn.h",
                        "CNNTrainer_torcs_agent_torcsAgent_dqn.py",
                        "CNNTranslator.h",
                        "HelperA.h",
                        "start_training.sh",
                        "reward/CMakeLists.txt",
                        "reward/HelperA.h",
                        "reward/torcs_agent_dqn_reward.cpp",
                        "reward/torcs_agent_dqn_reward.h",
                        "reward/pylib/CMakeLists.txt",
                        "reward/pylib/torcs_agent_dqn_reward_executor.cpp",
                        "reward/pylib/torcs_agent_dqn_reward_executor.h",
                        "reward/pylib/torcs_agent_dqn_reward_executor.i",
                        "reward/pylib/armanpy/armanpy.hpp",
                        "reward/pylib/armanpy/armanpy.i",
                        "reward/pylib/armanpy/armanpy_1d.i",
                        "reward/pylib/armanpy/armanpy_2d.i",
                        "reward/pylib/armanpy/armanpy_3d.i",
                        "reward/pylib/armanpy/numpy.i",
                        "reinforcement_learning/__init__.py",
                        "reinforcement_learning/strategy.py",
                        "reinforcement_learning/agent.py",
                        "reinforcement_learning/environment.py",
                        "reinforcement_learning/replay_memory.py",
                        "reinforcement_learning/util.py",
                        "reinforcement_learning/cnnarch_logger.py"
                )
        );
        assertTrue(Paths.get(
                "./target/generated-sources-emadl/reinforcement_learning/_torcs_agent_dqn_reward_executor.so")
                .toFile().exists());
        assertTrue(Paths.get(
                "./target/generated-sources-emadl/reinforcement_learning/torcs_agent_dqn_reward_executor.py")
                .toFile().exists());
    }
}