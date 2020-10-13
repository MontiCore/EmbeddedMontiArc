/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.monticar.generator.cpp.GeneratorEMAMOpt2CppCli;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.nio.file.Files;
import java.nio.file.Paths;

import static org.junit.Assert.assertTrue;

public class GeneratorEMAMOpt2CppCliTest extends BasicGenerationTest {

    @Test
    public void testTrajectoryControllerCli() {
        String targetDir = "target/generated-sources-cpp/cliTest/trajectoryControllerMPC/";
        String[] args = {
                "--models-dir=src/test/resources/",
                "--root-model=de.rwth.monticar.mpc.trajectoryControllerMPC",
                "--output-dir=" + targetDir,
                "--flag-generate-autopilot-adapter",
                "--flag-generate-cmake",
        };
        GeneratorEMAMOpt2CppCli.main(args);

        String[] positiveFileNames = {
                "CMakeLists.txt",
                "de_rwth_monticar_mpc_trajectoryControllerMPC.h",
                "AutopilotAdapter.h",
                "AutopilotAdapter.cpp",
        };

        for (String positiveFileName : positiveFileNames) {
            assertTrue(Files.exists(Paths.get(targetDir + positiveFileName)));
        }
    }
}
