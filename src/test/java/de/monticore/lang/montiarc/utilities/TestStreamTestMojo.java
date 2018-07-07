package de.monticore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.StreamTestMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.Test;

public class TestStreamTestMojo {

    @Test
    public void Test() {

        StreamTestMojo stm = new StreamTestMojo();

        stm.setPathMain("./src/test/resources/emam/main");
        stm.setPathTest("./src/test/resources/emam/test");

        stm.setPathTmpOut("./target/tmp");

        stm.setWrapperTestExtension("_TestWrapper");

        stm.setGpp("g++");
        stm.setGppPathToArmadilloH("/usr/local/Cellar/armadillo/8.500.1/include");
        try {
            stm.execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
        } catch (MojoFailureException e) {
            e.printStackTrace();
        }
    }
}