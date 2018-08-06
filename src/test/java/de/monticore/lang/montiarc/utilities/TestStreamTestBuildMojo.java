package de.monticore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.GeneratorEnum;
import de.monitcore.lang.montiarc.utilities.StreamTestBuildMojo;
import de.monitcore.lang.montiarc.utilities.StreamTestGeneratorMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.Test;

import static junit.framework.TestCase.fail;

public class TestStreamTestBuildMojo {

    protected static StreamTestBuildMojo getNewStreamTestMojo(String path, String pathOut){

        StreamTestBuildMojo stm = new StreamTestBuildMojo();

        stm.setPathMain(path+"/main");
        stm.setPathTest(path+"/test");

        stm.setPathTmpOut(pathOut);

        stm.setWrapperTestExtension("_TestWrapper");

        //stm.setGpp("g++");
        //stm.setCppInludePaths(new String[]{});
        //stm.setUsemingw(false);
        //
        //stm.addCppIncludePaths("/usr/local/Cellar/armadillo/8.500.1/include");

        stm.setGenerator(GeneratorEnum.MinGW);

        //use this in test to see all output
        stm.setShowBuildAndRunOutput(true);

        return stm;
    }

    @Test
    public void Test_01_execution_valid() {
        //valid
        ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/build/01");
    }

    protected void ValidInner(String path, String tmp){
        //valid
        StreamTestBuildMojo stm = getNewStreamTestMojo(path, tmp);

        try {
            stm.execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            fail(e.getMessage());
        } catch (MojoFailureException e) {
            e.printStackTrace();
            fail(e.getMessage());
        }
    }
}
