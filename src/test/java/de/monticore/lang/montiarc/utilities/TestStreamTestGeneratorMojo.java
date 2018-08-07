package de.monticore.lang.montiarc.utilities;

import de.monitcore.lang.montiarc.utilities.GeneratorEnum;
import de.monitcore.lang.montiarc.utilities.StreamTestGeneratorMojo;
import de.monitcore.lang.montiarc.utilities.StreamTestMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.Test;

import static junit.framework.TestCase.fail;

public class TestStreamTestGeneratorMojo {

    protected static StreamTestGeneratorMojo getNewStreamTestMojo(String path, String pathOut){

        StreamTestGeneratorMojo stm = new StreamTestGeneratorMojo();

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
    public void Test_05_many(){
        try {
            getNewStreamTestMojo("./src/test/resources/emam/execution/many", "./target/tmp/streamtest-generator/05").execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
        } catch (MojoFailureException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void Test_01_execution_valid() {
        //valid
        ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
    }

    @Test
    public void Test_02_execution_invalid() {
        //valid

        StreamTestGeneratorMojo stm = new StreamTestGeneratorMojo();

        stm.setPathMain("./src/test/resources/emam/execute_ParserTests");
        stm.setPathTest("./src/test/resources/emam/execute_ParserTests");

        stm.setPathTmpOut("./target/tmp/02");

        stm.setWrapperTestExtension("_TestWrapper");

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

    protected void ValidInner(String path, String tmp){
        //valid
        StreamTestGeneratorMojo stm = getNewStreamTestMojo(path, tmp);

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
