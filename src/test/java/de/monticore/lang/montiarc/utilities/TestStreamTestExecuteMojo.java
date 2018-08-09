package de.monticore.lang.montiarc.utilities;


import de.monitcore.lang.montiarc.utilities.GeneratorEnum;
import de.monitcore.lang.montiarc.utilities.StreamTestBuildMojo;
import de.monitcore.lang.montiarc.utilities.StreamTestExecuteMojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;
import org.junit.Test;

import static junit.framework.TestCase.fail;

public class TestStreamTestExecuteMojo  {

    protected static StreamTestExecuteMojo getNewStreamTestMojo(String path, String pathOut){

        StreamTestExecuteMojo stm = new StreamTestExecuteMojo();

        stm.setPathMain(path+"/main");
        stm.setPathTest(path+"/test");

        stm.setPathTmpOut(pathOut);

        stm.setWrapperTestExtension("_TestWrapper");

        //stm.setGpp("g++");
        //stm.setCppInludePaths(new String[]{});
        //stm.setUsemingw(false);
        //
        //stm.addCppIncludePaths("/usr/local/Cellar/armadillo/8.500.1/include");

        stm.setGenerator(GeneratorEnum.VS2017);

        //use this in test to see all output
        stm.setShowBuildAndRunOutput(false);

        return stm;
    }

    @Test
    public void Test_01_execution_valid() {
        //valid
        ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/exec/01");
    }
    @Test
    public void Test_02_execution_valid() {
        //valid
        ValidInner("./src/test/resources/emam/execution/invalid", "./target/tmp/exec/02");
    }

    protected void ValidInner(String path, String tmp){
        //valid
        StreamTestExecuteMojo stm = getNewStreamTestMojo(path, tmp);

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
