/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;


import de.monitcore.lang.monticar.utilities.*;
import de.monticore.lang.monticar.emadl.generator.Backend;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;


public class BaseTest {

    //<editor-fold desc="Static mojo creation">

    public static StreamTestGeneratorMojo getGeneratorMojo(String path, String pathOut){
        StreamTestGeneratorMojo stmb = new StreamTestGeneratorMojo();
        setup(stmb, path, pathOut);
        return stmb;
    }


    public static StreamTestBuildMojo getBuildMojo(String path, String pathOut){
        StreamTestBuildMojo stmb = new StreamTestBuildMojo();
        setup(stmb, path, pathOut);
        return stmb;
    }


    public static StreamTestExecuteMojo getExecutionMojo(String path, String pathOut){
        StreamTestExecuteMojo stmb = new StreamTestExecuteMojo();
        setup(stmb, path, pathOut);
        return stmb;
    }


    public static void setup(StreamTestMojoBase stmb, String path, String pathOut){
        stmb.setPathMain(path+"/main");
        stmb.setPathTest(path+"/test");

        stmb.setPathTmpOut(pathOut);

        stmb.setWrapperTestExtension("_TestWrapper");

        stmb.setGenerator(GeneratorEnum.MinGW);
        //stmb.setGenerator(GeneratorEnum.VS2017);

        //use this in test to see all output
        stmb.setShowBuildAndRunOutput(false);

        stmb.setForceRun(true);

        stmb.setShowDateAndTime(true);

        stmb.setBackend(Backend.GLUON);
        stmb.setTrainingNeeded(true);
        stmb.setPathToPython("/usr/bin/python");
        stmb.setRootModel("cNNCalculator.Connector");
    }

    //</editor-fold>


    protected static int valid(StreamTestMojoBase stmb){
        try {
            stmb.execute();
        } catch (MojoExecutionException e) {
            e.printStackTrace();
            return 1;
        } catch (MojoFailureException e) {
            e.printStackTrace();
            return 2;
        }
        return 0;
    }

    protected static int validGenerator(String path, String tmp){
        //valid
        return valid(getGeneratorMojo(path,tmp));
    }

    protected static int validBuild(String path, String tmp){
        //valid
        return valid(getBuildMojo(path,tmp));
    }

    protected static int validExecution(String path, String tmp){
        //valid
        return valid(getExecutionMojo(path,tmp));
    }
}
