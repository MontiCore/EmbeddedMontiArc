/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities.middleware;

import de.monitcore.lang.monticar.utilities.middleware.MiddlewareBuildMojo;
import de.monitcore.lang.monticar.utilities.middleware.MiddlewareGenerator;
import de.monitcore.lang.monticar.utilities.middleware.MiddlewareGeneratorMojo;
import de.monitcore.lang.monticar.utilities.middleware.MiddlewareMojoBase;
import de.monticore.lang.monticar.utilities.BaseTest;

public class MiddlewareBaseTest extends BaseTest {

    //<editor-fold desc="Test Setup">

    public static MiddlewareGeneratorMojo getMiddlewareGeneratorMojo(String path, String pathOut, String middlewareOut, String... rootModels){

        MiddlewareGeneratorMojo stmb = new MiddlewareGeneratorMojo();

        setupForMiddleware(stmb, path, pathOut, middlewareOut, rootModels);

        return stmb;
    }


    public static MiddlewareBuildMojo getMiddlewareBuildMojo(String path, String pathOut, String middlewareOut, String... rootModels){

        MiddlewareBuildMojo mbm = new MiddlewareBuildMojo();

        setupForMiddleware(mbm, path, pathOut, middlewareOut, rootModels);

        return mbm;
    }


    protected static void setupForMiddleware(MiddlewareMojoBase mmb, String path, String pathOut, String middlewareOut, String... rootModels){

        setup(mmb, path, pathOut);

        mmb.setPathMiddlewareOut(middlewareOut);
        mmb.addMiddlewareGenerator(MiddlewareGenerator.cpp);
        mmb.addMiddlewareGenerator(MiddlewareGenerator.roscpp);
        mmb.addMiddlewareGenerator(MiddlewareGenerator.emadlcpp);
        mmb.setEmadlBackend("GLUON");
        mmb.setRunStreamTestBefore(false);


        for (String model : rootModels){
            mmb.addMiddlewareRootModel(model);
        }
    }

    //</editor-fold>


    //<editor-fold desc="valid checker">

    protected static int validMiddlewareGenerator(String path, String tmp, String middlewareOut, String... rootModels){
        //valid
        return valid(getMiddlewareGeneratorMojo(path,tmp, middlewareOut, rootModels));
    }

    protected static int validMiddlewareBuild(String path, String tmp, String middlewareOut, String... rootModels){
        //valid
        return valid(getMiddlewareBuildMojo(path,tmp, middlewareOut, rootModels));
    }

    //</editor-fold>




}
