package de.monticore.lang.montiarc.utilities.middleware;


import de.monitcore.lang.montiarc.utilities.middleware.MiddlewareGenerator;
import de.monitcore.lang.montiarc.utilities.middleware.MiddlewareGeneratorMojo;
import de.monticore.lang.montiarc.utilities.BaseTest;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import static junit.framework.TestCase.assertTrue;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class TestMiddlewareGeneratorMojo extends BaseTest {

    //<editor-fold desc="Test Setup">

    public static MiddlewareGeneratorMojo getMiddlewareGeneratorMojo(String path, String pathOut, String middlewareOut, String... rootModels){

        MiddlewareGeneratorMojo stmb = new MiddlewareGeneratorMojo();
        setup(stmb, path, pathOut);

        stmb.setPathMiddlewareOut(middlewareOut);
        stmb.setMiddlewareGenerator(MiddlewareGenerator.roscpp);

        stmb.setRunStreamTestBefore(false);


        for (String model : rootModels){
            stmb.addMiddlewareRootModel(model);
        }

        return stmb;
    }

    protected static int validMiddlewareGenerator(String path, String tmp, String middlewareOut, String... rootModels){
        //valid
        return valid(getMiddlewareGeneratorMojo(path,tmp, middlewareOut, rootModels));
    }

    //</editor-fold>



    @Test
    public void Test_01_execution_valid() {
        //valid
        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
        int r = validMiddlewareGenerator("./src/test/resources/emam/middleware",
                "./target/tmp/middlewareGenerator/01/streamtestsout",
                "./target/tmp/middlewareGenerator/01/middleware/",
                "testA.and");
        assertTrue("MiddlewareGenerator-01: Generator is invalid.["+r+"]", r==0);
    }

    @Test
    public void Test_02_execution_valid() {
        //valid
        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
        int r = validMiddlewareGenerator("./src/test/resources/emam/middleware",
                "./target/tmp/middlewareGenerator/02/streamtestsout",
                "./target/tmp/middlewareGenerator/02/middleware/",
                "hier.koennte.ihre.werbung.stehen");
        assertTrue("MiddlewareGenerator-02: Generator is invalid.["+r+"]", r==1);
    }

}
