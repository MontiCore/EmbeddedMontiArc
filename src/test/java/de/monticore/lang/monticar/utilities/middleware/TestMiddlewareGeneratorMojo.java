/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities.middleware;


import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import static junit.framework.TestCase.assertTrue;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class TestMiddlewareGeneratorMojo extends MiddlewareBaseTest {


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
