/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities.middleware;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.SystemUtils;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import static junit.framework.TestCase.assertTrue;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class TestMiddlewareBuildMojo extends MiddlewareBaseTest {
    protected final boolean RunTests = false;

    @Test
    public void Test_01_build_valid() {
        if(SystemUtils.IS_OS_WINDOWS){
            Log.info("streamtest-middleare-build (MiddlewareBuildMojo) is only available on Linux & MacOS!", "Test_01_build_valid: ");
            return;
        }
        if(RunTests) {
            //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
            int r = validMiddlewareBuild("./src/test/resources/emam/middleware",
                    "./target/tmp/middlewareBuild/01/streamtestsout",
                    "./target/tmp/middlewareBuild/01/middleware/",
                    "testA.and");
            assertTrue("MiddlewareBuild-01: Build is invalid.[" + r + "]", r == 0);
        }
    }






}
