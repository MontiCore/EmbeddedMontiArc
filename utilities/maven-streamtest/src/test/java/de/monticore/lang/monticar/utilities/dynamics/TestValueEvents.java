/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities.dynamics;

import de.monticore.lang.monticar.utilities.BaseTest;
import org.junit.Test;

import static junit.framework.TestCase.assertTrue;

public class TestValueEvents extends BaseTest {


    @Test
    public void Test_01_execution_valid() {
        //valid
        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
        int r = BaseTest.validExecution("./src/test/resources/emam/dynamics/value", "./target/tmp/dynamics-value/01");
        assertTrue("Exec-01: Execution is invalid.[r="+r+" | != 0]", r==0);
    }

}
