/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;

import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import static junit.framework.TestCase.assertTrue;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class TestStreamTestExecuteMojo  {


    @Test
    public void Test_01_execution_valid() {
        //valid
        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
        int r = BaseTest.validExecution("./src/test/resources/emam/execution/valid", "./target/tmp/exec/01");
        assertTrue("Exec-01: Execution is invalid.[r="+r+" | != 0]", r==0);
    }

    @Test
    public void Test_02_execution_invalid() {
        int r = BaseTest.validExecution("./src/test/resources/emam/execution/invalid", "./target/tmp/exec/02");
        assertTrue("Exec-02: Execution is invalid.[r="+r+" | != 0]", r==1);
    }

    @Test
    public void Test_03_many(){
        int r = BaseTest.validExecution("./src/test/resources/emam/execution/many", "./target/tmp/exec/03");
        assertTrue("Exec-03: Execution is invalid.[r="+r+" | != 0]", r==0);
    }

    @Test
    public void Test_05_valid_struct(){
        int r = BaseTest.validExecution("./src/test/resources/emam/struct", "./target/tmp/generator/05");
        assertTrue("Execution-05: Execution is invalid.["+r+"]", r==0);
    }

    @Test
    public void Test_10_reRun(){
        Test_01_execution_valid();
        Test_02_execution_invalid();
        Test_03_many();
    }

}
