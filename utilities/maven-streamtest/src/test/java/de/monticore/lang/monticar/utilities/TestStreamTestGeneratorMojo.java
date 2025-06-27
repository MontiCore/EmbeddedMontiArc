/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;


import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import static junit.framework.TestCase.assertTrue;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class TestStreamTestGeneratorMojo {



    @Test
    public void Test_01_execution_valid() {
        //valid
        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
//        Log.initDEBUG();
        int r = BaseTest.validGenerator("./src/test/resources/emam/execution/valid", "./target/tmp/generator/01");
        assertTrue("Generator-01: Generator is invalid.["+r+"]", r==0);
    }


//    @Test
//    public void Test_06_execution_valid() {
//        //valid
//        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
//        int r = BaseTest.validGenerator("./src/test/resources/emadl/models", "./target/tmp/generator/06", true);
//        assertTrue("Generator-06: Generator is invalid.["+r+"]", r==0);
//    }

    @Test
    public void Test_02_execution_invalid() {
        int r = BaseTest.validGenerator("./src/test/resources/emam/execution/invalid", "./target/tmp/generator/02");
        assertTrue("Generator-02: Generator is invalid.["+r+"]", r==0);
    }

    @Test
    public void Test_03_many(){
        int r = BaseTest.validGenerator("./src/test/resources/emam/execution/many", "./target/tmp/generator/03");
        assertTrue("Generator-03: Generator is invalid.["+r+"]", r==0);
    }


    @Test
    public void Test_04_invalid_emam(){
        int r = BaseTest.validGenerator("./src/test/resources/emam/execute_ParserTests", "./target/tmp/generator/04");
        assertTrue("Generator-04: Generator is invalid.["+r+"]", r==1);
    }

    @Test
    public void Test_05_valid_struct(){
        int r = BaseTest.validGenerator("./src/test/resources/emam/struct", "./target/tmp/generator/05");
        assertTrue("Generator-05: Generator is invalid.["+r+"]", r==0);
    }

    @Test
    public void Test_10_reRun(){
        Test_01_execution_valid();
        Test_02_execution_invalid();
        Test_03_many();
    }

    @Test
    public void Test_06_Cube() {
        int r = BaseTest.validGenerator("./src/test/resources/emam/execution/cubeInvalidFile", "./target/tmp/generator/06");
        assertTrue("Generator-06: Generator is invalid.["+r+"]", r==1);
    }
}
