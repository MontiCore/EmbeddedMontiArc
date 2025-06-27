/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.utilities;


import org.junit.Test;

import static junit.framework.TestCase.assertTrue;


public class TestStreamTestBuildMojo {

    @Test
    public void Test_01_execution_valid() {
        //valid
        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
        int r = BaseTest.validBuild("./src/test/resources/emam/execution/valid", "./target/tmp/build/01");
        assertTrue("Build-01: Build is invalid.["+r+"]", r==0);
    }

    @Test
    public void Test_02_execution_invalid() {
        int r = BaseTest.validBuild("./src/test/resources/emam/execution/invalid", "./target/tmp/build/02");
        assertTrue("Build-02: Build is invalid.["+r+"]", r==0);
    }

    @Test
    public void Test_03_many(){
        int r = BaseTest.validBuild("./src/test/resources/emam/execution/many", "./target/tmp/build/03");
        assertTrue("Build-03: Build is invalid.["+r+"]", r==0);
    }

    @Test
    public void Test_05_valid_struct(){
        int r = BaseTest.validBuild("./src/test/resources/emam/struct", "./target/tmp/generator/05");
        assertTrue("Build-05: Build is invalid.["+r+"]", r==0);
    }

//    @Test
//    public void Test_06_execution_valid_EMADL() {
//        //valid
//        //ValidInner("./src/test/resources/emam/execution/valid", "./target/tmp/streamtest-generator/01");
//        int r = BaseTest.validBuild("./src/test/resources/emadl/models", "./target/tmp/build/06", true);
//        assertTrue("Generator-06: Generator is invalid.["+r+"]", r==0);
//    }

    @Test
    public void Test_10_reRun(){
        Test_01_execution_valid();
        Test_02_execution_invalid();
        Test_03_many();
    }

    @Test
    public void Test_06_valid_cube(){
        int r = BaseTest.validBuild("./src/test/resources/emam/execution/cube", "./target/tmp/build/06");
        assertTrue("Build-06: Build is valid.["+r+"]", r==0);
    }
}
