/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.dynamics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.FixMethodOrder;
import org.junit.Ignore;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import java.io.File;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.util.List;

import static org.junit.Assert.assertNotNull;


public class DynamicPortConnectDynamicInstanceTest extends AbstractSymtabTest {

    @Test
    public void Test_00_Test1() throws IOException {
        test("instanceRequest.test1", "./target/generated-sources-cpp/dynamics/instances/test00");
    }

    @Test
    public void Test_02_Test2() throws IOException {
        test("instanceRequest.test2", "./target/generated-sources-cpp/dynamics/instances/test02");
    }

    @Test
    public void Test_03_Test3() throws IOException {
        test("instanceRequest.test3", "./target/generated-sources-cpp/dynamics/instances/test03");
    }

    @Test
    @Ignore
    public void Test_04_Big() throws IOException {
        Instant start = Instant.now();
        test("instanceRequest.testBig", "./target/generated-sources-cpp/dynamics/instances/test04");
        Instant end = Instant.now();
        Log.info("TestBig time: "+Duration.between(start, end), "Test_04_Big");
    }

    protected void test(String instName, String target){
        try {
            TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

            EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(instName, EMAComponentInstanceSymbol.KIND).orElse(null);
            assertNotNull(componentSymbol);
            GeneratorCPP generatorCPP = new GeneratorCPP();
            generatorCPP.useArmadilloBackend();
            generatorCPP.setGenerationTargetPath(target);
//            generatorCPP.setUseThreadingOptimization(true);
            List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        }catch (IOException ex){
            ex.printStackTrace();
            Log.error(ex.getMessage());
        }
    }



}
