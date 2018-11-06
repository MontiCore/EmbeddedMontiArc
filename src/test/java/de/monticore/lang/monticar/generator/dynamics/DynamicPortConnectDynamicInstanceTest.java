package de.monticore.lang.monticar.generator.dynamics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class DynamicPortConnectDynamicInstanceTest extends AbstractSymtabTest {

    @Test
    public void Test_00_Test1() throws IOException {
        test("instanceRequest.test1", "./target/generated-sources-cpp/dynamics/DynamicPortConnectDynamicInstanceTest_Test_00_Test1");
    }

    @Test
    public void Test_02_Test2() throws IOException {
        test("instanceRequest.test2", "./target/generated-sources-cpp/dynamics/DynamicPortConnectDynamicInstanceTest_Test_02_Test2");
    }

    @Test
    public void Test_03_Test3() throws IOException {
        test("instanceRequest.test3", "./target/generated-sources-cpp/dynamics/DynamicPortConnectDynamicInstanceTest_Test_03_Test3");
    }

    @Test
    public void Test_04_Big() throws IOException {
        test("instanceRequest.testBig", "./target/generated-sources-cpp/dynamics/DynamicPortConnectDynamicInstanceTest_Test_04_Big");
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
            List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        }catch (IOException ex){
            ex.printStackTrace();
            Log.error(ex.getMessage());
        }
    }



}
