package de.monticore.lang.monticar.generator.dynamics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.FixMethodOrder;
import org.junit.Test;
import org.junit.runners.MethodSorters;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

@FixMethodOrder(MethodSorters.NAME_ASCENDING)
public class DynamicPortConnectionGenerationTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void Test_00_Exec() throws IOException {
        test("execOrder.not", "./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_00_Exec");
    }

    @Test
    public void Test_01_Not() throws IOException {
        test("portRequest.not", "./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_01_Not");
    }

    @Test
    public void Test_02_PortRequest1() throws IOException {
        test("portRequest.portRequest1", "./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_02_PortRequest1");
    }

    @Test
    public void Test_02_PortRequest2() throws IOException {
        test("portRequest.portRequest2", "./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_02_PortRequest2");
    }

    @Test
    public void Test_03_PortRequest3() throws IOException {
        test("portRequest.portRequest3", "./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_03_PortRequest3");
    }

    @Test
    public void Test_04_PortRequest4() throws IOException {
        test("portRequest.portRequest4", "./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_04_PortRequest4");
    }

    protected void test(String instName, String target){
        try {
            TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

            EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(instName, EMAComponentInstanceSymbol.KIND).orElse(null);
            assertNotNull(componentSymbol);
            GeneratorCPP generatorCPP = new GeneratorCPP();
            generatorCPP.useArmadilloBackend();
            generatorCPP.setGenerationTargetPath(target);
            List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
        }catch (IOException ex){
            ex.printStackTrace();
            Log.error(ex.getMessage());
        }
    }
}
