/* (c) https://github.com/MontiCore/monticore */
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


public class DynamicPortConnectionGenerationTest extends AbstractSymtabTest {

    protected String path(){
        return "src/test/resources/dynamics";
    }

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void Test_00_Exec() throws IOException {
        test("execOrder.not", "./target/generated-sources-cpp/dynamics/port-connect/test00");
    }

    @Test
    public void Test_01_Not() throws IOException {
        test("portRequest.not", "./target/generated-sources-cpp/dynamics/port-connect/test010");
    }

    @Test
    public void Test_02_PortRequest1() throws IOException {
        test("portRequest.portRequest1", "./target/generated-sources-cpp/dynamics/port-connect/test02_1");
    }

    @Test
    public void Test_02_PortRequest2() throws IOException {
        test("portRequest.portRequest2", "./target/generated-sources-cpp/dynamics/port-connect/test02_2");
    }

    @Test
    public void Test_03_PortRequest3() throws IOException {
        test("portRequest.portRequest3", "./target/generated-sources-cpp/dynamics/port-connect/test03");
    }

    @Test
    public void Test_04_PortRequest4() throws IOException {
        test("portRequest.portRequest4", "./target/generated-sources-cpp/dynamics/port-connect/test04");
    }

    @Test
    public void Test_05_PortRequest5() throws IOException {
        test("portRequest.portRequest5", "./target/generated-sources-cpp/dynamics/port-connect/test05");
    }

    @Test
    public void Test_06_PortRequest6() throws IOException {
        test("portRequest.portRequest6", "./target/generated-sources-cpp/dynamics/port-connect/test06");
    }

    protected void test(String instName, String target){
        try {
            TaggingResolver symtab = createSymTabAndTaggingResolver(path());

            EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(instName, EMAComponentInstanceSymbol.KIND).orElse(null);
            assertNotNull(componentSymbol);
            GeneratorCPP generatorCPP = new GeneratorCPP();
            generatorCPP.useArmadilloBackend();
            generatorCPP.setGenerationTargetPath(target);
            List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        }catch (IOException ex){
            ex.printStackTrace();
            Log.error(ex.getMessage());
        }
    }
}
