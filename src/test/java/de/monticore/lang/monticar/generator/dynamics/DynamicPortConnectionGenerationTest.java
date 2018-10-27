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
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("execOrder.not", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_00_Exec");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
    }

    @Test
    public void Test_01_Not() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("portRequest.not", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_01_Not");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
    }


    @Test
    public void Test_02_PortRequest1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("portRequest.portRequest1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_02_PortRequest1");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
    }

    @Test
    public void Test_02_PortRequest2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("portRequest.portRequest2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/DynamicPortConnectionGenerationTest_Test_02_PortRequest2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol, symtab);
    }
}
