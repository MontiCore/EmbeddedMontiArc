/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.dynamics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;


public class DynamicIsConnectedInMathTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void test_00_IsConnected() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("connected.main", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/isconnected/test00");
//            generatorCPP.setUseThreadingOptimization(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }

    @Test
    public void test_01_IsConnected() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("connected.test", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/isconnected/test01");
//            generatorCPP.setUseThreadingOptimization(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }

    @Test
    public void test_02_IsConnectedScatter() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("connected.scatterTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/isconnected/test02");
//            generatorCPP.setUseThreadingOptimization(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }
}
