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

public class DynamicGenerationTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void Test_01_TestDynamic1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testdynamic1.not", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/test/Test_01_TestDynamic1");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }

    @Test
    public void Test_02_TestEventComponent1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testeventcomponent1.not", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/test/Test_02_TestEventComponent1");
        generatorCPP.setGenerateCMake(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }

    @Test
    public void Test_03_TestEventComponent1() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/dynamics");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testeventcomponent2.test", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dynamics/test/Test_03_TestEventComponent2");
        generatorCPP.setGenerateCMake(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
//        files.stream().forEach(f -> System.out.println("Generated: "+f.getName()));
    }
}
