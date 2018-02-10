package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.*;

public class CMakeGenerationTest extends AbstractSymtabTest {

    @Test
    public void testEchoCMake() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/echoCMake/");
        generatorRosCpp.setGenerateCMake(true);
        List<File> files = TagHelper.generate(generatorRosCpp, taggingResolver, componentInstanceSymbol);

        testFilesAreEqual(files, "echoCMake/");
    }

    @Test
    public void testDontGenerateCMake() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/noCMake/");
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);

        List<String> fileNames = files.stream()
                .map(File::getName)
                .collect(Collectors.toList());

        assertFalse(fileNames.contains("CMakeLists.txt"));
    }


    @Test
    public void testGenerateCMake() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/CMake/");
        generatorRosCpp.setGenerateCMake(true);
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);

        List<String> fileNames = files.stream()
                .map(File::getName)
                .collect(Collectors.toList());

        assertTrue(fileNames.contains("CMakeLists.txt"));
    }

}

