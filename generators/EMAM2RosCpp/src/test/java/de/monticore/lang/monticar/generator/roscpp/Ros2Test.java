/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class Ros2Test extends AbstractSymtabTest{

    //TODO: change resources/results/echoRos2 to ros2
    @Test
    public void echoCMakeRos2() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.compB", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-rclcpp/echoCMakeRos2/");
        generatorRosCpp.setGenerateCMake(true);
        generatorRosCpp.setRos2Mode(true);
        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, taggingResolver, componentInstanceSymbol);

        testFilesAreEqual(files, "echoCMakeRos2/");
    }
    @Test
    public void testGenerateCMakeRos2() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.compB", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-rclcpp/CMakeRos2/");
        generatorRosCpp.setGenerateCMake(true);
        generatorRosCpp.setRos2Mode(true);
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);

        List<String> fileNames = files.stream()
                .map(File::getName)
                .collect(Collectors.toList());

        assertTrue(fileNames.contains("CMakeLists.txt"));
    }

    @Test
    public void testBasicStructCompGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.structs.basicStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-rclcpp/basicStructComp/");
        generatorRosCpp.setGenerateCMake(true);
        generatorRosCpp.setRos2Mode(true);
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);

        List<String> fileNames = files.stream()
                .map(File::getName)
                .collect(Collectors.toList());

        assertTrue(fileNames.contains("CMakeLists.txt"));
    }

    @Test
    public void testNestedStructCompGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.structs.nestedStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-rclcpp/nestedStructComp/");
        generatorRosCpp.setGenerateCMake(true);
        generatorRosCpp.setRos2Mode(true);
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, taggingResolver);

        List<String> fileNames = files.stream()
                .map(File::getName)
                .collect(Collectors.toList());

        assertTrue(fileNames.contains("CMakeLists.txt"));
    }


}

