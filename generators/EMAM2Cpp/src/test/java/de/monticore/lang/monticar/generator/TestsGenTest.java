/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import static org.junit.Assert.*;

public class TestsGenTest extends AbstractSymtabTest {

    private static final Path MODELS_DIR_PATH = Paths.get("src/test/resources/testgentest");

    @Test
    public void testMySuperAwesomeComponent1() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver(MODELS_DIR_PATH.toString());
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.subpackage1.mySuperAwesomeComponent1",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(MODELS_DIR_PATH);
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateCMake(false);
        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MySuperAwesomeComponent1/");
        generatorCPP.setCheckModelDir(true);
        Set<File> files = new HashSet<>(generatorCPP.generateFiles(symTab, componentSymbol));

//        assertEquals(18, files.size());
        assertEquals(13, files.size()); // TODO: check if 14 is correct here?
    }

    @Test
    public void testSimpleStructComp() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "structs.simpleStructComp",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/structs/simpleStructComp");
        generatorCPP.setCheckModelDir(true);
        Set<File> files = new HashSet<>(generatorCPP.generateFiles(symTab, componentSymbol));

        Optional<File> testFileOpt = files.stream().filter(file -> file.getName().endsWith("structs_simpleStructComp_test.hpp")).findFirst();

        assertTrue(testFileOpt.isPresent());
        List<String> content = FileUtils.readLines(testFileOpt.get(), "UTF-8");

        assertTrue(content.stream().anyMatch(line -> line.contains("component.in1.field = 1.0;")));
        assertTrue(content.stream().anyMatch(line -> line.contains("component.in1.field = 2.0;")));
        assertTrue(content.stream().anyMatch(line -> line.contains("component.in1.field = 10.0;")));
    }

    @Test
    public void testNestedStructComp() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "structs.nestedStructComp",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/structs/nestedStructComp");
        generatorCPP.setCheckModelDir(true);
        Set<File> files = new HashSet<>(generatorCPP.generateFiles(symTab, componentSymbol));

        Optional<File> testFileOpt = files.stream().filter(file -> file.getName().endsWith("structs_nestedStructComp_test.hpp")).findFirst();

        assertTrue(testFileOpt.isPresent());
        List<String> content = FileUtils.readLines(testFileOpt.get(), "UTF-8");

        assertTrue(content.stream().anyMatch(line -> line.contains("component.in1.simpleField = 1.0;")));
        assertTrue(content.stream().anyMatch(line -> line.contains("component.in1.structField.field = 3.0;")));

        assertTrue(content.stream().anyMatch(line -> line.contains("component.in1.simpleField = 2.0;")));
        assertTrue(content.stream().anyMatch(line -> line.contains("component.in1.structField.field = 4.0;")));
    }

    @Test
    public void testMatrixComp() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/matrixStreamBase/");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "basic.matrixComp",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources/matrixStreamBase/"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamTest/matrix/");
        generatorCPP.setCheckModelDir(true);
        Set<File> files = new HashSet<>(generatorCPP.generateFiles(symTab, componentSymbol));

        Optional<File> testFileOpt = files.stream().filter(file -> file.getName().endsWith("basic_matrixComp_test.hpp")).findFirst();

        assertTrue(testFileOpt.isPresent());
        List<String> content = FileUtils.readLines(testFileOpt.get(), "UTF-8");

        assertFalse(content.stream().anyMatch(line -> line.contains("NOT HANDLED VALUEPAIROPT!!!")));
        assertTrue(content.stream().anyMatch(line -> line.matches(".*component.colVecIn\\s*<<\\s*1.0\\s*<<\\s*2.0\\s*<<\\s*3.0\\s*<<\\s*arma::endr\\s*;.*")));
        assertTrue(content.stream().anyMatch(line -> line.matches(".*component.matIn\\s*<<\\s*1.0\\s*<<\\s*2.0\\s*<<\\s*3.0\\s*<<\\s*arma::endr\\s*<<\\s*4.0\\s*<<\\s*5.0\\s*<<\\s*6.0\\s*<<\\s*arma::endr\\s*<<\\s*7.0\\s*<<\\s*8.0\\s*<<\\s*9.0\\s*<<\\s*arma::endr\\s*;.*")));
        assertTrue(content.stream().anyMatch(line -> line.matches(".*component.rowVecIn\\s*<<\\s*1.0\\s*<<\\s*arma::endr\\s*<<\\s*2.0\\s*<<\\s*arma::endr\\s*<<\\s*3.0\\s*<<\\s*arma::endr\\s*;.*")));
    }

    @Test
    public void testCubeComp() throws IOException {
        Log.initDEBUG();
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/cube/");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "basic.cube",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources/cube/"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamTest/cube/");
        generatorCPP.setCheckModelDir(true);
        Set<File> files = new HashSet<>(generatorCPP.generateFiles(symTab, componentSymbol));

    }

    @Test
    public void testCubeComp2() throws IOException {
        Log.initDEBUG();
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/valid/main", "src/test/resources/valid/test");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "testA.and_TestWrapper",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources/cube/"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/streamTest/cube/");
        generatorCPP.setCheckModelDir(true);
        Set<File> files = new HashSet<>(generatorCPP.generateFiles(symTab, componentSymbol));
    }
}
