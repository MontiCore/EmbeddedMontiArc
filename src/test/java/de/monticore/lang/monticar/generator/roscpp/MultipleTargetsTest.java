package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class MultipleTargetsTest extends AbstractSymtabTest {

    @Test
    public void generateCppTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/genCpp/");

        //Execute
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        String[] positiveFileNames = {"tests_a_compA.h", "tests_a_compA_RosWrapper.h"};

        Arrays.stream(positiveFileNames).forEach(positiveFileName -> {
            assertTrue(fileNames.contains(positiveFileName));
        });

    }

    @Test
    public void dontGenerateCppTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/dontGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        //Execute
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        String[] positiveFileNames = {"tests_a_compA_RosWrapper.h"};
        String[] negativeFileNames = {"tests_a_compA.h"};


        Arrays.stream(positiveFileNames).forEach(positiveFileName -> {
            assertTrue(fileNames.contains(positiveFileName));
        });

        Arrays.stream(negativeFileNames).forEach(negativeFileName -> {
            assertFalse(fileNames.contains(negativeFileName));
        });

    }

    @Test
    public void useDefaultGeneratorCPP() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/defaultGenCpp/");

        //Execute
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        String[] positiveFileNames = {"tests_a_compA.h", "tests_a_compA_RosWrapper.h", "HelperA.h"};
        String[] negativeFileNames = {"Helper.h"}; //Octave helper, standard is armadillo


        Arrays.stream(positiveFileNames).forEach(positiveFileName -> {
            assertTrue(fileNames.contains(positiveFileName));
        });

        Arrays.stream(negativeFileNames).forEach(negativeFileName -> {
            assertFalse(fileNames.contains(negativeFileName));
        });
    }

    @Test
    public void useCustomGeneratorCPP() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useOctaveBackend();

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/customGenCpp/");
        generatorRosCpp.setGeneratorCPP(generatorCPP);

        //Execute
        List<File> files = generatorRosCpp.generateFiles(componentInstanceSymbol, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        String[] positiveFileNames = {"tests_a_compA.h", "tests_a_compA_RosWrapper.h", "Helper.h"};
        String[] negativeFileNames = {"HelperA.h"}; //Armadillo helper

        Arrays.stream(positiveFileNames).forEach(positiveFileName -> {
            assertTrue(fileNames.contains(positiveFileName));
        });

        Arrays.stream(negativeFileNames).forEach(negativeFileName -> {
            assertFalse(fileNames.contains(negativeFileName));
        });

    }


}
