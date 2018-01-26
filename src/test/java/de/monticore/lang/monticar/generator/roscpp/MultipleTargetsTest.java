package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.collect.Lists;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertTrue;

public class MultipleTargetsTest extends AbstractSymtabTest {

    @Test
    public void generateCppTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        ResolvedRosTag resolvedRosTag = new ResolvedRosTag(componentInstanceSymbol);
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/genCpp/");

        //Execute
        List<File> files = generatorRosCpp.generateFiles(resolvedRosTag, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        List<String> positiveFileNames = Lists.newArrayList("tests_a_compA.h", "tests_a_compA_RosWrapper.h");

        fileNames.containsAll(positiveFileNames);
    }

    @Test
    public void dontGenerateCppTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        ResolvedRosTag resolvedRosTag = new ResolvedRosTag(componentInstanceSymbol);
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/dontGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        //Execute
        List<File> files = generatorRosCpp.generateFiles(resolvedRosTag, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        List<String> positiveFileNames = Lists.newArrayList("tests_a_compA_RosWrapper.h");
        List<String> negativeFileNames = Lists.newArrayList("tests_a_compA.h");

        assertTrue(fileNames.containsAll(positiveFileNames));
        assertTrue(Collections.disjoint(fileNames, negativeFileNames));

    }

    @Test
    public void useDefaultGeneratorCPP() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        ResolvedRosTag resolvedRosTag = new ResolvedRosTag(componentInstanceSymbol);
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/defaultGenCpp/");

        //Execute
        List<File> files = generatorRosCpp.generateFiles(resolvedRosTag, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        List<String> positiveFileNames = Lists.newArrayList("tests_a_compA.h", "tests_a_compA_RosWrapper.h", "HelperA.h");
        List<String> negativeFileNames = Lists.newArrayList("Helper.h"); //Octave helper, standard is armadillo

        assertTrue(fileNames.containsAll(positiveFileNames));
        assertTrue(Collections.disjoint(fileNames, negativeFileNames));

    }

    @Test
    public void useCustomGeneratorCPP() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        ResolvedRosTag resolvedRosTag = new ResolvedRosTag(componentInstanceSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useOctaveBackend();

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/customGenCpp/");
        generatorRosCpp.setGeneratorCPP(generatorCPP);

        //Execute
        List<File> files = generatorRosCpp.generateFiles(resolvedRosTag, symtab);

        //Check
        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        List<String> positiveFileNames = Lists.newArrayList("tests_a_compA.h", "tests_a_compA_RosWrapper.h", "Helper.h");
        List<String> negativeFileNames = Lists.newArrayList("HelperA.h"); //Armadillo helper

        assertTrue(fileNames.containsAll(positiveFileNames));
        assertTrue(Collections.disjoint(fileNames, negativeFileNames));

    }


}
