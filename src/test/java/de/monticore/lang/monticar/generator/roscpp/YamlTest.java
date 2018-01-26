package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.collect.Lists;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticar.lang.monticar.generator.python.TagReader;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.helper.ResolveHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.YamlHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.CommonSymbol;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static org.junit.Assert.*;

public class YamlTest extends AbstractSymtabTest {

    @Test
    public void convertYamlTest() throws IOException {
        //Setup
        TagReader<RosTag> reader = new TagReader<>();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        //Execute
        List<RosTag> tags = reader.readYAML("src/test/resources/config/config.yaml");

        //Check
        assertTrue(tags.size() == 1);

        ResolvedRosTag tag = ResolveHelper.resolveRosTag(tags.get(0), symtab);

        assertNotNull(tag.getComponent());
        assertEquals(tag.getComponent().getFullName(), "tests.a.compA");

        assertTrue(tag.getPublisherInterfaces().size() == 1);
        assertTrue(tag.getSubscriberInterfaces().size() == 1);

        ResolvedRosInterface subInter = tag.getSubscriberInterfaces().stream().findFirst().get();

        assertEquals(subInter.getTopic(), "test");
        assertEquals(subInter.getInclude(), "automated_driving_msgs/CarMessage");

        Map<String, PortSymbol> portNamesToPort = subInter.getPorts().stream().collect(Collectors.toMap(CommonSymbol::getName, p -> p));
        assertTrue(portNamesToPort.size() == 2);
        assertTrue(portNamesToPort.containsKey("rosIn"));
        assertTrue(portNamesToPort.containsKey("noRosIn"));

        PortSymbol rosIn = portNamesToPort.get("rosIn");
        PortSymbol noRosIn = portNamesToPort.get("noRosIn");
        assertEquals(subInter.getMsgFieldForPort(rosIn).getConversion(rosIn), "msg->posX");
        assertEquals(subInter.getMsgFieldForPort(noRosIn).getConversion(noRosIn), "msg->posY");


        ResolvedRosInterface pubInter = tag.getPublisherInterfaces().stream().findFirst().get();

        assertEquals(pubInter.getTopic(), "test2");
        assertEquals(pubInter.getInclude(), "automated_driving_msgs/StampedFloat64");

        portNamesToPort = pubInter.getPorts().stream().collect(Collectors.toMap(CommonSymbol::getName, p -> p));

        assertTrue(portNamesToPort.size() == 1);
        assertTrue(portNamesToPort.containsKey("rosOut"));

        PortSymbol rosOut = portNamesToPort.get("rosOut");
        assertEquals(pubInter.getMsgFieldForPort(rosOut).getConversion(rosOut), ".data = component.rosOut");
    }

    @Test
    public void arrayPortTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/arrayGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        //Execute
        List<File> files = YamlHelper.generateFromFile("src/test/resources/config/array.yaml", symtab, generatorRosCpp);

        //Check
        assertTrue(files.size() == 1);

        String fileContent = Files.readAllLines(files.get(0).toPath()).stream()
                .collect(StringBuilder::new, StringBuilder::append, StringBuilder::append)
                .toString();

        String[] positivePortNames = {"val1[0]", "val1[1]", "valOut[0]", "valOut[1]"};
        String[] negativePortNames = {"val1[2]", "valOut[2]"};

        for (String name : positivePortNames) {
            assertTrue(fileContent.contains(name));
        }

        for (String name : negativePortNames) {
            assertFalse(fileContent.contains(name));
        }
    }

    @Test
    public void arrayPortColonSyntaxTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/arraySyntaxGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        //Execute
        List<File> files = YamlHelper.generateFromFile("src/test/resources/config/arrayColonSyntax.yaml", symtab, generatorRosCpp);

        //Check
        assertTrue(files.size() == 1);

        String fileContent = Files.readAllLines(files.get(0).toPath()).stream()
                .collect(StringBuilder::new, StringBuilder::append, StringBuilder::append)
                .toString();

        String[] positiveNames = {"val1[0]", "val1[5]", "valOut[0]", "valOut[5]", "posX[0]", "posX[5]"};
        String[] negativeNames = {"val1[6]", "valOut[6]", "posX[6]"};

        for (String name : positiveNames) {
            assertTrue(fileContent.contains(name));
        }

        for (String name : negativeNames) {
            assertFalse(fileContent.contains(name));
        }
    }

    @Test
    public void multipleComponentsTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/multipleGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        //Execute
        List<File> files = YamlHelper.generateFromFile("src/test/resources/config/multipleComponents.yaml", symtab, generatorRosCpp);

        //Check
        assertTrue(files.size() == 2);

        List<String> fileNames = files.stream().map(File::getName).collect(Collectors.toList());
        List<String> positiveFileList = Lists.newArrayList("tests_a_compA_RosWrapper.h", "test_basicPorts_RosWrapper.h");

        assertTrue(fileNames.containsAll(positiveFileList));
    }

    @Test
    public void converterMethodTest() throws IOException {
        //Setup
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/converterMethodGenCpp/");
        generatorRosCpp.setGenerateCpp(false);

        //Execute
        List<File> files = YamlHelper.generateFromFile("src/test/resources/config/converterMethod.yaml", symtab, generatorRosCpp);

        //Check
        assertTrue(files.size() == 1);

        testFilesAreEqual(files, "converterMethod/");
    }
}
