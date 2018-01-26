package de.monticore.lang.monticar.generator.roscpp;

import com.google.common.collect.Lists;
import de.monticar.lang.monticar.generator.python.RosTag;
import de.monticar.lang.monticar.generator.python.TagReader;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class YamlTest extends AbstractSymtabTest {

    @Ignore
    @Test
    public void convertYamlTest() throws IOException {
        //Setup
        TagReader<RosTag> reader = new TagReader<>();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        //Execute
//        List<RosTag> tags = reader.readYAML("src/test/resources/config/config.yaml");
//        YamlHelper.rosTagToDataHelper(symtab, tags.get(0));
//
//        //Check
//        assertTrue(tags.size() == 1);
//
//        Set<RosTopic> topics = DataHelper.getTopics();
//        assertTrue(topics.size() == 2);
//
//        HashMap<String, RosTopic> topicNameToTopic = new HashMap<>();
//        topics.forEach(t -> topicNameToTopic.put(t.getName(), t));
//
//        assertTrue(topicNameToTopic.containsKey("test"));
//        assertTrue(topicNameToTopic.containsKey("test2"));
//
//        RosTopic topicTest = topicNameToTopic.get("test");
//        RosTopic topicTest2 = topicNameToTopic.get("test2");
//
//        assertTrue(topicTest.getRosType().equals("CarMessage"));
//        assertTrue(topicTest2.getRosType().equals("StampedFloat64"));
//
//        assertTrue(topicTest.getImportString().equals("automated_driving_msgs/CarMessage"));
//        assertTrue(topicTest2.getImportString().equals("automated_driving_msgs/StampedFloat64"));
//
//        HashMap<String, PortSymbol> portNameToPort = new HashMap<>();
//        topics.forEach(t -> t.getPorts().forEach(p -> portNameToPort.put(p.getName(), p)));
//
//        assertTrue(portNameToPort.containsKey("rosIn"));
//        assertTrue(portNameToPort.containsKey("noRosIn"));
//        assertTrue(portNameToPort.containsKey("rosOut"));
//
//        PortSymbol port1 = portNameToPort.get("rosIn");
//        PortSymbol port2 = portNameToPort.get("noRosIn");
//        PortSymbol port3 = portNameToPort.get("rosOut");
//
//        assertTrue(topicTest.getPorts().contains(port1));
//        assertTrue(topicTest.getPorts().contains(port2));
//        assertFalse(topicTest.getPorts().contains(port3));
//
//        assertFalse(topicTest2.getPorts().contains(port1));
//        assertFalse(topicTest2.getPorts().contains(port2));
//        assertTrue(topicTest2.getPorts().contains(port3));
//
//        assertEquals(DataHelper.getMsgFieldFromPort(port1).orElse(null), "posX");
//        assertEquals(DataHelper.getMsgFieldFromPort(port2).orElse(null), "posY");
//        assertEquals(DataHelper.getMsgFieldFromPort(port3).orElse(null), "data");

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
}
