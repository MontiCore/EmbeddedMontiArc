package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;

import static org.junit.Assert.*;

public class TaggingTest extends AbstractSymtabTest {

    @Test
    public void testRosConnectionParsing() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        //rosIn
        PortSymbol rosIn = component.getPort("rosIn").orElse(null);
        assertNotNull(rosIn);

        Collection<TagSymbol> tags = symtab.getTags(rosIn, RosConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        RosConnectionSymbol tag = (RosConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName(), "/clock");
        assertEquals(tag.getTopicType(), "rosgraph_msgs/Clock");
        assertEquals(tag.getMsgField().get(), "clock.toSec()");

        //rosOut
        PortSymbol rosOut = component.getPort("rosOut").orElse(null);
        assertNotNull(rosOut);

        tags = symtab.getTags(rosOut, RosConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (RosConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName(), "/echo");
        assertEquals(tag.getTopicType(), "automated_driving_msgs/StampedFloat64");
        assertEquals(tag.getMsgField().get(), "data");

    }

    @Test
    public void testRosConnectionParsingOptionalMsgField() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.tagging.optionalMsgField", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        PortSymbol in1 = component.getPort("in1").orElse(null);
        PortSymbol out1 = component.getPort("out1").orElse(null);

        assertNotNull(in1);
        assertNotNull(out1);

        RosConnectionSymbol tagIn1 = (RosConnectionSymbol) symtab.getTags(in1, RosConnectionSymbol.KIND).stream().findFirst().orElse(null);
        RosConnectionSymbol tagOut1 = (RosConnectionSymbol) symtab.getTags(out1, RosConnectionSymbol.KIND).stream().findFirst().orElse(null);

        assertNotNull(tagIn1);
        assertNotNull(tagOut1);

        assertFalse(tagIn1.getMsgField().isPresent());
        assertTrue(tagOut1.getMsgField().isPresent());
        assertTrue(tagOut1.getMsgField().get().equals("msgField1"));

    }

    @Test
    public void testRosConnectionGeneration() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/echoTaggingCompRos/");

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        List<File> files = TagHelper.generate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "echo/");
    }

    @Test
    public void testBasicStruct() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        String targetPath = "./target/generated-sources-roscpp/basicStructCompRos/";
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath(targetPath);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.structs.basicStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        List<File> files = TagHelper.generate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "basicStruct/", targetPath);
    }

    @Test
    public void testNestedStruct() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        String targetPath = "./target/generated-sources-roscpp/nestedStructCompRos/";
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath(targetPath);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.structs.nestedStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        List<File> files = TagHelper.generate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "nestedStruct/", targetPath);
    }

    @Ignore
    @Test
    public void testArrayStruct() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        String targetPath = "./target/generated-sources-roscpp/arrayStructCompRos/";
        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath(targetPath);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.structs.arrayStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        List<File> files = TagHelper.generate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "arrayStruct/", targetPath);


    }
}
