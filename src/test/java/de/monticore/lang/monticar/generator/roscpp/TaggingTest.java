package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.Map;

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
        assertEquals(tag.getTopicName().get(), "/clock");
        assertEquals(tag.getTopicType().get(), "rosgraph_msgs/Clock");
        assertEquals(tag.getMsgField().get(), "clock.toSec()");

        //rosOut
        PortSymbol rosOut = component.getPort("rosOut").orElse(null);
        assertNotNull(rosOut);

        tags = symtab.getTags(rosOut, RosConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (RosConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echo");
        assertEquals(tag.getTopicType().get(), "automated_driving_msgs/StampedFloat64");
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
        TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "echo/");
    }

    @Test
    public void testNoMsgFieldBasicTypesComp() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);


        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        generatorRosCpp.setGenerationTargetPath("./target/generated-sources-roscpp/basicTypesComp/");

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.msg.basicTypesComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<PortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "basicTypesComp/");

    }

    @Test
    public void testNoMsgFieldBasicStructComp() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);


        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String generationTargetPath = "./target/generated-sources-roscpp/basicStructComp/";
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);
        generatorRosCpp.setGenerateCMake(true);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.msg.basicStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<PortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "basicStructComp/", generationTargetPath);
    }

    @Test
    public void testNoMsgFieldNestedStructComp() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String generationTargetPath = "./target/generated-sources-roscpp/nestedStructComp/";
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.msg.nestedStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<PortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "nestedStructComp/", generationTargetPath);
    }

    @Test
    public void testNoMsgFieldMultiNestedStructComp() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String generationTargetPath = "./target/generated-sources-roscpp/multiNestedStructComp/";
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.msg.multiNestedStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<PortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "multiNestedStructComp/", generationTargetPath);
    }

    @Test
    public void testMatrixTypesComp() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String generationTargetPath = "./target/generated-sources-roscpp/matrixTypesComp/";
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.structs.matrixTypesComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<PortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "matrixTypesComp/", generationTargetPath);
    }
}
