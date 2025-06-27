/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import static org.junit.Assert.*;

public class TaggingTest extends AbstractSymtabTest {

    @Before
    public void clearLogs(){
        Log.getFindings().clear();
    }

    @Test
    public void testRosConnectionParsing() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.a.compA", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        //rosIn
        EMAPortSymbol rosIn = component.getPortInstance("rosIn").orElse(null);
        assertNotNull(rosIn);

        Collection<TagSymbol> tags = symtab.getTags(rosIn, RosConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        RosConnectionSymbol tag = (RosConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/clock");
        assertEquals(tag.getTopicType().get(), "rosgraph_msgs/Clock");
        assertEquals(tag.getMsgField().get(), "clock.toSec()");

        //rosOut
        EMAPortSymbol rosOut = component.getPortInstance("rosOut").orElse(null);
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

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.tagging.optionalMsgField", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        EMAPortSymbol in1 = component.getPortInstance("in1").orElse(null);
        EMAPortSymbol out1 = component.getPortInstance("out1").orElse(null);

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

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.a.compA", EMAComponentInstanceSymbol.KIND).orElse(null);
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

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.msg.basicTypesComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<EMAPortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

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

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.msg.basicStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<EMAPortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

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

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.msg.nestedStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<EMAPortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

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

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.msg.multiNestedStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<EMAPortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

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

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.structs.matrixTypesComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<EMAPortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "matrixTypesComp/", generationTargetPath);
    }

    @Test
    public void testArrayHandlingComp() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String generationTargetPath = "./target/generated-sources-roscpp/arrayHandlingComp/";
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.structs.arrayHandlingComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);
        Map<EMAPortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(symtab, component);

        List<File> files = TagHelper.resolveAndGenerate(generatorRosCpp, symtab, component);

        testFilesAreEqual(files, "arrayHandlingComp/", generationTargetPath);
    }


    @Test
    public void testMissingTopicName() throws IOException {
        Log.enableFailQuick(false);

        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String relResultPath = "missingTopicName/";
        String generationTargetPath = "./target/generated-sources-roscpp/" + relResultPath;
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);

        EMAComponentInstanceSymbol component = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.tagging.missingInfo", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        RosConnectionSymbol rosConnectionSymbol = new RosConnectionSymbol((String) null,"std_msgs/Float64","data");
        component.getPortInstance("missingTopicName").get().setMiddlewareSymbol(rosConnectionSymbol);

        TagHelper.resolveTags(taggingResolver,component);
        List<File> files = generatorRosCpp.generateFiles(component,taggingResolver);

        assertEquals(files.size(), 0);
        assertEquals(Log.getErrorCount(),1);
        assertTrue(Log.getFindings().get(0).getMsg().contains("0x9d80f"));

        Log.getFindings().clear();
        Log.enableFailQuick(true);
    }

    @Test
    public void testMissingTopicType() throws IOException {
        Log.enableFailQuick(false);

        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        GeneratorRosCpp generatorRosCpp = new GeneratorRosCpp();
        String relResultPath = "missingTopicType/";
        String generationTargetPath = "./target/generated-sources-roscpp/" + relResultPath;
        generatorRosCpp.setGenerationTargetPath(generationTargetPath);

        EMAComponentInstanceSymbol component = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.tagging.missingInfo", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        RosConnectionSymbol rosConnectionSymbol = new RosConnectionSymbol("/test",(String) null,"data");
        component.getPortInstance("missingTopicType").get().setMiddlewareSymbol(rosConnectionSymbol);

        TagHelper.resolveTags(taggingResolver,component);
        List<File> files = generatorRosCpp.generateFiles(component,taggingResolver);

        assertEquals(files.size(), 0);
        assertEquals(Log.getErrorCount(),1);
        assertTrue(Log.getFindings().get(0).getMsg().contains("0x2cc67"));

        Log.getFindings().clear();
        Log.enableFailQuick(true);
    }
}
