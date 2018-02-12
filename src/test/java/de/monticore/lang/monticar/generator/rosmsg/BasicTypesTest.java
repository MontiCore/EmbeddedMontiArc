package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class BasicTypesTest extends AbstractSymtabTest {

    @Test
    public void testBasicTypes() throws IOException {
        Scope symtab = createSymTab("src/test/resources/");
        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.basicTypesComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();

        PortSymbol inQ = component.getPort("inQ").orElse(null);
        PortSymbol inZ = component.getPort("inZ").orElse(null);
        PortSymbol inB = component.getPort("inB").orElse(null);

        assertNotNull(inQ);
        assertNotNull(inZ);
        assertNotNull(inB);

        assertEquals(generatorRosMsg.getRosType(inQ.getTypeReference()),"std_msgs/Float64");
        assertEquals(generatorRosMsg.getRosType(inZ.getTypeReference()),"std_msgs/Int32");
        assertEquals(generatorRosMsg.getRosType(inB.getTypeReference()),"std_msgs/Bool");
    }

    @Test
    public void testBasicStructComp() throws IOException {

        Scope symtab = createSymTab("src/test/resources/");
        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.basicStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        generatorRosMsg.setTarget("target/generated-sources-rosmsg/basic", "basic");

        PortSymbol in1 = component.getPort("in1").orElse(null);
        PortSymbol out1 = component.getPort("out1").orElse(null);

        assertNotNull(in1);
        assertNotNull(out1);

        assertEquals(generatorRosMsg.getRosType(in1.getTypeReference()), "basic/structs_BasicStruct");
        assertEquals(generatorRosMsg.getRosType(out1.getTypeReference()), "basic/structs_BasicStruct");

        List<File> files = generatorRosMsg.generate(in1.getTypeReference());
        testFilesAreEqual(files, "basicStruct/");
    }


    @Test
    public void testNestedStructComp() throws IOException {

        Scope symtab = createSymTab("src/test/resources/");
        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.nestedStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        generatorRosMsg.setTarget("target/generated-sources-rosmsg/nested", "nested");

        PortSymbol inNested = component.getPort("inNested").orElse(null);

        assertNotNull(inNested);

        assertEquals(generatorRosMsg.getRosType(inNested.getTypeReference()), "nested/structs_NestedStruct");

        List<File> files = generatorRosMsg.generate(inNested.getTypeReference());
        testFilesAreEqual(files, "nestedStruct/");
    }

    @Test
    public void testMultiNestedStructComp() throws IOException {

        Scope symtab = createSymTab("src/test/resources/");
        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.multiNestedStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        generatorRosMsg.setTarget("target/generated-sources-rosmsg/multinested", "multinested");

        PortSymbol inMultiNested = component.getPort("inMultiNested").orElse(null);

        assertNotNull(inMultiNested);

        assertEquals(generatorRosMsg.getRosType(inMultiNested.getTypeReference()), "multinested/structs_MultiNestedStruct");

        List<File> files = generatorRosMsg.generate(inMultiNested.getTypeReference());
        testFilesAreEqual(files, "multinestedStruct/");
    }

}
