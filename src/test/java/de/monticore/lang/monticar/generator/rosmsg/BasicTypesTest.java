/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

import static org.junit.Assert.*;

public class BasicTypesTest extends AbstractSymtabTest {

    @Test
    public void testBasicTypes() throws IOException {
        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.basicTypesComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        EMAPortSymbol inQ = component.getPortInstance("inQ").orElse(null);
        EMAPortSymbol inZ = component.getPortInstance("inZ").orElse(null);
        EMAPortSymbol inB = component.getPortInstance("inB").orElse(null);

        assertNotNull(inQ);
        assertNotNull(inZ);
        assertNotNull(inB);

        String packageName = "basic";
        RosMsg msgInQ = GeneratorRosMsg.getRosType(packageName, inQ.getTypeReference());
        RosMsg msgInZ = GeneratorRosMsg.getRosType(packageName, inZ.getTypeReference());
        RosMsg msgInB = GeneratorRosMsg.getRosType(packageName, inB.getTypeReference());

        assertNotNull(msgInQ);
        assertNotNull(msgInZ);
        assertNotNull(msgInB);

        assertEquals(msgInQ.getFields().size(), 1);
        assertEquals(msgInZ.getFields().size(), 1);
        assertEquals(msgInB.getFields().size(), 1);

        assertEquals(msgInQ.getFields().get(0).getName(), "data");
        assertEquals(msgInZ.getFields().get(0).getName(), "data");
        assertEquals(msgInB.getFields().get(0).getName(), "data");

        assertEquals(msgInQ.getFields().get(0).getType().getName(), "float64");
        assertEquals(msgInZ.getFields().get(0).getType().getName(), "int32");
        assertEquals(msgInB.getFields().get(0).getType().getName(), "bool");
    }

    @Test
    public void testMatrixTypes() {
        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.matrixTypesComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        Map<String, RosMsg> portToMsg = component.getPortInstanceList().stream()
                .collect(Collectors.toMap(CommonSymbol::getName, p -> GeneratorRosMsg.getRosType("std_msgs", p.getTypeReference())));

        assertTrue(portToMsg.get("in1").getName().equals("std_msgs/Float64MultiArray"));
        assertTrue(portToMsg.get("in2").getName().equals("std_msgs/Float64MultiArray"));
        assertTrue(portToMsg.get("out1").getName().equals("std_msgs/ByteMultiArray"));
        assertTrue(portToMsg.get("out2").getName().equals("std_msgs/Int32MultiArray"));
    }

    @Test
    public void testBasicStructComp() throws IOException {

        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.basicStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        String packageName = "basic";
        generatorRosMsg.setTarget("target/generated-sources-rosmsg/basic", packageName);

        EMAPortSymbol in1 = component.getPortInstance("in1").orElse(null);
        EMAPortSymbol out1 = component.getPortInstance("out1").orElse(null);

        assertNotNull(in1);
        assertNotNull(out1);

        RosMsg msgIn1 = GeneratorRosMsg.getRosType(packageName, in1.getTypeReference());
        RosMsg msgOut1 = GeneratorRosMsg.getRosType(packageName, out1.getTypeReference());

        assertEquals(msgIn1, msgOut1);
        assertEquals(msgIn1.getName(), "basic/structs_BasicStruct");

        assertTrue(Objects.equals(msgIn1, getBasicStruct(packageName)));

        List<File> files = generatorRosMsg.generate(in1.getTypeReference());
        testFilesAreEqual(files, "basicStruct/");
    }


    @Test
    public void testNestedStructComp() throws IOException {

        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.nestedStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        String packageName = "nested";
        generatorRosMsg.setTarget("target/generated-sources-rosmsg/nested", packageName);

        EMAPortSymbol inNested = component.getPortInstance("inNested").orElse(null);

        assertNotNull(inNested);

        RosMsg rosMsg = GeneratorRosMsg.getRosType(packageName, inNested.getTypeReference());
        assertEquals(rosMsg.getName(), "nested/structs_NestedStruct");

        assertEquals(rosMsg, getNestedStruct(packageName));


        List<File> files = generatorRosMsg.generate(inNested.getTypeReference());
        testFilesAreEqual(files, "nestedStruct/");
    }

    @Test
    public void testMultiNestedStructComp() throws IOException {

        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.multiNestedStructComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();
        String packageName = "multinested";
        generatorRosMsg.setTarget("target/generated-sources-rosmsg/multinested", packageName);

        EMAPortSymbol inMultiNested = component.getPortInstance("inMultiNested").orElse(null);

        assertNotNull(inMultiNested);

        assertEquals(GeneratorRosMsg.getRosType(packageName, inMultiNested.getTypeReference()), getMultinestedStruct(packageName));

        List<File> files = generatorRosMsg.generate(inMultiNested.getTypeReference());
        testFilesAreEqual(files, "multinestedStruct/");
    }

    @Test
    public void testGenericCompInstance() {
        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.genericCompInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        RosMsg typeB = GeneratorRosMsg.getRosType("struct_msgs", component.getSubComponent("instB").get().getPortInstance("in1").get().getTypeReference());
        RosMsg typeQ = GeneratorRosMsg.getRosType("struct_msgs", component.getSubComponent("instQ").get().getPortInstance("in1").get().getTypeReference());
        RosMsg typeZ = GeneratorRosMsg.getRosType("struct_msgs", component.getSubComponent("instZ").get().getPortInstance("in1").get().getTypeReference());

        assertTrue(typeB.getName().equals("std_msgs/Bool"));
        assertTrue(typeQ.getName().equals("std_msgs/Float64"));
        assertTrue(typeZ.getName().equals("std_msgs/Int32"));
    }

    public RosMsg getBasicStruct(String packageName) {
        RosMsg res = new RosMsg(packageName + "/structs_BasicStruct");
        res.addField(new RosField("fieldQ1", new RosType("float64")));
        res.addField(new RosField("fieldQ2", new RosType("float64")));
        res.addField(new RosField("fieldZ1", new RosType("int32")));
        res.addField(new RosField("fieldZ2", new RosType("int32")));
        res.addField(new RosField("fieldB1", new RosType("bool")));

        return res;
    }

    public RosMsg getNestedStruct(String packageName) {
        RosMsg res = new RosMsg(packageName + "/structs_NestedStruct");
        res.addField(new RosField("fieldNested1", getBasicStruct(packageName)));
        res.addField(new RosField("fieldNested2", getBasicStruct(packageName)));
        res.addField(new RosField("fieldQ", new RosType("float64")));
        res.addField(new RosField("fieldB", new RosType("bool")));
        res.addField(new RosField("fieldZ", new RosType("int32")));
        return res;
    }

    public RosMsg getMultinestedStruct(String packageName) {
        RosMsg res = new RosMsg(packageName + "/structs_MultiNestedStruct");
        res.addField(new RosField("fieldMultiNested1", getNestedStruct(packageName)));
        res.addField(new RosField("fieldMultiNested2", getNestedStruct(packageName)));
        res.addField(new RosField("fieldNested", getBasicStruct(packageName)));
        res.addField(new RosField("fieldQ", new RosType("float64")));
        return res;
    }


}
