/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import java.util.Map;
import java.util.stream.Collectors;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class Ros2Test extends AbstractSymtabTest {

    @Test
    public void testGenericCompInstance() {
        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.genericCompInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        RosMsg typeB = GeneratorRosMsg.getRosType("struct_msgs", component.getSubComponent("instB").get().getPortInstance("in1").get().getTypeReference(), true);
        RosMsg typeQ = GeneratorRosMsg.getRosType("struct_msgs", component.getSubComponent("instQ").get().getPortInstance("in1").get().getTypeReference(), true);
        RosMsg typeZ = GeneratorRosMsg.getRosType("struct_msgs", component.getSubComponent("instZ").get().getPortInstance("in1").get().getTypeReference(), true);

        assertTrue(typeB.getName().equals("std_msgs/msg/Bool"));
        assertTrue(typeQ.getName().equals("std_msgs/msg/Float64"));
        assertTrue(typeZ.getName().equals("std_msgs/msg/Int32"));
    }

    @Test
    public void testMatrixTypes() {
        Scope symtab = createSymTab("src/test/resources/");
        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("tests.matrixTypesComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        Map<String, RosMsg> portToMsg = component.getPortInstanceList().stream()
                .collect(Collectors.toMap(CommonSymbol::getName, p -> GeneratorRosMsg.getRosType("std_msgs", p.getTypeReference(), true)));

        assertTrue(portToMsg.get("in1").getName().equals("std_msgs/msg/Float64MultiArray"));
        assertTrue(portToMsg.get("in2").getName().equals("std_msgs/msg/Float64MultiArray"));
        assertTrue(portToMsg.get("out1").getName().equals("std_msgs/msg/ByteMultiArray"));
        assertTrue(portToMsg.get("out2").getName().equals("std_msgs/msg/Int32MultiArray"));
    }
}
