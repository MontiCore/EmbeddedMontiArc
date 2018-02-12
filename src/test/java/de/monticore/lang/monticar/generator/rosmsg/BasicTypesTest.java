package de.monticore.lang.monticar.generator.rosmsg;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class BasicTypesTest extends AbstractSymtabTest {

    @Test
    public void testBasicTypes(){
        Scope symtab = createSymTab("src/test/resources/");
        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.basicTypes", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        GeneratorRosMsg generatorRosMsg = new GeneratorRosMsg();

        PortSymbol inQ = component.getPort("inQ").orElse(null);
        PortSymbol inZ = component.getPort("inZ").orElse(null);
        PortSymbol inB = component.getPort("inB").orElse(null);

        assertNotNull(inQ);
        assertNotNull(inZ);
        assertNotNull(inB);

        assertEquals(generatorRosMsg.getRosTopic(inQ.getTypeReference()).getTopicType(),"std_msgs/Float64");
        assertEquals(generatorRosMsg.getRosTopic(inZ.getTypeReference()).getTopicType(),"std_msgs/Int32");
        assertEquals(generatorRosMsg.getRosTopic(inB.getTypeReference()).getTopicType(),"std_msgs/Bool");
    }


}
