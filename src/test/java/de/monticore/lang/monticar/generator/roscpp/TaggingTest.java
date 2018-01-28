package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosConnectionSymbol;
import de.monticore.lang.monticar.generator.roscpp.tagging.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.*;

public class TaggingTest extends AbstractSymtabTest {

    @Test
    public void testRosConnectionParsing() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        RosToEmamTagSchema.registerTagTypes(symtab);

        ExpandedComponentInstanceSymbol component = symtab.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        PortSymbol rosIn = component.getPort("rosIn").orElse(null);
        assertNotNull(rosIn);

        Collection<TagSymbol> tags = symtab.getTags(rosIn, RosConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        RosConnectionSymbol tag = (RosConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName(), "topicName1");
        assertEquals(tag.getTopicType(), "topicType1");
        assertEquals(tag.getMsgField(), "msgField1");

    }
}
