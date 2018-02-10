package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.middleware.helpers.RosHelper;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import static org.junit.Assert.*;

public class MiddlewareSymbolTest extends AbstractSymtabTest {

    @Test
    public void testMWInferFromTags() {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.infer.inferComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        ExpandedComponentInstanceSymbol sub1 = componentInstanceSymbol.getSubComponent("sub1").orElse(null);
        assertNotNull(sub1);

        PortSymbol rosIn = componentInstanceSymbol.getPort("rosIn").orElse(null);
        PortSymbol rosOut = componentInstanceSymbol.getPort("rosOut").orElse(null);
        PortSymbol sub1RosIn = sub1.getPort("rosIn").orElse(null);
        PortSymbol sub1RosOut = sub1.getPort("rosOut").orElse(null);

        assertNotNull(rosIn);
        assertNotNull(rosOut);
        assertNotNull(sub1RosIn);
        assertNotNull(sub1RosOut);

        RosConnectionSymbol rosInRCS = (RosConnectionSymbol) rosIn.getMiddlewareSymbol().orElse(null);
        RosConnectionSymbol rosOutRCS = (RosConnectionSymbol) rosOut.getMiddlewareSymbol().orElse(null);
        RosConnectionSymbol sub1RosInRCS = (RosConnectionSymbol) sub1RosIn.getMiddlewareSymbol().orElse(null);
        RosConnectionSymbol sub1RosOutRCS = (RosConnectionSymbol) sub1RosOut.getMiddlewareSymbol().orElse(null);

        assertNotEquals(rosInRCS, sub1RosInRCS);
        assertNotEquals(rosOutRCS, sub1RosOutRCS);

        RosHelper.fixRosConnectionSymbols(componentInstanceSymbol);

        assertEquals(rosInRCS, sub1RosInRCS);
        assertEquals(rosOutRCS, sub1RosOutRCS);
    }
}
