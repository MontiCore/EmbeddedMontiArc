/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.middleware.someip;

import de.monticore.lang.embeddedmontiarc.AbstractTaggingResolverTest;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.Collection;
import java.util.Optional;

import static org.junit.Assert.*;

public class TaggingTest extends AbstractTaggingResolverTest {


    @Test
    public void testSomeIPConnectionParsing() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("middleware.someip.basicParsing", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        //someipIn
        EMAPortSymbol someIPIn = component.getPortInstance("someIPIn").orElse(null);
        assertNotNull(someIPIn);

        Collection<TagSymbol> tags = symtab.getTags(someIPIn, SomeIPConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        SomeIPConnectionSymbol tag = (SomeIPConnectionSymbol) tags.iterator().next();
        assertEquals((int)tag.getserviceID().get(), 1);
        assertEquals((int)tag.getinstanceID().get(), 2);
        assertEquals((int)tag.geteventgroupID().get(), 3);

        //someipOut
        EMAPortSymbol someIPOut = component.getPortInstance("someIPOut").orElse(null);
        assertNotNull(someIPOut);

        tags = symtab.getTags(someIPOut, SomeIPConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (SomeIPConnectionSymbol) tags.iterator().next();
        assertEquals((int)tag.getserviceID().get(), 1);
        assertEquals((int)tag.getinstanceID().get(), 2);
        assertEquals((int)tag.geteventgroupID().get(), 3);

        //emptyTagIn
        EMAPortSymbol emptyTagIn = component.getPortInstance("emptyTagIn").orElse(null);
        assertNotNull(emptyTagIn);

        tags = symtab.getTags(emptyTagIn, SomeIPConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (SomeIPConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getserviceID(), Optional.empty());
        assertEquals(tag.getinstanceID(), Optional.empty());
		assertEquals(tag.geteventgroupID(), Optional.empty());
    }
}
