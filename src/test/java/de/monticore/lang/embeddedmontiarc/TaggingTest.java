/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.adaptable.AdaptableSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.*;

public class TaggingTest extends AbstractTaggingResolverTest {

    @Test
    public void testAdaptableSymbolParsing(){
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");

        EMAComponentSymbol emaComponentSymbol = taggingResolver.<EMAComponentSymbol>resolve("testing.ConfigPortTag",EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(emaComponentSymbol);

        EMAPortSymbol in1 = emaComponentSymbol.getIncomingPort("in1").orElse(null);
        EMAPortSymbol in2 = emaComponentSymbol.getIncomingPort("in2").orElse(null);
        assertNotNull(in1);
        assertNotNull(in2);

        Collection<TagSymbol> tags1 = taggingResolver.getTags(in1, AdaptableSymbol.KIND);
        Collection<TagSymbol> tags2 = taggingResolver.getTags(in2, AdaptableSymbol.KIND);
        assertTrue(tags1.size() == 1);
        assertTrue(tags2.isEmpty());

        assertTrue(in1.isConfig());
        assertFalse(in2.isConfig());
    }

}
