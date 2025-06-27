/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.cocos;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathSymbol;
import de.monticore.lang.monticar.emadl._cocos.DataPathCocos;

import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;
import java.util.Collection;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import de.monticore.lang.monticar.emadl.AbstractTaggingResolverTest;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;

public class TaggingCoCoTest extends AbstractTaggingResolverTest {
    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testCoCosWithInvalidType() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.Alexnet", EMAComponentSymbol.KIND)
            .orElse(null);
        assertNotNull(symbol);

        Collection<TagSymbol> tags = tagging.getTags(symbol, DataPathSymbol.KIND);
        assertEquals(1, tags.size());
        DataPathSymbol tag = (DataPathSymbol) tags.iterator().next();
        checkValid(tag);

        assertFalse(Log.getFindings().isEmpty());
    }

    @Test
    public void testCoCosWithValidType() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.CorrectTypeNet", EMAComponentSymbol.KIND)
            .orElse(null);
        assertNotNull(symbol);

        Collection<TagSymbol> tags = tagging.getTags(symbol, DataPathSymbol.KIND);
        assertEquals(1, tags.size());
        DataPathSymbol tag = (DataPathSymbol) tags.iterator().next();
        checkValid(tag);

        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testCoCosForInstancesWithValidType() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources/");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.CorrectTypeInstance", EMAComponentSymbol.KIND)
            .orElse(null);
        assertNotNull(symbol);

        Collection<TagSymbol> tagsNet1 = tagging.getTags(symbol.getSpannedScope().getLocalSymbols().get("net1").iterator().next(), DataPathSymbol.KIND);
        assertEquals(1, tagsNet1.size());
        checkValid((DataPathSymbol) tagsNet1.iterator().next());

        Collection<TagSymbol> tagsNet2 = tagging.getTags(symbol.getSpannedScope().getLocalSymbols().get("net2").iterator().next(), DataPathSymbol.KIND);
        assertEquals(1, tagsNet2.size());
        checkValid((DataPathSymbol) tagsNet2.iterator().next());

        assertTrue(Log.getFindings().isEmpty());
    }

    protected static void checkValid(DataPathSymbol dataPathSymbol) {
        Log.getFindings().clear();
        DataPathCocos.check(dataPathSymbol);
    }
}
