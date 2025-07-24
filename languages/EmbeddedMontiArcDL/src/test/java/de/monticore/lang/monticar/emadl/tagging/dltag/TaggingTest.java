/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.tagging.dltag;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import de.monticore.lang.monticar.emadl.AbstractTaggingResolverTest;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathSymbol;

import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.assertEquals;

public class TaggingTest extends AbstractTaggingResolverTest {

    @Test
    public void basicTaggingAlexNet() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>resolve("tagging.Alexnet", EMAComponentSymbol.KIND)
                .orElse(null);

        Collection<TagSymbol> tags = tagging.getTags(symbol, DataPathSymbol.KIND);
        assertEquals(1, tags.size());

        DataPathSymbol tagDataPath = (DataPathSymbol) tags.iterator().next();
        assertEquals(tagDataPath.getPath(), "data");
        assertEquals(tagDataPath.getType(), "random");
    }

    @Test
    public void instanceTaggingParent() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources/");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>
                resolve("tagging.Parent", EMAComponentSymbol.KIND).get();

        Collection<TagSymbol> tagsA1 = tagging.getTags(symbol.getSpannedScope().getLocalSymbols().get("a1").iterator().next(), DataPathSymbol.KIND);
        assertEquals(1, tagsA1.size());

        DataPathSymbol tagA1 = (DataPathSymbol) tagsA1.iterator().next();
        assertEquals(tagA1.getPath(), "src/test/models/");
        assertEquals(tagA1.getType(), "LMDB");


        Collection<TagSymbol> tagsA2 = tagging.getTags(symbol.getSpannedScope().getLocalSymbols().get("a2").iterator().next(), DataPathSymbol.KIND);
        assertEquals(1, tagsA2.size());

        DataPathSymbol tagA2 = (DataPathSymbol) tagsA2.iterator().next();
        assertEquals(tagA2.getPath(), "lisjef");
        assertEquals(tagA2.getType(), "r34");
    }
}
