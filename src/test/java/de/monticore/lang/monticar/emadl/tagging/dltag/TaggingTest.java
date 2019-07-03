/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.emadl.tagging.dltag;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;

import de.monticore.lang.monticar.emadl.AbstractTaggingResolverTest;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.lang.monticar.emadl.tagging.dltag.DataPathSymbol;

import org.junit.Test;

import java.util.ArrayList;
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

        DataPathSymbol tag = (DataPathSymbol) tags.iterator().next();
        assertEquals(tag.getPath(), "data");
        assertEquals(tag.getType(), "random");
    }

    @Test
    public void instanceTaggingParent() {
        TaggingResolver tagging = createSymTabandTaggingResolver("src/test/resources/");
        EMAComponentSymbol symbol = tagging.<EMAComponentSymbol>
                resolve("tagging.Parent", EMAComponentSymbol.KIND).get();

        EMAComponentInstanceSymbol mainInstance = tagging.<EMAComponentInstanceSymbol>
                resolve("tagging.parent", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol a1 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                resolve("a1", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol a2 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                resolve("a2", EMAComponentInstanceSymbol.KIND).get();

        assertEquals("tagging.parent.a1", a1.getFullName());
        assertEquals("tagging.parent.a2", a2.getFullName());

        /*ArrayList<EMAComponentInstanceSymbol> subInstances = new ArrayList<EMAComponentInstanceSymbol>(mainInstance.getSubComponents());

        EMAComponentInstanceSymbol a1 = subInstances.get(0);
        EMAComponentInstanceSymbol a2 = subInstances.get(1);
        System.out.println(a1);
        System.out.println(a2);

        Collection<TagSymbol> tagsA1 = tagging.getTags(a1, DataPathSymbol.KIND);
        assertEquals(1, tagsA1.size());

        DataPathSymbol tagA1 = (DataPathSymbol) tagsA1.iterator().next();
        assertEquals(tagA1.getPath(), "src/test/models/");
        assertEquals(tagA1.getType(), "LMDB");

        Collection<TagSymbol> tagsA2 = tagging.getTags(a2, DataPathSymbol.KIND);
        assertEquals(1, tagsA2.size());

        DataPathSymbol tagA2 = (DataPathSymbol) tagsA2.iterator().next();
        assertEquals(tagA2.getPath(), "lisjef");
        assertEquals(tagA2.getType(), "r34");*/

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
