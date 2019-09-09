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
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.middleware.someip;

import de.monticore.lang.embeddedmontiarc.AbstractTaggingResolverTest;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
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
        EMAPortSymbol someIPIn = component.getPortInstance("someipIn").orElse(null);
        assertNotNull(someIPIn);

        Collection<TagSymbol> tags = symtab.getTags(someIPIn, SomeIPConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        SomeIPConnectionSymbol tag = (SomeIPConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getserviceID().get(), 1);
        assertEquals(tag.getinstanceID().get(), 2);
        assertEquals(tag.geteventgroupID().get(), 3);

        //someipOut
        EMAPortSymbol someIPOut = component.getPortInstance("someipOut").orElse(null);
        assertNotNull(someIPOut);

        tags = symtab.getTags(someIPOut, SomeIPConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (SomeIPConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getserviceID().get(), 1);
        assertEquals(tag.getinstanceID().get(), 2);
        assertEquals(tag.geteventgroupID().get(), 3);

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

    @Test
    public void testSomeIPConnectionParsingOptionalMsgField() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("middleware.someip.optionalMsgField", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        EMAPortSymbol in1 = component.getPortInstance("in1").orElse(null);
        EMAPortSymbol out1 = component.getPortInstance("out1").orElse(null);

        assertNotNull(in1);
        assertNotNull(out1);

        SomeIPConnectionSymbol tagIn1 = (SomeIPConnectionSymbol) symtab.getTags(in1, SomeIPConnectionSymbol.KIND).stream().findFirst().orElse(null);
        SomeIPConnectionSymbol tagOut1 = (SomeIPConnectionSymbol) symtab.getTags(out1, SomeIPConnectionSymbol.KIND).stream().findFirst().orElse(null);

        assertNotNull(tagIn1);
        assertNotNull(tagOut1);

        assertFalse(tagIn1.getserviceID().isPresent());
        assertTrue(tagOut1.getserviceID().isPresent());
        //assertTrue(tagOut1.getserviceID().get().equals("msgField1"));

    }
}
