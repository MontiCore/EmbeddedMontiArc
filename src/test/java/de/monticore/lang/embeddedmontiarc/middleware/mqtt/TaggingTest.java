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
package de.monticore.lang.embeddedmontiarc.middleware.mqtt;

import de.monticore.lang.embeddedmontiarc.AbstractTaggingResolverTest;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.tagging._symboltable.TagSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.Collection;
import java.util.Optional;

import static org.junit.Assert.*;

public class TaggingTest extends AbstractTaggingResolverTest {


    @Test
    public void testMqttConnectionParsing() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("middleware.mqtt.basicParsing", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        //mqttIn
        EMAPortSymbol mqttIn = component.getPortInstance("mqttIn").orElse(null);
        assertNotNull(mqttIn);

        Collection<TagSymbol> tags = symtab.getTags(mqttIn, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        MqttConnectionSymbol tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/clock");
        assertEquals(tag.getMsgField().get(), "clock.toSec()");
        
        //mqttOut
        EMAPortSymbol mqttOut = component.getPortInstance("mqttOut").orElse(null);
        assertNotNull(mqttOut);

        tags = symtab.getTags(mqttOut, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echo");
        assertEquals(tag.getMsgField().get(), "data");
        
        //emptyTagIn
        EMAPortSymbol emptyTagIn = component.getPortInstance("emptyTagIn").orElse(null);
        assertNotNull(emptyTagIn);

        tags = symtab.getTags(emptyTagIn, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);
        
        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName(), Optional.empty());
        assertEquals(tag.getMsgField(), Optional.empty());
        
    }

    @Test
    public void testMqttConnectionParsingOptionalMsgField() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol component = symtab.<EMAComponentInstanceSymbol>resolve("middleware.mqtt.optionalMsgField", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(component);

        EMAPortSymbol in1 = component.getPortInstance("in1").orElse(null);
        EMAPortSymbol out1 = component.getPortInstance("out1").orElse(null);

        assertNotNull(in1);
        assertNotNull(out1);

        MqttConnectionSymbol tagIn1 = (MqttConnectionSymbol) symtab.getTags(in1, MqttConnectionSymbol.KIND).stream().findFirst().orElse(null);
        MqttConnectionSymbol tagOut1 = (MqttConnectionSymbol) symtab.getTags(out1, MqttConnectionSymbol.KIND).stream().findFirst().orElse(null);

        assertNotNull(tagIn1);
        assertNotNull(tagOut1);

        assertFalse(tagIn1.getMsgField().isPresent());
        assertTrue(tagOut1.getMsgField().isPresent());
        assertTrue(tagOut1.getMsgField().get().equals("msgField1"));

    }
}
