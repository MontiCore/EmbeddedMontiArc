/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.middleware.mqtt;

import de.monticore.lang.embeddedmontiarc.AbstractTaggingResolverTest;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
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

        //mqttInQ
        EMAPortSymbol mqttInQ = component.getPortInstance("mqttInQ").orElse(null);
        assertNotNull(mqttInQ);

        Collection<TagSymbol> tags = symtab.getTags(mqttInQ, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        MqttConnectionSymbol tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoQ");

        //mqttInN
        EMAPortSymbol mqttInN = component.getPortInstance("mqttInN").orElse(null);
        assertNotNull(mqttInN);

        tags = symtab.getTags(mqttInN, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoN");

        //mqttInZ
        EMAPortSymbol mqttInZ = component.getPortInstance("mqttInZ").orElse(null);
        assertNotNull(mqttInZ);

        tags = symtab.getTags(mqttInZ, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoZ");

        //mqttInB
        EMAPortSymbol mqttInB = component.getPortInstance("mqttInB").orElse(null);
        assertNotNull(mqttInB);

        tags = symtab.getTags(mqttInB, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoB");

        //mqttOutQ
        EMAPortSymbol mqttOutQ = component.getPortInstance("mqttOutQ").orElse(null);
        assertNotNull(mqttOutQ);

        tags = symtab.getTags(mqttOutQ, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoQ");

        //mqttOutN
        EMAPortSymbol mqttOutN = component.getPortInstance("mqttOutN").orElse(null);
        assertNotNull(mqttOutN);

        tags = symtab.getTags(mqttOutN, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoN");

        //mqttOutZ
        EMAPortSymbol mqttOutZ = component.getPortInstance("mqttOutZ").orElse(null);
        assertNotNull(mqttOutZ);

        tags = symtab.getTags(mqttOutZ, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoZ");

        //mqttOutB
        EMAPortSymbol mqttOutB = component.getPortInstance("mqttOutB").orElse(null);
        assertNotNull(mqttOutB);

        tags = symtab.getTags(mqttOutB, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName().get(), "/echoB");

        //emptyTagIn
        EMAPortSymbol emptyTagIn = component.getPortInstance("emptyTagIn").orElse(null);
        assertNotNull(emptyTagIn);

        tags = symtab.getTags(emptyTagIn, MqttConnectionSymbol.KIND);
        assertTrue(tags.size() == 1);

        tag = (MqttConnectionSymbol) tags.iterator().next();
        assertEquals(tag.getTopicName(), Optional.empty());

    }
}
