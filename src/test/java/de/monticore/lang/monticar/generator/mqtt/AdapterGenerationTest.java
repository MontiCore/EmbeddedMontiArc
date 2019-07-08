package de.monticore.lang.monticar.generator.mqtt;

import static org.junit.Assert.assertNotNull;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class AdapterGenerationTest extends AbstractSymtabTest {

	@Test
    public void testAdapter() throws IOException {
        
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        
        // Ros schema is used for now
        RosToEmamTagSchema.registerTagTypes(taggingResolver);
        
        // Create component instance and run the generator
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.compA", EMAComponentInstanceSymbol.KIND).orElse(null);
        
        assertNotNull(componentInstanceSymbol);
        
        GeneratorMqtt generatorMqtt = new GeneratorMqtt();
        
        // Connect component's ports to topics
        componentInstanceSymbol.getPortInstance("portA").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clock"));
    }
}
