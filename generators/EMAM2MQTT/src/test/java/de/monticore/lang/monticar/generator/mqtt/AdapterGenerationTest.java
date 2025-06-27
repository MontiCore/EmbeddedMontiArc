/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt;

import static org.junit.Assert.assertNotNull;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.junit.Test;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class AdapterGenerationTest extends AbstractSymtabTest {

	@Test
    public void testAdapter() throws IOException {

        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");

        MqttToEmamTagSchema.registerTagTypes(taggingResolver);

        // Create component instance and run the generator
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.compA", EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        GeneratorMqtt generatorMqtt = new GeneratorMqtt();

        generatorMqtt.setGenerationTargetPath("./target/generated-sources/");

        // Connect component's ports to topics
        componentInstanceSymbol.getPortInstance("portA").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clock"));
        componentInstanceSymbol.getPortInstance("portB").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clock"));
        componentInstanceSymbol.getPortInstance("portC").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clock"));
        componentInstanceSymbol.getPortInstance("portD").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clock"));
		componentInstanceSymbol.getPortInstance("portE").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clockN"));
		componentInstanceSymbol.getPortInstance("portF").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clockN"));
		componentInstanceSymbol.getPortInstance("portG").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clockZ"));
		componentInstanceSymbol.getPortInstance("portH").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clockZ"));
		componentInstanceSymbol.getPortInstance("portI").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clockB"));
		componentInstanceSymbol.getPortInstance("portJ").orElse(null).setMiddlewareSymbol(new MqttConnectionSymbol("/clockB"));

        List<File> files = generatorMqtt.generateMqttAdapter(componentInstanceSymbol);        	

        testFilesAreEqual(files, "echoAdapter/");
    }
}
