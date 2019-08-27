/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt;

import static org.junit.Assert.assertNotNull;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttToEmamTagSchema;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

public class CMakeGenerationTest extends AbstractSymtabTest {
	
	@Test
    public void testCMake() throws IOException {
        
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        
        MqttToEmamTagSchema.registerTagTypes(taggingResolver);
        
        // Create component instance and run the generator
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("tests.a.compA", EMAComponentInstanceSymbol.KIND).orElse(null);
        
        assertNotNull(componentInstanceSymbol);
        
        GeneratorMqtt generatorMqtt = new GeneratorMqtt();
        
        generatorMqtt.setGenerationTargetPath("./target/generated-sources/");
        
        List<File> files = new ArrayList<>();
        
        files.add(generatorMqtt.generateCMake(componentInstanceSymbol));
        
        testFilesAreEqual(files, "echoCMake/");
    }

}
