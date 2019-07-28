package de.monticore.lang.monticar.generator.middleware.impls;

import java.io.File;
import java.io.IOException;
import java.util.List;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.mqtt.GeneratorMqtt;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;

public class MqttCppGenImpl implements GeneratorImpl {
	
	private String generationTargetPath;
    private GeneratorMqtt generatorMqttCpp;

    public MqttCppGenImpl(){
    	generatorMqttCpp = new GeneratorMqtt();
    }

	@Override
	public List<File> generate(EMAComponentInstanceSymbol componentInstanceSymbol, TaggingResolver taggingResolver)
			throws IOException {
		generatorMqttCpp.setGenerationTargetPath(generationTargetPath);
		return generatorMqttCpp.generateMqttAdapter(componentInstanceSymbol);
	}

	@Override
	public void setGenerationTargetPath(String path) {
		this.generationTargetPath = path;
	}

	@Override
	public boolean willAccept(EMAComponentInstanceSymbol componentInstanceSymbol) {
		
		boolean result = componentInstanceSymbol.getPortInstanceList().stream().anyMatch(EMAPortInstanceSymbol::isRosPort); // Should be adapted to Mqtt !!!
        if(!result){
            Log.warn("GeneratorMqttCpp: No MQTT Ports found! Ignoring component " + componentInstanceSymbol.getName());
        }

        return result;
	}

}
