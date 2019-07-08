package de.monticore.lang.monticar.generator.mqtt;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.mqtt.template.MqttAdapterModel;
import de.monticore.lang.monticar.generator.mqtt.template.MqttTemplates;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class GeneratorMqtt 
{

	List<File> generateMqttAdapter(EMAComponentInstanceSymbol component) 
    {
		List<File> files = new ArrayList<>();

		// Get info about the ports from the component
		Collection<EMAPortInstanceSymbol> ports = component.getPortInstanceList();
		
		// Create and fill model
		MqttAdapterModel model = new MqttAdapterModel(component.getFullName());
		
		model.addPorts(ports);
		
		//Generate files and write to project
		String content = MqttTemplates.generateMqttAdapter(model);
		
		File file = new File("./target/generated-sources/ports.txt");
		files.add(file);
		
        FileWriter fr = null;
        try {
            fr = new FileWriter(file);
            fr.write(content);
        } catch (IOException e) {
            e.printStackTrace();
        }finally{
            //Close resources
            try {
                fr.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
		
    	return files;
    }
	
	File generateCMake(EMAComponentInstanceSymbol component) 
    {
		
		// Create and fill model
		MqttAdapterModel model = new MqttAdapterModel(component.getFullName());
		
		//Generate files and write to project
		String content = MqttTemplates.generateMqttCMakeLists(model);
		
		File file = new File("./target/generated-sources/CMakeLists.txt");
		
        FileWriter fr = null;
        try {
            fr = new FileWriter(file);
            fr.write(content);
        } catch (IOException e) {
            e.printStackTrace();
        }finally{
            //Close resources
            try {
                fr.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
		
    	return file;
    }

}
