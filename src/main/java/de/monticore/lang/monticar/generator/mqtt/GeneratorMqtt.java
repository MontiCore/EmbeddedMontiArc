/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
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

	private String generationTargetPath;

	public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath.endsWith("/") ? generationTargetPath : generationTargetPath + "/";
    }

	public List<File> generateMqttAdapter(EMAComponentInstanceSymbol component)
    {
		List<File> files = new ArrayList<>();
		List<String> contents = new ArrayList<String>();
		List<FileWriter> frs = new ArrayList<FileWriter>();

		// Create and fill model
		MqttAdapterModel model = new MqttAdapterModel(component.getFullName());

		model.addPorts(component.getPortInstanceList());

		component.getPortArrays().forEach(port -> {
		    port.setMiddlewareSymbol(new MqttConnectionSymbol(port.getFullName()));
        });

		//Generate files and write to project
		contents.add(MqttTemplates.generateMqttAdapterH(model));
		files.add(new File(generationTargetPath+"MqttAdapter_"+model.getEscapedCompName()+".h"));
		contents.add(MqttTemplates.generateMqttAdapterCPP(model));
		files.add(new File(generationTargetPath+"MqttAdapter_"+model.getEscapedCompName()+".cpp"));
		contents.add(MqttTemplates.generateMqttCallbackQH(model));
		files.add(new File(generationTargetPath+"CallbackQ.hpp"));
		contents.add(MqttTemplates.generateMqttCallbackQCPP(model));
		files.add(new File(generationTargetPath+"CallbackQ.cpp"));
		contents.add(MqttTemplates.generateMqttCallbackNH(model));
		files.add(new File(generationTargetPath+"CallbackN.hpp"));
		contents.add(MqttTemplates.generateMqttCallbackNCPP(model));
		files.add(new File(generationTargetPath+"CallbackN.cpp"));
		contents.add(MqttTemplates.generateMqttCallbackZH(model));
		files.add(new File(generationTargetPath+"CallbackZ.hpp"));
		contents.add(MqttTemplates.generateMqttCallbackZCPP(model));
		files.add(new File(generationTargetPath+"CallbackZ.cpp"));
		contents.add(MqttTemplates.generateMqttCallbackBH(model));
		files.add(new File(generationTargetPath+"CallbackB.hpp"));
		contents.add(MqttTemplates.generateMqttCallbackBCPP(model));
		files.add(new File(generationTargetPath+"CallbackB.cpp"));

		//If file directory does not exist, create it so files can be created
		File directory = new File(generationTargetPath);
		directory.mkdirs();


        try {
        	int counter = 0;
        	for (File file : files)
        	{
        		frs.add(new FileWriter(file));
        		frs.get(counter).write(contents.get(counter));
        		counter++;
        	}
        } catch (IOException e) {
            e.printStackTrace();
        }finally{
            //Close resources
            try {
                for (FileWriter fr : frs)
                	fr.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        files.add(generateCMake(component));
        files.add(generateFindMqtt(component));

    	return files;
    }

	public File generateCMake(EMAComponentInstanceSymbol component)
    {

		// Create and fill model
		MqttAdapterModel model = new MqttAdapterModel(component.getFullName());

		//Generate files and write to project
		String content = MqttTemplates.generateMqttCMakeLists(model);

		File file = new File(generationTargetPath+"CMakeLists.txt");

        createFile(file, content);

    	return file;
    }

	public File generateFindMqtt(EMAComponentInstanceSymbol component)
	{

		// Create and fill model
		MqttAdapterModel model = new MqttAdapterModel(component.getFullName());

		//Generate files and write to project
		String content = MqttTemplates.generateMqttFindMqtt(model);

		File file = new File(generationTargetPath+"FindMQTT.cmake");

        createFile(file, content);

    	return file;
    }

	public List<File> generatePrettyPrint(EMAComponentInstanceSymbol component)
	{
		List<File> files = new ArrayList<>();

		// Get info about the ports from the component
		Collection<EMAPortInstanceSymbol> ports = component.getPortInstanceList();

		// Create and fill model
		MqttAdapterModel model = new MqttAdapterModel(component.getFullName());

		model.addPortsDesc(ports);

		//Generate files and write to project
		String content = MqttTemplates.generatePrettyPrint(model);

		File file = new File(generationTargetPath+"ports.txt");
		files.add(file);

        createFile(file, content);

    	return files;

	}

	private void createFile(File file, String content) {
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
    }

}
