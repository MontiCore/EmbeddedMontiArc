/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.mqtt.MqttConnectionSymbol;
import de.monticore.lang.monticar.generator.mqtt.template.MqttAdapterModel;
import de.monticore.lang.monticar.generator.mqtt.template.MqttTemplates;
import de.monticore.lang.monticar.generator.mqtt.helper.FilesHelper;

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

	public List<File> generateMqttAdapter(EMAComponentInstanceSymbol component) {
		List<FilesHelper> generateList = new ArrayList<>();
		List<File> res = new ArrayList<>();

		// Create and fill model
		MqttAdapterModel model = new MqttAdapterModel(component.getFullName());

		model.addPorts(component.getPortInstanceList());

		//Generate files and write to project
		generateList.add(new FilesHelper(new File(generationTargetPath+"MqttAdapter_"+model.getEscapedCompName()+".h"), MqttTemplates.generateMqttAdapterH(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"MqttAdapter_"+model.getEscapedCompName()+".cpp"), MqttTemplates.generateMqttAdapterCPP(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackQ.hpp"), MqttTemplates.generateMqttCallbackQH(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackQ.cpp"), MqttTemplates.generateMqttCallbackQCPP(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackN.hpp"), MqttTemplates.generateMqttCallbackNH(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackN.cpp"), MqttTemplates.generateMqttCallbackNCPP(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackZ.hpp"), MqttTemplates.generateMqttCallbackZH(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackZ.cpp"), MqttTemplates.generateMqttCallbackZCPP(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackB.hpp"), MqttTemplates.generateMqttCallbackBH(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"CallbackB.cpp"), MqttTemplates.generateMqttCallbackBCPP(model)));

		//If file directory does not exist, create it so files can be created
		File directory = new File(generationTargetPath);
		directory.mkdirs();

    for (FilesHelper filesHelper : generateList) {
				createFile(filesHelper.getFile(), filesHelper.getContent());
				res.add(filesHelper.getFile());
		}
    res.add(generateCMake(component));
    res.add(generateFindMqtt(component));

    return res;
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

		File directory = new File(generationTargetPath+"modules/");
		directory.mkdirs();

		File file = new File(generationTargetPath+"modules/FindMQTT.cmake");

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
