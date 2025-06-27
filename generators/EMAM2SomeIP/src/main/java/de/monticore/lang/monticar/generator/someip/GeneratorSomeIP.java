/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.someip;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.monticar.generator.someip.template.SomeIPAdapterModel;
import de.monticore.lang.monticar.generator.someip.template.SomeIPTemplates;
import de.monticore.lang.monticar.generator.someip.helper.FilesHelper;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class GeneratorSomeIP {

	private String generationTargetPath;

	public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath.endsWith("/") ? generationTargetPath : generationTargetPath + "/";
    }


	public List<File> generateSomeIPAdapter(EMAComponentInstanceSymbol component) {
		List<FilesHelper> generateList = new ArrayList<>();
		List<File> files = new ArrayList<>();

		// Create and fill model
		SomeIPAdapterModel model = new SomeIPAdapterModel(component.getFullName());

		model.addPorts(component.getPortInstanceList());

		//Generate files and write to project

		generateList.add(new FilesHelper(new File(generationTargetPath+"SomeIPAdapter_" + model.getEscapedCompName() + ".h"), SomeIPTemplates.generateSomeIPAdapterH(model)));
		generateList.add(new FilesHelper(new File(generationTargetPath+"SomeIPAdapter_" + model.getEscapedCompName() + ".cpp"), SomeIPTemplates.generateSomeIPAdapterCPP(model)));


		//If file directory does not exist, create it so files can be created
		File directory = new File(generationTargetPath);
		directory.mkdirs();

		for (FilesHelper filesHelper : generateList) {
			createFile(filesHelper.getFile(), filesHelper.getContent());
			files.add(filesHelper.getFile());
		}

    	files.add(generateCMake(component));

    	return files;

    }

    public File generateCMake(EMAComponentInstanceSymbol component) {

		// Create and fill model
		SomeIPAdapterModel model = new SomeIPAdapterModel(component.getFullName());

		//Generate files and write to project
		String content = SomeIPTemplates.generateSomeIPCMakeLists(model);

		File file = new File(generationTargetPath + "CMakeLists.txt");

        FileWriter fr = null;
        try {
            fr = new FileWriter(file);
            fr.write(content);
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            //Close resources
            try {
                fr.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    	return file;
    }

    public List<File> generatePrettyPrint(EMAComponentInstanceSymbol component)	{
		List<File> files = new ArrayList<>();

		// Get info about the ports from the component
		Collection<EMAPortInstanceSymbol> ports = component.getPortInstanceList();

		// Create and fill model
		SomeIPAdapterModel model = new SomeIPAdapterModel(component.getFullName());

		model.addPortsDesc(ports);

		//Generate files and write to project
		String content = SomeIPTemplates.generatePrettyPrint(model);

		File file = new File(generationTargetPath + "ports.txt");
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
