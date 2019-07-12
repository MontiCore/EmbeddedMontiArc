package de.monticore.lang.monticar.generator.someip;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
//import de.monticore.lang.embeddedmontiarc.tagging.middleware.someip.SomeIPConnectionSymbol;
import de.monticore.lang.monticar.generator.someip.template.SomeIPAdapterModel;
import de.monticore.lang.monticar.generator.someip.template.SomeIPTemplates;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

public class GeneratorSomeIP {

	List<File> generateSomeIPAdapter(EMAComponentInstanceSymbol component) {
		List<File> files = new ArrayList<>();
		List<String> contents = new ArrayList<String>();
		List<FileWriter> frs = new ArrayList<FileWriter>();

		// Create and fill model
		SomeIPAdapterModel model = new SomeIPAdapterModel(component.getFullName());

		model.addPorts(component.getPortInstanceList());

		//Generate files and write to project
		contents.add(SomeIPTemplates.generateSomeIPAdapterH(model));
		files.add(new File("./target/generated-sources/SomeIPAdapter_"+model.getEscapedCompName()+".h"));
		contents.add(SomeIPTemplates.generateSomeIPAdapterCPP(model));
		files.add(new File("./target/generated-sources/SomeIPAdapter_"+model.getEscapedCompName()+".cpp"));

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

    	return files;

    }

    File generateCMake(EMAComponentInstanceSymbol component) {

		// Create and fill model
		SomeIPAdapterModel model = new SomeIPAdapterModel(component.getFullName());

		//Generate files and write to project
		String content = SomeIPTemplates.generateSomeIPCMakeLists(model);

		File file = new File("./target/generated-sources/CMakeLists.txt");

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

    List<File> generatePrettyPrint(EMAComponentInstanceSymbol component)
	{
		List<File> files = new ArrayList<>();

		// Get info about the ports from the component
		Collection<EMAPortInstanceSymbol> ports = component.getPortInstanceList();

		// Create and fill model
		SomeIPAdapterModel model = new SomeIPAdapterModel(component.getFullName());

		model.addPortsDesc(ports);

		//Generate files and write to project
		String content = SomeIPTemplates.generatePrettyPrint(model);

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



}
