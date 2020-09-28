/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.Target;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.util.*;

public class CNNArch2Gluon extends CNNArchGenerator {

    public CNNArch2Gluon() {
        architectureSupportChecker = new CNNArch2GluonArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2GluonLayerSupportChecker();
    }

    @Override
    public CMakeConfig getCmakeConfig() {
        return null;
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    @Override
    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ArchitectureSymbol architecture){
        List<FileContent> fileContents = compileFileContents(architecture);
        return fileContents;
    }

    public List<FileContent> generateStringsAllowMultipleIO(ArchitectureSymbol architecture, Boolean pythonFilesOnly) {
        List<FileContent> fileContents;
        if (pythonFilesOnly) {
            fileContents = compilePythonFilesOnlyContents(architecture);
        } else {
            fileContents = compileFileContents(architecture);
        }
        return fileContents;
    }

    private List<FileContent> compilePythonFiles(CNNArch2GluonTemplateController controller, ArchitectureSymbol architecture) {
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp;

        temp = controller.process("CNNNet", Target.PYTHON);
        fileContents.add(temp);

        if (architecture.getDataPath() != null) {
            temp = controller.process("CNNDataLoader", Target.PYTHON);
            fileContents.add(temp);
        }

        temp = controller.process("CNNCreator", Target.PYTHON);
        fileContents.add(temp);

        return fileContents;
    }

    private List<FileContent> compileCppFiles(CNNArch2GluonTemplateController controller) {
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp;

        temp = controller.process("CNNPredictor", Target.CPP);
        fileContents.add(temp);

        temp = controller.process("CNNSupervisedTrainer", Target.PYTHON);
        fileContents.add(temp);

        temp = controller.process("CNNGanTrainer", Target.PYTHON);
        fileContents.add(temp);

        temp = controller.process("execute", Target.CPP);
        temp = new FileContent(temp.getFileContent(), temp.getFileName().replace(".h",""));
        fileContents.add(temp);

        temp = controller.process("CNNModelLoader", Target.CPP);
        fileContents.add(new FileContent(temp.getFileContent(), "CNNModelLoader.h"));

        return fileContents;
    }


    private List<FileContent> compileFileContents(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();

        architecture.processForEpisodicReplayMemory();

        List<FileContent> fileContents = new ArrayList<>();
        CNNArch2GluonTemplateController archTc = new CNNArch2GluonTemplateController(
                architecture, templateConfiguration);

        fileContents.addAll(compilePythonFiles(archTc, architecture));
        fileContents.addAll(compileCppFiles(archTc));

        return fileContents;
    }

    private List<FileContent> compilePythonFilesOnlyContents(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();
        CNNArch2GluonTemplateController archTc = new CNNArch2GluonTemplateController(
                architecture, templateConfiguration);
        return compilePythonFiles(archTc, architecture);
    }

    public List<FileContent> generateCMakeContent(String rootModelName) {
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName =  rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        CMakeConfig cMakeConfig = new CMakeConfig(rootModelName);
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addCmakeLibraryLinkage("mxnet");

        List<FileContent> fileContents = new ArrayList<>();
        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContents.add(fileContent);
        }
        return fileContents;
    }
}
