/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.Target;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class CNNArch2Gluon extends CNNArchGenerator {

    public CNNArch2Gluon() {
        architectureSupportChecker = new CNNArch2GluonArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2GluonLayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    @Override
    public Map<String, String> generateStrings(ArchitectureSymbol architecture){
        Map<String, String> fileContentMap = compileFileContentMap(architecture);
        return fileContentMap;
    }

    public Map<String, String> generateStringsAllowMultipleIO(ArchitectureSymbol architecture, Boolean pythonFilesOnly) {
        Map<String, String> fileContentMap;
        if (pythonFilesOnly) {
            fileContentMap = compilePythonFilesOnlyContentMap(architecture);
        } else {
            fileContentMap = compileFileContentMap(architecture);
        }
        return fileContentMap;
    }

    private Map<String, String> compilePythonFiles(CNNArch2GluonTemplateController controller, ArchitectureSymbol architecture) {
        Map<String, String> fileContentMap = new HashMap<>();
        Map.Entry<String, String> temp;

        temp = controller.process("CNNNet", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        if (architecture.getDataPath() != null) {
            temp = controller.process("CNNDataLoader", Target.PYTHON);
            fileContentMap.put(temp.getKey(), temp.getValue());
        }

        temp = controller.process("CNNCreator", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        return fileContentMap;
    }

    private Map<String, String> compileCppFiles(CNNArch2GluonTemplateController controller) {
        Map<String, String> fileContentMap = new HashMap<>();
        Map.Entry<String, String> temp;

        temp = controller.process("CNNPredictor", Target.CPP);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = controller.process("CNNSupervisedTrainer", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = controller.process("execute", Target.CPP);
        fileContentMap.put(temp.getKey().replace(".h", ""), temp.getValue());

        temp = controller.process("CNNBufferFile", Target.CPP);
        fileContentMap.put("CNNBufferFile.h", temp.getValue());

        return fileContentMap;
    }

    private Map<String, String> compileFileContentMap(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();

        Map<String, String> fileContentMap = new HashMap<>();
        CNNArch2GluonTemplateController archTc = new CNNArch2GluonTemplateController(
                architecture, templateConfiguration);

        fileContentMap.putAll(compilePythonFiles(archTc, architecture));
        fileContentMap.putAll(compileCppFiles(archTc));

        return fileContentMap;
    }

    private Map<String, String> compilePythonFilesOnlyContentMap(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();
        CNNArch2GluonTemplateController archTc = new CNNArch2GluonTemplateController(
                architecture, templateConfiguration);
        return compilePythonFiles(archTc, architecture);
    }

    public Map<String, String> generateCMakeContent(String rootModelName) {
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName =  rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        CMakeConfig cMakeConfig = new CMakeConfig(rootModelName);
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addCMakeCommand("set(LIBS ${LIBS} mxnet)");

        Map<String,String> fileContentMap = new HashMap<>();
        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContentMap.put(fileContent.getFileName(), fileContent.getFileContent());
        }
        return fileContentMap;
    }
}
