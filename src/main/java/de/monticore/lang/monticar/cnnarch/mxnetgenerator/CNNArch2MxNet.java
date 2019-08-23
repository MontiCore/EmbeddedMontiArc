/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.ArchitectureSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchSymbolCompiler;
import de.monticore.lang.monticar.cnnarch.generator.DataPathConfigParser;
import de.monticore.lang.monticar.cnnarch.generator.LayerSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.Target;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.symboltable.Scope;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class CNNArch2MxNet extends CNNArchGenerator {

    public CNNArch2MxNet() {
        architectureSupportChecker = new CNNArch2MxNetArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2MxNetLayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public Map<String, String> generateStrings(ArchitectureSymbol architecture){
        Map<String, String> fileContentMap = new HashMap<>();
        CNNArch2MxNetTemplateController archTc = new CNNArch2MxNetTemplateController(architecture);
        Map.Entry<String, String> temp;

        temp = archTc.process("CNNPredictor", Target.CPP);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = archTc.process("CNNCreator", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = archTc.process("execute", Target.CPP);
        fileContentMap.put(temp.getKey().replace(".h", ""), temp.getValue());

        temp = archTc.process("CNNBufferFile", Target.CPP);
        fileContentMap.put("CNNBufferFile.h", temp.getValue());

        return fileContentMap;
    }

    public Map<String, String> generateCMakeContent(String rootModelName) {
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName = rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

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
