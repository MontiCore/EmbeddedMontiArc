/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.Target;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;

import java.util.*;

public class CNNArch2Tensorflow extends CNNArchGenerator {

    public CNNArch2Tensorflow() {
        architectureSupportChecker = new CNNArch2TensorflowArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2TensorflowLayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public Map<String, String> generateStrings(ArchitectureSymbol architecture){
        Map<String, String> fileContentMap = compileFileContentMap(architecture);
        
        return fileContentMap;
    }
    
    private Map<String, String> compilePythonFiles(CNNArch2TensorflowTemplateController controller, ArchitectureSymbol architecture) {
        Map<String, String> fileContentMap = new HashMap<>();
        Map.Entry<String, String> temp;
     
        temp = controller.process("CNNCreator", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());
        
        if (architecture.getDataPath() != null) {
            temp = controller.process("CNNDataLoader", Target.PYTHON);
            fileContentMap.put(temp.getKey(), temp.getValue());
        }
        
        return fileContentMap;
    }
    
    private Map<String, String> compileCppFiles(CNNArch2TensorflowTemplateController controller) {
        Map<String, String> fileContentMap = new HashMap<>();
        Map.Entry<String, String> temp;

        temp = controller.process("CNNPredictor", Target.CPP);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = controller.process("execute", Target.CPP);
        fileContentMap.put(temp.getKey().replace(".h", ""), temp.getValue());

        return fileContentMap;
    }
    
    private Map<String, String> compileFileContentMap(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new TensorflowTemplateConfiguration();

        Map<String, String> fileContentMap = new HashMap<>();
        CNNArch2TensorflowTemplateController archTc = new CNNArch2TensorflowTemplateController(
                architecture, templateConfiguration);

        fileContentMap.putAll(compilePythonFiles(archTc, architecture));
        fileContentMap.putAll(compileCppFiles(archTc));

        return fileContentMap;
    }

    public Map<String, String> generateCMakeContent(String rootModelName) {
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName =  rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        CMakeConfig cMakeConfig = new CMakeConfig(rootModelName);
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));

        cMakeConfig.addCMakeCommand("find_package(TensorflowCC REQUIRED)");
        cMakeConfig.addCMakeCommand("set(LIBS ${LIBS} TensorflowCC::Shared)");

        Map<String,String> fileContentMap = new HashMap<>();
        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContentMap.put(fileContent.getFileName(), fileContent.getFileContent());
        }
        return fileContentMap;
    }
}
