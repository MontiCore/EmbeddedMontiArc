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
import de.monticore.lang.tagging._symboltable.TaggingResolver;

import java.util.*;

public class CNNArch2Tensorflow extends CNNArchGenerator {

    private CMakeConfig cMakeConfig = new CMakeConfig("");

    public CNNArch2Tensorflow() {
        architectureSupportChecker = new CNNArch2TensorflowArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2TensorflowLayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ArchitectureSymbol architecture){
        if(architecture != null && architecture.getFullName() != null) {
            cMakeConfig.getCMakeListsViewModel().setCompName(architecture.getFullName().replace('.', '_').replace('[', '_').replace(']', '_'));
        }
        List<FileContent> fileContents = compileFileContentMap(architecture);
        
        return fileContents;
    }
    
    private List<FileContent> compilePythonFiles(CNNArch2TensorflowTemplateController controller, ArchitectureSymbol architecture) {
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp;
     
        temp = controller.process("CNNCreator", Target.PYTHON);
        fileContents.add(temp);
        
        if (architecture.getDataPath() != null) {
            temp = controller.process("CNNDataLoader", Target.PYTHON);
            fileContents.add(temp);
        }
        
        return fileContents;
    }
    
    private List<FileContent> compileCppFiles(CNNArch2TensorflowTemplateController controller) {
        // Add cmake dependencies when they are needed
        addCMakeDependencies();
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp;

        temp = controller.process("CNNPredictor", Target.CPP);
        fileContents.add(temp);

        temp = controller.process("execute", Target.CPP);
        fileContents.add(new FileContent(temp.getFileContent(), temp.getFileName().replace(".h", "")));

        return fileContents;
    }
    
    private List<FileContent> compileFileContentMap(ArchitectureSymbol architecture) {
        TemplateConfiguration templateConfiguration = new TensorflowTemplateConfiguration();

        List<FileContent> fileContents = new ArrayList<>();
        CNNArch2TensorflowTemplateController archTc = new CNNArch2TensorflowTemplateController(
                architecture, templateConfiguration);

        fileContents.addAll(compilePythonFiles(archTc, architecture));
        fileContents.addAll(compileCppFiles(archTc));

        return fileContents;
    }

    public List<FileContent> generateCMakeContent(String rootModelName) {
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName =  rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        cMakeConfig.getCMakeListsViewModel().setCompName(rootModelName);

        addCMakeDependencies();

        List<FileContent> fileContents = new ArrayList<>();
        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContents.add(fileContent);
        }
        return fileContents;
    }

    private void addCMakeDependencies() {
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addCMakeCommand("find_package(TensorflowCC REQUIRED)");
        cMakeConfig.addCmakeLibraryLinkage("TensorflowCC::Shared");
    }

    @Override
    public CMakeConfig getCmakeConfig() {
        return cMakeConfig;
    }
}
