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
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnnarch.generator.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch.generator.DataPathConfigParser;
import de.monticore.lang.monticar.cnnarch.generator.Target;

import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class CNNArch2Caffe2 extends CNNArchGenerator {

    public CNNArch2Caffe2() {
        architectureSupportChecker = new CNNArch2Caffe2ArchitectureSupportChecker();
        layerSupportChecker = new CNNArch2Caffe2LayerSupportChecker();
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public Map<String, String> generateStrings(ArchitectureSymbol architecture){
        Map<String, String> fileContentMap = new HashMap<>();
        CNNArchTemplateController archTc = new CNNArchTemplateController(architecture);
        Map.Entry<String, String> temp;

        temp = archTc.process("CNNPredictor", Target.CPP);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = archTc.process("CNNCreator", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());

        temp = archTc.process("execute", Target.CPP);
        fileContentMap.put(temp.getKey().replace(".h", ""), temp.getValue());

        return fileContentMap;
    }

    public Map<String, String> generateCMakeContent(String rootModelName) {
        // model name should start with a lower case letter. If it is a component, replace dot . by _
        rootModelName = rootModelName.replace('.', '_').replace('[', '_').replace(']', '_');
        rootModelName =  rootModelName.substring(0, 1).toLowerCase() + rootModelName.substring(1);

        CMakeConfig cMakeConfig = new CMakeConfig(rootModelName);
        cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        cMakeConfig.addModuleDependency(new CMakeFindModule("Caffe2", true));
        cMakeConfig.addCMakeCommand("set(LIBS ${LIBS} -lprotobuf -lglog -lgflags)");
        cMakeConfig.addCMakeCommand("find_package(CUDA)" + "\n");
                                        //Needed since CUDA cannot be found correctly (including CUDA_curand_LIBRARY)

        cMakeConfig.addCMakeCommand("if(CUDA_FOUND)" + "\n"
                                    + "  set(LIBS ${LIBS} caffe2 caffe2_gpu)" + "\n"
                                    + "  set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})" + "\n"
                                    + "  set(LIBS ${LIBS} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY})" + "\n"
                                    + "else()" + "\n" 
                                    + "  set(LIBS ${LIBS} caffe2)" + "\n" 
                                    + "endif()");

        Map<String,String> fileContentMap = new HashMap<>();
        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContentMap.put(fileContent.getFileName(), fileContent.getFileContent());
        }
        return fileContentMap;
    }
}
