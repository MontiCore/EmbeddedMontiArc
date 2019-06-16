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
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.DataPathConfigParser;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class CNNArch2MxNet extends CNNArchGenerator {

    public CNNArch2MxNet() {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
    }

    public void generate(Scope scope, String rootModelName){
        CNNArchSymbolCompiler symbolCompiler = new CNNArchSymbolCompiler(new CNNArch2MxNetArchitectureSupportChecker(),
                                                                         new CNNArch2MxNetLayerSupportChecker());
        ArchitectureSymbol architectureSymbol = symbolCompiler.compileArchitectureSymbol(scope, rootModelName);

        try{
            String confPath = getModelsDirPath() + "/data_paths.txt";
            DataPathConfigParser newParserConfig = new DataPathConfigParser(confPath);
            String dataPath = newParserConfig.getDataPath(rootModelName);
            architectureSymbol.setDataPath(dataPath);
            architectureSymbol.setComponentName(rootModelName);
            generateFiles(architectureSymbol);
        } catch (IOException e){
            Log.error(e.toString());
        }
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

    public void generateFromFilecontentsMap(Map<String, String> fileContentMap) throws IOException {
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(getGenerationTargetPath());
        for (String fileName : fileContentMap.keySet()){
            genCPP.generateFile(new FileContent(fileContentMap.get(fileName), fileName));
        }
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
