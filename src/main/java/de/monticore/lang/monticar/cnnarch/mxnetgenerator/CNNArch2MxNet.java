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
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.List;

public class CNNArch2MxNet extends CNNArchGenerator {

    private boolean isSupportedLayer(ArchitectureElementSymbol element, LayerSupportChecker layerChecker){
        List<ArchitectureElementSymbol> constructLayerElemList;

        if (element.getResolvedThis().get() instanceof CompositeElementSymbol) {
            constructLayerElemList = ((CompositeElementSymbol)element.getResolvedThis().get()).getElements();
            for (ArchitectureElementSymbol constructedLayerElement : constructLayerElemList) {
                if (!isSupportedLayer(constructedLayerElement, layerChecker)) {
                    return false;
                }
            }
        }
        if (!layerChecker.isSupported(element.toString())) {
            Log.error("Unsupported layer " + "'" + element.getName() + "'" + " for the backend MXNET.");
            return false;
        } else {
            return true;
        }
    }

    private boolean supportCheck(ArchitectureSymbol architecture){
        LayerSupportChecker layerChecker = new LayerSupportChecker();
        for (ArchitectureElementSymbol element : ((CompositeElementSymbol)architecture.getBody()).getElements()){
            if(!isSupportedLayer(element, layerChecker)) {
                return false;
            }
        }
        return true;
    }

    public CNNArch2MxNet() {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
    }

    public void generate(Scope scope, String rootModelName){
        Optional<CNNArchCompilationUnitSymbol> compilationUnit = scope.resolve(rootModelName, CNNArchCompilationUnitSymbol.KIND);
        if (!compilationUnit.isPresent()){
            Log.error("could not resolve architecture " + rootModelName);
            quitGeneration();
        }

        CNNArchCocos.checkAll(compilationUnit.get());
        if (!supportCheck(compilationUnit.get().getArchitecture())){
            quitGeneration();
        }

        try{
            generateFiles(compilationUnit.get().getArchitecture());
        } catch (IOException e){
            Log.error(e.toString());
        }
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

        temp = archTc.process("CNNBufferFile", Target.CPP);
        fileContentMap.put("CNNBufferFile.h", temp.getValue());

        checkValidGeneration(architecture);

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
