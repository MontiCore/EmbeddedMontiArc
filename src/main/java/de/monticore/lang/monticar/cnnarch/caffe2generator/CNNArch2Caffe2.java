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

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch.CNNArchGenerator;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchLanguage;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class CNNArch2Caffe2 implements CNNArchGenerator{

    private String generationTargetPath;

    private void supportCheck(ArchitectureSymbol architecture){
        LayerSupportChecker layerChecker = new LayerSupportChecker();
        for (ArchitectureElementSymbol element : ((CompositeElementSymbol)architecture.getBody()).getElements()){
            if (!layerChecker.isSupported(element.toString())) {
                Log.error("Unsupported layer " + "'" + element.getName() + "'" + " for the backend CAFFE2. Code generation aborted.");
                System.exit(1);
            }
        }
    }

    public CNNArch2Caffe2() {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
    }

    @Override
    public boolean isCMakeRequired() {
        return true;
    }

    public String getGenerationTargetPath() {
        if (generationTargetPath.charAt(generationTargetPath.length() - 1) != '/') {
            this.generationTargetPath = generationTargetPath + "/";
        }
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    public void generate(Path modelsDirPath, String rootModelName){
        final ModelPath mp = new ModelPath(modelsDirPath);
        GlobalScope scope = new GlobalScope(mp, new CNNArchLanguage());
        generate(scope, rootModelName);
    }

    public void generate(Scope scope, String rootModelName){
        Optional<CNNArchCompilationUnitSymbol> compilationUnit = scope.resolve(rootModelName, CNNArchCompilationUnitSymbol.KIND);
        if (!compilationUnit.isPresent()){
            Log.error("could not resolve architecture " + rootModelName);
            System.exit(1);
        }

        CNNArchCocos.checkAll(compilationUnit.get());
        supportCheck(compilationUnit.get().getArchitecture());

        try{
            generateFiles(compilationUnit.get().getArchitecture());
        }
        catch (IOException e){
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

        checkValidGeneration(architecture);

        return fileContentMap;
    }

    private void checkValidGeneration(ArchitectureSymbol architecture){
        if (architecture.getInputs().size() > 1){
            Log.error("This cnn architecture has multiple inputs, " +
                            "which is currently not supported by the caffe2generator. "
                    , architecture.getSourcePosition());
        }
        if (architecture.getOutputs().size() > 1){
            Log.error("This cnn architecture has multiple outputs, " +
                            "which is currently not supported by the caffe2generator. "
                    , architecture.getSourcePosition());
        }
        if (architecture.getOutputs().get(0).getDefinition().getType().getWidth() != 1 ||
                architecture.getOutputs().get(0).getDefinition().getType().getHeight() != 1){
            Log.error("This cnn architecture has a multi-dimensional output, " +
                            "which is currently not supported by the caffe2generator."
                    , architecture.getSourcePosition());
        }
    }

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public void generateFiles(ArchitectureSymbol architecture) throws IOException{
        Map<String, String> fileContentMap = generateStrings(architecture);
        generateFromFilecontentsMap(fileContentMap);
    }

    public void generateCMake(String rootModelName) {
        Map<String, String> fileContentMap = generateCMakeContent(rootModelName);
        try {
            generateFromFilecontentsMap(fileContentMap);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void generateFromFilecontentsMap(Map<String, String> fileContentMap) throws IOException {
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
        cMakeConfig.addModuleDependency(new CMakeFindModule("Caffe2", true));
        cMakeConfig.addCMakeCommand("set(LIBS ${LIBS} -lprotobuf -lglog -lgflags)");
        cMakeConfig.addCMakeCommand("find_package(CUDA)" + "\n"
                                        + "set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})" + "\n"
                                        + "set(LIBS ${LIBS} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY})" + "\n"); //Needed since CUDA cannot be found correctly (including CUDA_curand_LIBRARY) and as optional using CMakeFindModule


        cMakeConfig.addCMakeCommand("if(CUDA_FOUND)" + "\n" + "  set(LIBS ${LIBS} caffe2 caffe2_gpu)"
                                        + "\n" + "else()" + "\n" + "  set(LIBS ${LIBS} caffe2)" + "\n" + "endif()");

        Map<String,String> fileContentMap = new HashMap<>();
        for (FileContent fileContent : cMakeConfig.generateCMakeFiles()){
            fileContentMap.put(fileContent.getFileName(), fileContent.getFileContent());
        }
        return fileContentMap;
    }
}
