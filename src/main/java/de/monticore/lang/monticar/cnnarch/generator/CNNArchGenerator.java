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
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchLanguage;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

public abstract class CNNArchGenerator {

    protected ArchitectureSupportChecker architectureSupportChecker;
    protected LayerSupportChecker layerSupportChecker;

    private String generationTargetPath;
    private String modelsDirPath;

    protected CNNArchGenerator() {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
    }

    public static void quitGeneration(){
        Log.error("Code generation is aborted");
        System.exit(1);
    }

    public boolean isCMakeRequired() {
        return true;
    }

    public String getGenerationTargetPath(){
        if (generationTargetPath.charAt(generationTargetPath.length() - 1) != '/') {
            this.generationTargetPath = generationTargetPath + "/";
        }
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath){
        this.generationTargetPath = generationTargetPath;
    }

    protected String getModelsDirPath() {
        return this.modelsDirPath;
    }

    public void generate(Path modelsDirPath, String rootModelName){
        this.modelsDirPath = modelsDirPath.toString();
        final ModelPath mp = new ModelPath(modelsDirPath);
        GlobalScope scope = new GlobalScope(mp, new CNNArchLanguage());
        generate(scope, rootModelName);
    }

    // TODO: Rewrite so that CNNArchSymbolCompiler is used in EMADL2CPP instead of this method
    public boolean check(ArchitectureSymbol architecture) {
        return architectureSupportChecker.check(architecture) && layerSupportChecker.check(architecture);
    }

    public void generate(Scope scope, String rootModelName){
        CNNArchSymbolCompiler symbolCompiler = new CNNArchSymbolCompiler(architectureSupportChecker, layerSupportChecker);
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
    public abstract Map<String, String> generateStrings(ArchitectureSymbol architecture);

    //check cocos with CNNArchCocos.checkAll(architecture) before calling this method.
    public void generateFiles(ArchitectureSymbol architecture) throws IOException{
        Map<String, String> fileContentMap = generateStrings(architecture);
        generateFromFilecontentsMap(fileContentMap);
    }

    public void generateFromFilecontentsMap(Map<String, String> fileContentMap) throws IOException {
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(getGenerationTargetPath());
        for (String fileName : fileContentMap.keySet()){
            genCPP.generateFile(new FileContent(fileContentMap.get(fileName), fileName));
        }
    }

    public void generateCMake(String rootModelName){
        Map<String, String> fileContentMap = generateCMakeContent(rootModelName);
        try {
            generateFromFilecontentsMap(fileContentMap);
        } catch (IOException e) {
            Log.error("CMake file could not be generated" + e.getMessage());
        }
    }

    public abstract Map<String, String> generateCMakeContent(String rootModelName);
}
