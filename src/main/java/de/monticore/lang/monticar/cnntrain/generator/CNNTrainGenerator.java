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
package de.monticore.lang.monticar.cnntrain.generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class CNNTrainGenerator {

    private String generationTargetPath;

    public CNNTrainGenerator() {
        setGenerationTargetPath("./target/generated-sources-cnntrain/");
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
        GlobalScope scope = new GlobalScope(mp, new CNNTrainLanguage());
        generate(scope, rootModelName);
    }

    public void generate(Scope scope, String rootModelName){
        Optional<CNNTrainCompilationUnitSymbol> compilationUnit = scope.resolve(rootModelName, CNNTrainCompilationUnitSymbol.KIND);
        if (!compilationUnit.isPresent()){
            Log.error("could not resolve configuration " + rootModelName);
            System.exit(1);
        }

        CNNTrainCocos.checkAll(compilationUnit.get());

        try{
            ConfigurationSymbol configuration = compilationUnit.get().getConfiguration();
            generateFiles(configuration);
        }
        catch (IOException e){
            Log.error(e.toString());
        }
    }

    //check cocos with CNNTrainCocos.checkAll(configuration) before calling this method.
    public Map<String, String> generateStrings(ConfigurationSymbol configuration){
        Map<String, String> fileContentMap = new HashMap<>();
        CNNTrainTemplateController cnnTrainTemplateController = new CNNTrainTemplateController(configuration);
        Map.Entry<String, String> temp;

        temp = cnnTrainTemplateController.process("Config", Target.PYTHON);
        fileContentMap.put(temp.getKey(), temp.getValue());
        return fileContentMap;
    }

    //check cocos with CNNTrainCocos.checkAll(configuration) before calling this method.
    public void generateFiles(ConfigurationSymbol configuration) throws IOException{
        Log.info("Start generating files...", "Generation Test");
        CNNTrainTemplateController trainTemplateController = new CNNTrainTemplateController(configuration);
        Map<String, String> fileContentMap = generateStrings(configuration);

        for (String fileName : fileContentMap.keySet()){
            File f = new File(getGenerationTargetPath() + fileName);
            Log.info(f.getName(), "FileCreation:");
            if (!f.exists()) {
                f.getParentFile().mkdirs();
                if (!f.createNewFile()) {
                    Log.error("File could not be created");
                }
            }

            FileWriter writer = new FileWriter(f);
            writer.write(fileContentMap.get(fileName));
            writer.close();
        }
    }

}
