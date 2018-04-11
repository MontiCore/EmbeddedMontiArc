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
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
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
        Map.Entry<String,String> fileContent = generateFileContent(scope, rootModelName);
        try {
            generateConfigFile(fileContent);
        }
        catch (IOException e){
            Log.error(e.toString());
        }
    }

    public Map.Entry<String,String> generateFileContent(Scope scope, String rootModelName){
        Optional<CNNTrainCompilationUnitSymbol> compilationUnit = scope.resolve(rootModelName, CNNTrainCompilationUnitSymbol.KIND);
        if (!compilationUnit.isPresent()){
            Log.error("CNNTrainCompilationUnitSymbol is empty. Could not resolve configuration " + rootModelName);
            System.exit(1);
        }
        CNNTrainCocos.checkAll(compilationUnit.get());
        ConfigurationSymbol configuration = compilationUnit.get().getConfiguration();
        CNNTrainTemplateController cnnTrainTemplateController = new CNNTrainTemplateController(configuration);
        Map.Entry<String, String> fileContent1;

        fileContent1 = cnnTrainTemplateController.process("Config", Target.PYTHON);
        Map.Entry<String,String> fileContent = fileContent1;
        return fileContent;

    }

    public void generateConfigFile(Map.Entry<String,String> fileContent) throws IOException{
        Log.info("Start generating config file...", "Generation Test");

        File f = new File(getGenerationTargetPath() + fileContent.getKey());
        Log.info(f.getName(), "FileCreation:");
        if (!f.exists()) {
            f.getParentFile().mkdirs();
            if (!f.createNewFile()) {
                Log.error("File could not be created");
            }
        }

        FileWriter writer = new FileWriter(f);
        writer.write(fileContent.getValue());
        writer.close();
    }

}
