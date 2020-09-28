/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainNode;
import de.monticore.lang.monticar.cnntrain._ast.ASTOptimizerEntry;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.OptimizerSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

public class CNNTrain2Caffe2 extends CNNTrainGenerator {

    public CNNTrain2Caffe2() {
        trainParamSupportChecker = new CNNArch2Caffe2TrainParamSupportChecker();
    }

    @Override
    public void generate(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configuration = getConfigurationSymbol(modelsDirPath, rootModelName);
        List<FileContent> fileContents= generateStrings(configuration);
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(getGenerationTargetPath());
        try {
            for (FileContent fileContent : fileContents){
                genCPP.generateFile(fileContent);
            }
        } catch (IOException e) {
            Log.error("CNNTrainer file could not be generated" + e.getMessage());
        }
    }

    @Override
    public List<FileContent>  generateStrings(ConfigurationSymbol configuration) {
        ConfigurationData configData = new ConfigurationData(configuration, getInstanceName());
        List<ConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);
        Map<String, Object> ftlContext = Collections.singletonMap("configurations", configDataList);

        String templateContent = TemplateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp = new FileContent(templateContent, "CNNTrainer_" + getInstanceName() + ".py");
        fileContents.add(temp);
        return fileContents;
    }
}
