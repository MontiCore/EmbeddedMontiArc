package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import com.google.common.collect.Maps;
import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.util.*;
import java.nio.file.Path;

public class CNNTrain2Tensorflow extends CNNTrainGenerator {

    public CNNTrain2Tensorflow() {
        trainParamSupportChecker = new CNNArch2TensorflowTrainParamSupportChecker();
    }
    
    @Override
    public ConfigurationSymbol getConfigurationSymbol(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configurationSymbol = super.getConfigurationSymbol(modelsDirPath, rootModelName);
        return configurationSymbol;
    }

	@Override
    public void generate(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configuration = this.getConfigurationSymbol(modelsDirPath, rootModelName);
        
        generateFilesFromConfigurationSymbol(configuration);
    }
    
    private void generateFilesFromConfigurationSymbol(ConfigurationSymbol configuration) {
        Map<String, String> fileContents = this.generateStrings(configuration);
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(this.getGenerationTargetPath());

        try {
            Iterator var6 = fileContents.keySet().iterator();

            while(var6.hasNext()) {
                String fileName = (String)var6.next();
                genCPP.generateFile(new FileContent((String)fileContents.get(fileName), fileName));
            }
        } catch (IOException var8) {
            Log.error("CNNTrainer file could not be generated" + var8.getMessage());
        }
    }

	@Override
    public Map<String, String> generateStrings(ConfigurationSymbol configuration) {
        TemplateConfiguration templateConfiguration = new TensorflowTemplateConfiguration();
        
        ConfigurationData configData = new ConfigurationData(configuration, getInstanceName());
        
        List<ConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);
        
        Map<String, Object> ftlContext = Maps.newHashMap();
        ftlContext.put("configurations", configDataList);
        Map<String, String> fileContentMap = new HashMap<>();

        String cnnTrainTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        fileContentMap.put("CNNTrainer_" + getInstanceName() + ".py", cnnTrainTemplateContent);
 
        return fileContentMap;
    }
}