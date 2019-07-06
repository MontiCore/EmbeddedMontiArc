package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.se_rwth.commons.logging.Log;

import java.nio.file.Path;
import java.io.IOException;
import java.util.*;

public class CNNTrain2MxNet extends CNNTrainGenerator {

    @Override
    public void generate(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configuration = getConfigurationSymbol(modelsDirPath, rootModelName);
        Map<String, String> fileContents = generateStrings(configuration);
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(getGenerationTargetPath());
        try {
            for (String fileName : fileContents.keySet()){
                genCPP.generateFile(new FileContent(fileContents.get(fileName), fileName));
            }
        } catch (IOException e) {
            Log.error("CNNTrainer file could not be generated" + e.getMessage());
        }
    }

    @Override
    public Map<String, String> generateStrings(ConfigurationSymbol configuration) {
        TemplateConfiguration templateConfiguration = new MxNetTemplateConfiguration();
        ConfigurationData configData = new ConfigurationData(configuration, getInstanceName());
        List<ConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);
        Map<String, Object> ftlContext = Collections.singletonMap("configurations", configDataList);

        String templateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        return Collections.singletonMap("CNNTrainer_" + getInstanceName() + ".py", templateContent);
    }
}