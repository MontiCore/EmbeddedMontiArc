package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.mxnetgenerator.ConfigurationData;

import de.monticore.lang.monticar.cnnarch.mxnetgenerator.CNNTrain2MxNet;
import de.monticore.lang.monticar.cnnarch.mxnetgenerator.TemplateConfiguration;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;

import java.util.*;

public class CNNTrain2Gluon extends CNNTrain2MxNet {
    public CNNTrain2Gluon() {
        super();
    }

    @Override
    public Map<String, String> generateStrings(ConfigurationSymbol configuration) {
        TemplateConfiguration templateConfiguration = new GluonTemplateConfiguration();
        ConfigurationData configData = new ConfigurationData(configuration, getInstanceName());
        List<ConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);
        Map<String, Object> ftlContext = Collections.singletonMap("configurations", configDataList);

        Map<String, String> fileContentMap = new HashMap<>();

        String cnnTrainTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        fileContentMap.put("CNNTrainer_" + getInstanceName() + ".py", cnnTrainTemplateContent);

        String cnnSupervisedTrainerContent = templateConfiguration.processTemplate(ftlContext, "CNNSupervisedTrainer.ftl");
        fileContentMap.put("supervised_trainer.py", cnnSupervisedTrainerContent);

        return fileContentMap;
    }
}