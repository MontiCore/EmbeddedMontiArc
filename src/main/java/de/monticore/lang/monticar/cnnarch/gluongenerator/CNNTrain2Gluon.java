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

        String templateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        return Collections.singletonMap("CNNTrainer_" + getInstanceName() + ".py", templateContent);
    }
}