/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;
import de.monticore.lang.monticar.generator.FileContent;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class CNNTrain2MxNet extends CNNTrainGenerator {

    public CNNTrain2MxNet() {
        super(new CNNArch2MxNetTrainParamSupportChecker());
    }

    @Override
    public void generate(Path modelsDirPath, String rootModelName) {
        TrainingConfiguration trainingConfiguration = createTrainingConfiguration(modelsDirPath, rootModelName, null);
        MXNetConfigurationData configurationData = new MXNetConfigurationData(trainingConfiguration,
                new TrainingComponentsContainer(), getInstanceName(), new CNNArch2MxNetTrainParamSupportChecker());
        if (configurationData.isReinforcementLearning()) {
            throw new IllegalStateException("Cannot call generate of reinforcement configuration without specifying " +
                    "the trained architecture");
        }
        generateFilesFromConfigurationSymbol(trainingConfiguration, new TrainingComponentsContainer(), null);
    }

    @Override
    public List<FileContent> generateStrings(TrainingConfiguration trainingConfiguration,
                                             TrainingComponentsContainer trainingComponentsContainer,
                                             Path outputPath) {
        TemplateConfiguration templateConfiguration = new MxNetTemplateConfiguration();
        MXNetConfigurationData configData = new MXNetConfigurationData(trainingConfiguration,
                trainingComponentsContainer, getInstanceName(), new CNNArch2MxNetTrainParamSupportChecker());
        List<MXNetConfigurationData> configDataList = Lists.newArrayList(configData);
        Map<String, Object> ftlContext = Collections.singletonMap("configurations", configDataList);

        String templateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp = new FileContent(templateContent, "CNNTrainer_" + getInstanceName() + ".py");
        fileContents.add(temp);
        return fileContents;
    }
}