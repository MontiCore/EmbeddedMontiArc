/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;
import de.monticore.lang.monticar.generator.FileContent;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;

public class CNNTrain2Caffe2 extends CNNTrainGenerator {

    public CNNTrain2Caffe2() {
        super(new CNNArch2Caffe2TrainParamSupportChecker());
    }

    @Override
    public void generate(Path modelsDirPath, String rootModelName) {
        TrainingConfiguration trainingConfiguration = createTrainingConfiguration(modelsDirPath, rootModelName, null);
        Caffee2ConfigurationData configurationData = new Caffee2ConfigurationData(trainingConfiguration,
                new TrainingComponentsContainer(), getInstanceName(), new CNNArch2Caffe2TrainParamSupportChecker());
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
        Caffee2ConfigurationData configData = new Caffee2ConfigurationData(trainingConfiguration,
                trainingComponentsContainer, getInstanceName(), new CNNArch2Caffe2TrainParamSupportChecker());
        List<Caffee2ConfigurationData> configDataList = Lists.newArrayList(configData);
        Map<String, Object> ftlContext = Collections.singletonMap("configurations", configDataList);

        String templateContent = TemplateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        List<FileContent> fileContents = new ArrayList<>();
        FileContent temp = new FileContent(templateContent, "CNNTrainer_" + getInstanceName() + ".py");
        fileContents.add(temp);
        return fileContents;
    }
}