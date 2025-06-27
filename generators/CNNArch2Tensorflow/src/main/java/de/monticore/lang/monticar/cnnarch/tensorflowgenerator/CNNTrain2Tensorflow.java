/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import com.google.common.collect.Maps;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;
import de.monticore.lang.monticar.generator.FileContent;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class CNNTrain2Tensorflow extends CNNTrainGenerator {

    public CNNTrain2Tensorflow() {
        super(new CNNArch2TensorflowTrainParamSupportChecker());
    }
    
	@Override
    public void generate(Path modelsDirPath, String rootModelName) {
        TrainingConfiguration trainingConfiguration = createTrainingConfiguration(modelsDirPath, rootModelName, null);
        TensorflowConfigurationData configurationData = new TensorflowConfigurationData(trainingConfiguration,
                new TrainingComponentsContainer(), getInstanceName(), new CNNArch2TensorflowTrainParamSupportChecker());
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
        TemplateConfiguration templateConfiguration = new TensorflowTemplateConfiguration();
        TensorflowConfigurationData configData = new TensorflowConfigurationData(trainingConfiguration,
                trainingComponentsContainer, getInstanceName(), new CNNArch2TensorflowTrainParamSupportChecker());
        List<TensorflowConfigurationData> configDataList = new ArrayList<>();
        configDataList.add(configData);
        
        Map<String, Object> ftlContext = Maps.newHashMap();
        ftlContext.put("configurations", configDataList);
        List<FileContent> fileContents = new ArrayList<>();

        String cnnTrainTemplateContent = templateConfiguration.processTemplate(ftlContext, "CNNTrainer.ftl");
        fileContents.add(new FileContent(cnnTrainTemplateContent, "CNNTrainer_" + getInstanceName() + ".py"));
        return fileContents;
    }
}