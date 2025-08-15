/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;

public class Caffee2ConfigurationData extends ConfigurationData {

    public Caffee2ConfigurationData(TrainingConfiguration trainingConfiguration,
                                    TrainingComponentsContainer trainingComponentsContainer, String instanceName, TrainParamSupportChecker trainParamSupportChecker) {
        super(trainingConfiguration, trainingComponentsContainer, instanceName, trainParamSupportChecker);
    }
}