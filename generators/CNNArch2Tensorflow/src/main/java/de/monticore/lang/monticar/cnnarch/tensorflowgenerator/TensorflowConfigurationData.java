/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;

public class TensorflowConfigurationData extends ConfigurationData {

    public TensorflowConfigurationData(TrainingConfiguration trainingConfiguration,
                                       TrainingComponentsContainer trainingComponentsContainer, String instanceName, TrainParamSupportChecker trainParamSupportChecker) {
        super(trainingConfiguration, trainingComponentsContainer, instanceName, trainParamSupportChecker);
    }

    /*
     * Add Tensorflow-specific configuration here.
     */
}