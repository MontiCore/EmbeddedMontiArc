/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.ConfigurationData;
import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;

public class MXNetConfigurationData extends ConfigurationData {

    public MXNetConfigurationData(TrainingConfiguration trainingConfiguration,
                                  TrainingComponentsContainer trainingComponentsContainer, String instanceName, TrainParamSupportChecker trainParamSupportChecker) {
        super(trainingConfiguration, trainingComponentsContainer, instanceName, trainParamSupportChecker);
    }

    /*
     * Add MxNet-specific configuration here.
     */
}