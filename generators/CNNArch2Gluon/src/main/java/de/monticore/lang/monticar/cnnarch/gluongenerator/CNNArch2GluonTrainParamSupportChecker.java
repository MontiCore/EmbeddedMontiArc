/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;


import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;

import java.util.Collection;

public class CNNArch2GluonTrainParamSupportChecker extends TrainParamSupportChecker {

    @Override
    public Collection<String> getUnsupportedParameters() {
        return Lists.newArrayList();
    }

    @Override
    public Collection<String> getUnsupportedOptimizers() {
        return Lists.newArrayList();
    }

    @Override
    public Collection<String> getUnsupportedOptimizerParameters() {
        return Lists.newArrayList();
    }
}