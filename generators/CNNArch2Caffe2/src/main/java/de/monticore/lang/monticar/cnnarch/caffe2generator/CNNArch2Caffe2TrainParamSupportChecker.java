/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.caffe2generator;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;

import java.util.Collection;

import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.*;
import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.CENTERED;

public class CNNArch2Caffe2TrainParamSupportChecker extends TrainParamSupportChecker {

    @Override
    public Collection<String> getUnsupportedParameters() {
        return Lists.newArrayList(LOAD_CHECKPOINT, NORMALIZE);
    }

    @Override
    public Collection<String> getUnsupportedOptimizers() {
        return Lists.newArrayList(OPTIMIZER_ADA_DELTA, OPTIMIZER_NESTEROV);
    }

    @Override
    public Collection<String> getUnsupportedOptimizerParameters() {
        return Lists.newArrayList(LEARNING_RATE_MINIMUM,
                RESCALE_GRAD, CLIP_GRADIENT, GAMMA2, CLIP_WEIGHTS, CENTERED);
    }
}