package de.monticore.lang.monticar.cnnarch.generator.validation;

import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch.generator.annotations.ArchitectureAdapter;
import de.monticore.lang.monticar.cnnarch.generator.training.Hyperparameter;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.*;

public class TrainedArchitectureChecker {

    public static void checkComponents(TrainingConfiguration trainingConfiguration,
                                       TrainingComponentsContainer trainingComponentsContainer) {
        checkOUParameterDimensionEqualsActionDimension(trainingConfiguration, trainingComponentsContainer);
    }

    private static void checkOUParameterDimensionEqualsActionDimension(TrainingConfiguration trainingConfiguration,
                                                                       TrainingComponentsContainer trainingComponentsContainer) {

        Optional<ArchitectureAdapter> trainedArchitectureOpt = trainingComponentsContainer.getTrainedArchitecture();
        if (!trainedArchitectureOpt.isPresent() || !trainingConfiguration.isReinforcementLearning()
                || !trainingConfiguration.hasStrategy()) {
            return;
        }

        Optional<String> strategyNameOpt = trainingConfiguration.getStrategyName();
        String strategyName = strategyNameOpt.get();
        if (STRATEGY_OU.equals(strategyName)) {
            final ArchitectureAdapter architectureAdapter = trainingComponentsContainer.getTrainedArchitecture().get();
            final String outputNameOfTrainedArchitecture = architectureAdapter.getOutputs().get(0);
            final List<Integer> actionDimensions = architectureAdapter.getDimensions().get(outputNameOfTrainedArchitecture);
            if (actionDimensions.isEmpty()) {
                return;
            }
            final int vectorSize = actionDimensions.get(0);

            Optional<Hyperparameter> strategyOpt = trainingConfiguration.getStrategyParameter(STRATEGY_OU_MU);
            if (strategyOpt.isPresent()) {
                Hyperparameter strategy = strategyOpt.get();
                logIfDimensionIsUnequal(architectureAdapter.getArchitectureSymbol(), (Collection<?>) strategy.getValue(),
                        vectorSize, outputNameOfTrainedArchitecture, STRATEGY_OU_MU);
            }

            strategyOpt = trainingConfiguration.getStrategyParameter(STRATEGY_OU_SIGMA);
            if (strategyOpt.isPresent()) {
                Hyperparameter strategy = strategyOpt.get();
                logIfDimensionIsUnequal(architectureAdapter.getArchitectureSymbol(), (Collection<?>) strategy.getValue(),
                        vectorSize, outputNameOfTrainedArchitecture, STRATEGY_OU_SIGMA);
            }

            strategyOpt = trainingConfiguration.getStrategyParameter(STRATEGY_OU_THETA);
            if (strategyOpt.isPresent()) {
                Hyperparameter strategy = strategyOpt.get();
                logIfDimensionIsUnequal(architectureAdapter.getArchitectureSymbol(), (Collection<?>) strategy.getValue(),
                        vectorSize, outputNameOfTrainedArchitecture, STRATEGY_OU_THETA);
            }
        }
    }

    private static void logIfDimensionIsUnequal(ArchitectureSymbol architectureSymbol, Collection<?> ouParameters, int actionVectorDimension,
                                                String outputNameOfTrainedArchitecture, String ouParameterName) {
        final int ouParameterDimension = ouParameters.size();
        if (ouParameterDimension != actionVectorDimension) {
            String message = String.format("x0xC7102 Vector parameter '%s' of parameter '%s' must have the same dimensions as the action " +
                    "dimension of output '%s' which is %s.", ouParameterName, STRATEGY_OU, outputNameOfTrainedArchitecture, actionVectorDimension);
            Log.error(message, architectureSymbol.getSourcePosition());
        }
    }
}