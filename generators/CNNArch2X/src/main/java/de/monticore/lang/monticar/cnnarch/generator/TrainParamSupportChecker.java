/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import conflang._symboltable.ConfigurationSymbol;
import conflang._symboltable.NestedConfigurationEntrySymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.Optional;

import static de.monticore.lang.monticar.cnnarch.generator.training.TrainingParameterConstants.OPTIMIZER;

public abstract class TrainParamSupportChecker {

    public abstract Collection<String> getUnsupportedParameters();

    public abstract Collection<String> getUnsupportedOptimizers();

    public abstract Collection<String> getUnsupportedOptimizerParameters();

    public void check(ConfigurationSymbol configurationSymbol) {
        checkUnsupportedParameters(configurationSymbol);
        checkUnsupportedOptimizers(configurationSymbol);
        checkUnsupportedOptimizerParameters(configurationSymbol);
    }

    private void checkUnsupportedParameters(ConfigurationSymbol configuration) {
        Collection<String> unsupportedParameters = getUnsupportedParameters();
        if (unsupportedParameters.isEmpty()) {
            return;
        }

        for (String unsupportedParameter : unsupportedParameters) {
            if (configuration.containsConfigurationEntry(unsupportedParameter)) {
                Log.warn(String.format("Parameter '%s' is not supported with this backend and will be ignored.", unsupportedParameter));
            }
        }
    }

    private void checkUnsupportedOptimizers(ConfigurationSymbol configuration) {
        Collection<String> unsupportedOptimizers = getUnsupportedOptimizers();
        if (unsupportedOptimizers.isEmpty()) {
            return;
        }

        Optional<NestedConfigurationEntrySymbol> optimizerOpt = configuration.getConfigurationEntryOfKind(OPTIMIZER, NestedConfigurationEntrySymbol.KIND);
        if (!optimizerOpt.isPresent()) {
            return;
        }

        NestedConfigurationEntrySymbol optimizer = optimizerOpt.get();
        for (String unsupportedOptimizer : unsupportedOptimizers) {
            if (optimizer.getValue().equals(unsupportedOptimizer)) {
                Log.warn(String.format("Optimizer '%s' is not supported with this backend and will be ignored.", unsupportedOptimizer));
            }
        }
    }

    private void checkUnsupportedOptimizerParameters(ConfigurationSymbol configuration) {
        Collection<String> unsupportedOptimizerParameters = getUnsupportedOptimizerParameters();
        if (unsupportedOptimizerParameters.isEmpty()) {
            return;
        }

        Optional<NestedConfigurationEntrySymbol> optimizerOpt = configuration.getConfigurationEntryOfKind(OPTIMIZER, NestedConfigurationEntrySymbol.KIND);
        if (!optimizerOpt.isPresent()) {
            return;
        }

        NestedConfigurationEntrySymbol optimizer = optimizerOpt.get();
        for (String unsupportedParameter : unsupportedOptimizerParameters) {
            if (optimizer.hasConfigurationEntry(unsupportedParameter)) {
                Log.warn(String.format("Optimizer parameter '%s' is not supported with this backend and will be ignored.", unsupportedParameter));
            }
        }
    }
}