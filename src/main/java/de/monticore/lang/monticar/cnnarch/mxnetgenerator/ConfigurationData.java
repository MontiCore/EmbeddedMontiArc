package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnntrain._symboltable.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ConfigurationData {

    ConfigurationSymbol configuration;
    String instanceName;

    public ConfigurationData(ConfigurationSymbol configuration, String instanceName) {
        this.configuration = configuration;
        this.instanceName = instanceName;
    }

    public ConfigurationSymbol getConfiguration() {
        return configuration;
    }

    public String getInstanceName() {
        return instanceName;
    }

    public String getNumEpoch() {
        return String.valueOf(getConfiguration().getNumEpoch().getValue());
    }

    public String getBatchSize() {
        return String.valueOf(getConfiguration().getBatchSize().getValue());
    }

    public LoadCheckpointSymbol getLoadCheckpoint() {
        return getConfiguration().getLoadCheckpoint();
    }

    public NormalizeSymbol getNormalize() {
        return getConfiguration().getNormalize();
    }

    public TrainContextSymbol getContext() {
        return getConfiguration().getTrainContext();
    }

    public String getOptimizerName() {
        return getConfiguration().getOptimizer().getName();
    }

    public Map<String, String> getOptimizerParams() {
        // get classes for single enum values
        List<Class> lrPolicyClasses = new ArrayList<>();
        for (LRPolicy enum_value: LRPolicy.values()) {
            lrPolicyClasses.add(enum_value.getClass());
        }

        Map<String, String>  mapToStrings = new HashMap<>();
        Map<String, OptimizerParamSymbol> optimizerParams = getConfiguration().getOptimizer().getOptimizerParamMap();
        for (Map.Entry<String, OptimizerParamSymbol> entry : optimizerParams.entrySet()) {
            String paramName = entry.getKey();
            String valueAsString = entry.getValue().toString();
            Class realClass = entry.getValue().getValue().getValue().getClass();
            if (realClass == Boolean.class) {
                valueAsString = (Boolean) entry.getValue().getValue().getValue() ? "True" : "False";
            }
            else if (lrPolicyClasses.contains(realClass)) {
                valueAsString = "'" + valueAsString + "'";
            }
            mapToStrings.put(paramName, valueAsString);
        }
        return mapToStrings;
    }
}
