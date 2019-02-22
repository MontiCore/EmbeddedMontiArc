package de.monticore.lang.monticar.cnnarch.gluongenerator;

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
        if (!getConfiguration().getEntryMap().containsKey("num_epoch")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("num_epoch").getValue());
    }

    public String getBatchSize() {
        if (!getConfiguration().getEntryMap().containsKey("batch_size")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("batch_size") .getValue());
    }

    public Boolean getLoadCheckpoint() {
        if (!getConfiguration().getEntryMap().containsKey("load_checkpoint")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("load_checkpoint").getValue().getValue();
    }

    public Boolean getNormalize() {
        if (!getConfiguration().getEntryMap().containsKey("normalize")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("normalize").getValue().getValue();
    }

    public String getContext() {
        if (!getConfiguration().getEntryMap().containsKey("context")) {
            return null;
        }
        return getConfiguration().getEntry("context").getValue().toString();
    }

    public String getEvalMetric() {
        if (!getConfiguration().getEntryMap().containsKey("eval_metric")) {
            return null;
        }
        return getConfiguration().getEntry("eval_metric").getValue().toString();
    }

    public String getOptimizerName() {
        if (getConfiguration().getOptimizer() == null) {
            return null;
        }
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
            } else if (lrPolicyClasses.contains(realClass)) {
                valueAsString = "'" + valueAsString + "'";
            }
            mapToStrings.put(paramName, valueAsString);
        }
        return mapToStrings;
    }
}
