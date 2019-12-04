/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnntrain._symboltable.*;
import static de.monticore.lang.monticar.cnntrain.helper.ConfigEntryNameConstants.*;

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

    public Map<String, Object> getEvalMetric() {
        return getMultiParamEntry(EVAL_METRIC, "name");
    }

    public String getLossName() {
        if (getConfiguration().getLoss() == null) {
            return null;
        }
        return getConfiguration().getLoss().getName();
    }

    public Map<String, String> getLossParams() {

        Map<String, String>  mapToStrings = new HashMap<>();
        Map<String, LossParamSymbol> lossParams = getConfiguration().getLoss().getLossParamMap();
        for (Map.Entry<String, LossParamSymbol> entry : lossParams.entrySet()) {
            String paramName = entry.getKey();
            String valueAsString = entry.getValue().toString();
            Class realClass = entry.getValue().getValue().getValue().getClass();
            if (realClass == Boolean.class) {
                valueAsString = (Boolean) entry.getValue().getValue().getValue() ? "True" : "False";
            }
            mapToStrings.put(paramName, valueAsString);
        }
        if (mapToStrings.isEmpty()){
            return null;
        } else{
            return mapToStrings;}
    }
    
    public String getLossWeights() {
        if (!getConfiguration().getEntryMap().containsKey("loss_weights")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("loss_weights").getValue());
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

    public Boolean getSaveAttentionImage() {
        if (!getConfiguration().getEntryMap().containsKey("save_attention_image")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("save_attention_image").getValue().getValue();
    }

    public Boolean getUseTeacherForcing() {
        if (!getConfiguration().getEntryMap().containsKey("use_teacher_forcing")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("use_teacher_forcing").getValue().getValue();
    }

    protected Map<String, Object> getMultiParamEntry(final String key, final String valueName) {
        if (!configurationContainsKey(key)) {
            return null;
        }

        Map<String, Object> resultView = new HashMap<>();

        ValueSymbol value = this.getConfiguration().getEntryMap().get(key).getValue();

        if (value instanceof MultiParamValueSymbol) {
            MultiParamValueSymbol multiParamValue = (MultiParamValueSymbol) value;
            resultView.put(valueName, multiParamValue.getValue());
            resultView.putAll(multiParamValue.getParameters());
        }
        else {
            resultView.put(valueName, value.getValue());
        }

        return resultView;
    }

    protected Boolean configurationContainsKey(final String key) {
        return this.getConfiguration().getEntryMap().containsKey(key);
    }

    protected Object retrieveConfigurationEntryValueByKey(final String key) {
        return this.getConfiguration().getEntry(key).getValue().getValue();
    }
}
