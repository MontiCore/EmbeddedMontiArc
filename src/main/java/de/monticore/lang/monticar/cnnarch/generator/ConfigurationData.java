/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.monticore.lang.monticar.cnntrain._symboltable.*;
import jline.internal.Log;

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

    public String getKValue() {
        if (!getConfiguration().getEntryMap().containsKey("k_value")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("k_value") .getValue());
    }

    public String getGeneratorLossWeight() {
        if (!getConfiguration().getEntryMap().containsKey("generator_loss_weight")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("generator_loss_weight") .getValue());
    }

    public String getDiscriminatorLossWeight() {
        if (!getConfiguration().getEntryMap().containsKey("discriminator_loss_weight")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("discriminator_loss_weight") .getValue());
    }

    public String getSpeedPeriod() {
        if (!getConfiguration().getEntryMap().containsKey("speed_period")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("speed_period") .getValue());
    }

    public Boolean getPrintImages() {
        if (!getConfiguration().getEntryMap().containsKey("print_images")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("print_images").getValue().getValue();
    }

    public String getGeneratorLoss() {
        if (!getConfiguration().getEntryMap().containsKey("generator_loss")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("generator_loss") .getValue());
    }

    public String getGeneratorTargetName() {
        if (!getConfiguration().getEntryMap().containsKey("generator_target_name")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("generator_target_name") .getValue());
    }

    public String getNoiseInput() {
        if (!getConfiguration().getEntryMap().containsKey("noise_input")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("noise_input") .getValue());
    }

    public Boolean getLoadCheckpoint() {
        if (!getConfiguration().getEntryMap().containsKey("load_checkpoint")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("load_checkpoint").getValue().getValue();
    }

    public String getCheckpointPeriod() {
        if (!getConfiguration().getEntryMap().containsKey("checkpoint_period")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("checkpoint_period").getValue());
    }

    public String getLogPeriod() {
        if (!getConfiguration().getEntryMap().containsKey("log_period")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("log_period").getValue());
    }

    public Boolean getLoadPretrained() {
        if (!getConfiguration().getEntryMap().containsKey("load_pretrained")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("load_pretrained").getValue().getValue();
    }

    public Boolean getNormalize() {
        if (!getConfiguration().getEntryMap().containsKey("normalize")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("normalize").getValue().getValue();
    }

    public Boolean getShuffleData() {
        if (!getConfiguration().getEntryMap().containsKey("shuffle_data")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("shuffle_data").getValue().getValue();
    }

    public String getClipGlobalGradNorm() {
        if (!getConfiguration().getEntryMap().containsKey("clip_global_grad_norm")) {
            return null;
        }
        return String.valueOf(getConfiguration().getEntry("clip_global_grad_norm").getValue());
    }

    public String getPreprocessingName() {
        if (!getConfiguration().getEntryMap().containsKey("preprocessing_name")) {
            return null;
        }
        return (String) getConfiguration().getEntry("preprocessing_name").getValue().toString();
    }

    public Boolean getPreprocessor() {
        return (Boolean) configuration.hasPreprocessor();
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

    public Boolean getEvalTrain() {
        if (!getConfiguration().getEntryMap().containsKey("eval_train")) {
            return null;
        }
        return (Boolean) getConfiguration().getEntry("eval_train").getValue().getValue();
    }

    protected Map<String, Map<String, Object>> getMultiParamMapEntry(final String key, final String valueName) {
        if (!configurationContainsKey(key)) {
            return null;
        }

        Map<String, Map<String,Object>> resultView = new HashMap<>();

        ValueSymbol value = this.getConfiguration().getEntryMap().get(key).getValue();

        if (value instanceof MultiParamValueMapSymbol) {
            MultiParamValueMapSymbol multiParamValueMap = (MultiParamValueMapSymbol) value;
            resultView.putAll(multiParamValueMap.getParameters());
            Map<String,String> names = multiParamValueMap.getMultiParamValueNames();
            for(String distrName : names.keySet()) {
                resultView.get(distrName).put(valueName, names.get(distrName));
            }
        }

        return resultView;
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
