package de.monticore.montipipes.config;

public enum ExecutionScriptConfiguration {

    MODEL_DIRECTORY("model_dir", "./model/");

    private final String configurationName;

    private final String defaultValue;

    private String configurationValue;

    ExecutionScriptConfiguration(final String configurationName, final String defaultValue) {
        this.configurationName = configurationName;
        this.defaultValue= defaultValue;
    }

    public String getConfigurationName() {
        return configurationName;
    }
    public String getDefaultValue() {
        return defaultValue;
    }

    public void setConfigurationValue(final String configurationValue) {
        this.configurationValue = configurationValue;
    }
}
