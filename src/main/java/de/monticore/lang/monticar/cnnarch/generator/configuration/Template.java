package de.monticore.lang.monticar.cnnarch.generator.configuration;

public enum Template {
    TRAINING_CONFIGURATION_CLASS("templates.confLang.python.TrainingConfigurationClass"),
    TRAINING_CONFIGURATION_METHOD("templates.confLang.python.Training_Configuration_Method"),
    SIMPLE_CONFIGURATION("templates.confLang.python.SimpleConfiguration"),
    NESTED_CONFIGURATION("templates.confLang.python.NestedConfiguration");

    private final String templateName;

    Template(final String templateName) {
        this.templateName = templateName;
    }

    public String getTemplateName() {
        return templateName;
    }
}
