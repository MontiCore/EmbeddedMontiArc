package de.monticore.montipipes.config;

public enum Template {
    PIPELINE_EXECUTION("templates.pipeline_executor");

    private final String templateName;

    Template(final String templateName) {
        this.templateName = templateName;
    }

    public String getTemplateName() {
        return templateName;
    }
}
