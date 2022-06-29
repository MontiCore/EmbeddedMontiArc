package de.monticore.lang.monticar.cnnarch.generator;

public enum FreeMarkerTemplate {
    SCHEMA_CLASS("templates.SchemaAccessClass"),
    SCHEMA_API_Method("templates.SchemaAPIMethod");

    private final String templateName;

    FreeMarkerTemplate(final String templateName) {
        this.templateName = templateName;
    }

    public String getTemplateName() {
        return templateName;
    }
}
