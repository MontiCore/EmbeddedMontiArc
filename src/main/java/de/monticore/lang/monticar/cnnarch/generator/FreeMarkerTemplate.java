package de.monticore.lang.monticar.cnnarch.generator;

public enum FreeMarkerTemplate {
    SCHEMA_CLASS("templates.SchemaAccessClass"),
    SCHEMA_API_Method("templates.Schema_API_Method"),
    SCHEMA_API_OBJECT_TYPE("templates.SchemaAPIObjectType"),
    SCHEMA_API_OBJECTTYPE("templates.SchemaAPI_ObjectType");

    private final String templateName;

    FreeMarkerTemplate(final String templateName) {
        this.templateName = templateName;
    }

    public String getTemplateName() {
        return templateName;
    }
}
