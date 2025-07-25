package de.monticore.lang.monticar.cnnarch.generator.configuration;

import java.nio.file.Path;
import java.nio.file.Paths;

public enum Template {
    TRAINING_CONFIGURATION_CLASS("templates.confLang.python.TrainingConfigurationClass"),
    EMPTY_CONFIGURATION_METHOD("templates.confLang.python.Empty_Configuration_Method"),
    SIMPLE_CONFIGURATION("templates.confLang.python.SimpleConfiguration"),
    NESTED_CONFIGURATION("templates.confLang.python.NestedConfiguration");

    private final String templateName;
    private final static String templateFileExtension = ".ftl";

    Template(final String templateName) {
        this.templateName = templateName;
    }

    public String getTemplateName() {
        return templateName;
    }

    public Path toPath(){
      return Paths.get("src/main/resources/"+ getTemplateName().replace(".","/")+ templateFileExtension);
    }
}
