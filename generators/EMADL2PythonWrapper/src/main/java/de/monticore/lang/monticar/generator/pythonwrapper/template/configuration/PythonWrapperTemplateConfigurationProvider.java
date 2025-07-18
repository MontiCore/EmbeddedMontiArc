/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template.configuration;

import freemarker.template.Configuration;
import freemarker.template.TemplateExceptionHandler;

/**
 *
 */
public class PythonWrapperTemplateConfigurationProvider implements TemplateConfigurationProvider {
    private static Configuration templateConfiguration;
    private static final String PATH_TO_TEMPLATES_DIRECTORY = "/de/monticore/lang/monticar/generator/pythonwrapper/template";

    public PythonWrapperTemplateConfigurationProvider() {

    }

    public Configuration getConfiguration() {
        if (templateConfiguration == null) {
            templateConfiguration = new Configuration(Configuration.VERSION_2_3_23);
            templateConfiguration.setDefaultEncoding("UTF-8");
            templateConfiguration.setTemplateExceptionHandler(TemplateExceptionHandler.RETHROW_HANDLER);
            templateConfiguration.setLogTemplateExceptions(false);
            templateConfiguration.setClassForTemplateLoading(this.getClass(), PATH_TO_TEMPLATES_DIRECTORY);
        }
        return templateConfiguration;
    }
}
