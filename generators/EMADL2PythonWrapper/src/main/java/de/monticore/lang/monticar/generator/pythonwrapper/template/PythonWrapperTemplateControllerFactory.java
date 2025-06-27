/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.template;

import de.monticore.lang.monticar.generator.pythonwrapper.template.configuration.PythonWrapperTemplateConfigurationProvider;
import de.monticore.lang.monticar.generator.pythonwrapper.template.configuration.TemplateConfigurationProvider;

/**
 *
 */
public class PythonWrapperTemplateControllerFactory {
    public TemplateController create() {
        TemplateConfigurationProvider configurationProvider = new PythonWrapperTemplateConfigurationProvider();
        TemplateLoader templateLoader = new TemplateLoader(configurationProvider.getConfiguration());
        TemplateDataPreparer templateDataPreparer = new TemplateDataPreparer();
        return new TemplateController(templateLoader, templateDataPreparer);
    }
}
