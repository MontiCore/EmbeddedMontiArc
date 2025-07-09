/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.tensorflowgenerator;

import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import freemarker.template.Configuration;

public class TensorflowTemplateConfiguration extends TemplateConfiguration {
    private static Configuration configuration;

    public TensorflowTemplateConfiguration() {
        super();
        if (configuration == null) {
            configuration = super.createConfiguration();
        }
    }

    @Override
    protected String getBaseTemplatePackagePath() {
        return "/templates/tensorflow/";
    }

    @Override
    public Configuration getConfiguration() {
        return configuration;
    }
}
