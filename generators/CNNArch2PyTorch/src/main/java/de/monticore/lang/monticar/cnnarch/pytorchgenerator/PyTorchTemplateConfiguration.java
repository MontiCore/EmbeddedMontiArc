package de.monticore.lang.monticar.cnnarch.pytorchgenerator;

import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import freemarker.template.Configuration;

public class PyTorchTemplateConfiguration extends TemplateConfiguration {
    private static Configuration configuration;

    public PyTorchTemplateConfiguration() {
        super();
        if (configuration == null) {
            configuration = super.createConfiguration();
        }
    }

    @Override
    protected String getBaseTemplatePackagePath() {
        return "/templates/pytorch/";
    }

    @Override
    public Configuration getConfiguration() {
        return configuration;
    }
}
