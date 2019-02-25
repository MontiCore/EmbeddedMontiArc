package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import freemarker.template.Configuration;

/**
 *
 */
public class MxNetTemplateConfiguration extends TemplateConfiguration {
    private static Configuration configuration;

    public MxNetTemplateConfiguration() {
        super();
        if (configuration == null) {
            configuration = super.createConfiguration();
        }
    }

    @Override
    protected String getBaseTemplatePackagePath() {
        return "/templates/mxnet/";
    }

    @Override
    public Configuration getConfiguration() {
        return configuration;
    }
}