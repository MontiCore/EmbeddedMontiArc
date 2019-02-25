package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.mxnetgenerator.TemplateConfiguration;
import freemarker.template.Configuration;

/**
 *
 */
public class GluonTemplateConfiguration extends TemplateConfiguration {
    private static Configuration configuration;

    public GluonTemplateConfiguration() {
        super();
        if (configuration == null) {
            configuration = super.createConfiguration();
        }
    }

    @Override
    protected String getBaseTemplatePackagePath() {
        return "/templates/gluon/";
    }

    @Override
    public Configuration getConfiguration() {
        return configuration;
    }
}
