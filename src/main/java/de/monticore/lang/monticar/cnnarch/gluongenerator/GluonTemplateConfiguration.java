/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.gluongenerator;

import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import freemarker.template.Configuration;

import java.util.Map;

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

    public Map<String, Object> getOptimizerParameters() {

        return null;
    }
}
