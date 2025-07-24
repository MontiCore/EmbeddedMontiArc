/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import freemarker.template.Configuration;

public class MxNetTemplateConfiguration extends TemplateConfiguration {
    private static Configuration configuration;

    public MxNetTemplateConfiguration() {
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
