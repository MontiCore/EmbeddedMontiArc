package de.monticore.lang.monticar.cnnarch.generator.transformation;

import conflang._ast.ASTConfLangCompilationUnit;
import conflang._ast.ASTConfigurationEntry;
import conflang._ast.ASTNestedConfigurationEntry;
import de.monticore.generating.templateengine.TemplateHookPoint;
import de.monticore.lang.monticar.cnnarch.generator.configuration.ConfLangGeneratorConfiguration;
import de.monticore.lang.monticar.cnnarch.generator.configuration.Template;

/***
 * Creates template hook ponts for AST nodes
 */
public class TemplateLinker {

    public void linkToTrainingConfigurationTemplates(ASTConfLangCompilationUnit configurationModel) {
        for (ASTConfigurationEntry astConfigurationEntry : configurationModel.getConfiguration().getAllConfigurationEntries()
        ) {
            if (astConfigurationEntry.isNestedConfiguration()) {
                ConfLangGeneratorConfiguration.getGlobalExtensionManagement().replaceTemplate(Template.TRAINING_CONFIGURATION_METHOD.getTemplateName(),
                        astConfigurationEntry, new TemplateHookPoint(Template.NESTED_CONFIGURATION.getTemplateName()));

                for (ASTConfigurationEntry nestedEntry : ((ASTNestedConfigurationEntry) astConfigurationEntry).getConfigurationEntryList()
                ) {
                    ConfLangGeneratorConfiguration.getGlobalExtensionManagement().replaceTemplate(Template.TRAINING_CONFIGURATION_METHOD.getTemplateName(),
                            nestedEntry, new TemplateHookPoint(Template.SIMPLE_CONFIGURATION.getTemplateName()));

                }
            }
            else {
                ConfLangGeneratorConfiguration.getGlobalExtensionManagement().replaceTemplate(Template.TRAINING_CONFIGURATION_METHOD.getTemplateName(),
                        astConfigurationEntry, new TemplateHookPoint(Template.SIMPLE_CONFIGURATION.getTemplateName()));
            }
            }
    }

}
