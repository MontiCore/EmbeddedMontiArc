/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

import java.util.Set;

public interface TemplateRegistry {
    /**
     * @return The registered templates.
     */
    Set<Template> getTemplates();

    /**
     * A method which can be used to register a template.
     * @param templatePath The relative path to the template to be registered.
     * @param outputPath The relative path where the output file should be generated to.
     * @param arguments Arguments to be passed to the Generator Engine.
     */
    void registerTemplate(String templatePath, String outputPath, Object ...arguments);

    /**
     * Sets the root directory of the templates.
     * @param templateRoot The relative path where template resolution should start.
     */
    void setTemplateRoot(String templateRoot);

    /**
     * @return The relative path where the template resolution should start.
     */
    String getTemplateRoot();

    /**
     * Sets the suffix which should be added to generated files when handwritten code exists.
     * @param suffix The suffix to be set.
     */
    void setTopPatternSuffix(String suffix);

    /**
     * @return The suffix which is added to generated files when handwritten code exists.
     */
    String getTopPatternSuffix();
}
