/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;

public interface TemplateVariableService {
    /**
     * @param text The text for which the variables should be resolved.
     * @param template The template which acts as input for the resolving function.
     * @return The text with the variables replaced by their values.
     */
    String resolve(String text, Template template);
}
