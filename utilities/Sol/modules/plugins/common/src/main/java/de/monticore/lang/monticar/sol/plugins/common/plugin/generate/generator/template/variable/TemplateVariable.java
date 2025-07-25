/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.variable;

import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template.Template;

public interface TemplateVariable {
    /**
     * @return A unique identifier which can be used as variable.
     */
    String getIdentifier();

    /**
     * @param template The context as template in which the variable will be resolved.
     * @return The value with which the variable will be replaced.
     */
    String resolve(Template template);
}
