/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.template;

public interface TemplateContribution {
    /**
     * A method which is called once the TemplateRegistry is being configured.
     * @param registry The TemplateRegistry to be configured.
     */
    void registerTemplates(TemplateRegistry registry);
}
