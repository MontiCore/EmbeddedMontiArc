/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import de.monticore.generating.templateengine.GlobalExtensionManagement;

public interface GlexContribution {
    /**
     * A method which is called when a generator is configured and which can be used to add global values for templates.
     * @param glex The GlobalExtensionManagement object of the GeneratorSetup.
     */
    void defineGlobalVars(GlobalExtensionManagement glex);
}
