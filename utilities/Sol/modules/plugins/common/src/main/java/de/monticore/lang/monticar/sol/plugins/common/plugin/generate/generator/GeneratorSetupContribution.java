/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import de.monticore.generating.GeneratorSetup;

public interface GeneratorSetupContribution {
    /**
     * A method which is called when a generator is configured and which can be used to set attributes of GeneratorSetup.
     * @param setup The GeneratorSetup object whose attributes will be set.
     */
    void setup(GeneratorSetup setup);
}
