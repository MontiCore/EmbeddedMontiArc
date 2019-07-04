/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

public interface Generator {
    /**
     * A method which triggers configuration of the generator.
     * @throws Exception An exception which might occur during configuration.
     */
    void configure() throws Exception;

    /**
     * A method which checks whether all registered generator phases can be executed.
     * @throws Exception An exception thrown if one of the phases cannot be executed.
     */
    void canGenerate() throws Exception;

    /**
     * A method which triggers the generation process.
     * @throws Exception An exception which might occur during generation.
     */
    void generate() throws Exception;
}
