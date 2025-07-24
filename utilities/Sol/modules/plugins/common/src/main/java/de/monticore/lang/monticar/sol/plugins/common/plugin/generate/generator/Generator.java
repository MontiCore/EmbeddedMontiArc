/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

public interface Generator {
    /**
     * A method which triggers configuration of the generator.
     * @throws Exception An exception which might occur during configuration.
     */
    void configure() throws Exception;

    /**
     * A method which triggers the generation process.
     * @throws Exception An exception which might occur during generation.
     */
    void generate() throws Exception;

    /**
     * A method which triggers the shutdown process.
     * @throws Exception An exception which might occur during shutdown.
     */
    void shutdown() throws Exception;
}
