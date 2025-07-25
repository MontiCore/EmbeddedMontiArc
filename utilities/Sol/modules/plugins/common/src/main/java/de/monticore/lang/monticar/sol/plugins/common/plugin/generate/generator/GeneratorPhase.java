/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import de.monticore.generating.GeneratorEngine;

public interface GeneratorPhase {
    /**
     * @return A label from which a developer can identify the phase.
     */
    String getLabel();

    /**
     * @return At what position this phase should be executed. Higher means sooner.
     */
    int getPriority();

    /**
     * Triggers configuration of this generation phase.
     * @throws Exception An exception which might occur during configuration.
     */
    default void configure() throws Exception {}

    /**
     * @return True if the phase **should** be executed, false otherwise. Can be used to compare input against state.
     */
    default boolean shouldGenerate() {
        return true;
    }

    /**
     * Triggers the execution of this generation phase.
     * @param engine The GeneratorEngine on which the GeneratorPhase will be executed.
     * @throws Exception An exception which might occur during this generation phase.
     */
    void generate(GeneratorEngine engine) throws Exception;

    /**
     * Triggers shutdown of this generation phase.
     * @throws Exception An exception which might occur during shutdown.
     */
    default void shutdown() throws Exception {}
}
