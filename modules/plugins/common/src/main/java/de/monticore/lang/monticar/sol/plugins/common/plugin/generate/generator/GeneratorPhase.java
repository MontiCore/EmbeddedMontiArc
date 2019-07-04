/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
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
     * @throws Exception Exception which is thrown if the phase cannot be executed.
     */
    default void canExecute() throws Exception {}

    /**
     * @return True if the phase **should** be executed, false otherwise. Can be used to compare input against state.
     */
    default boolean shouldExecute() {
        return true;
    }

    /**
     * Triggers the execution of this generation phase.
     * @throws Exception An exception which might occur during this generation phase.
     */
    void execute(GeneratorEngine engine) throws Exception;
}
