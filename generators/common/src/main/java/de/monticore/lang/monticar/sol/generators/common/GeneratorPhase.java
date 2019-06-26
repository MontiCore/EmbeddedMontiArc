/*
 * Copyright (C) 2019 SE RWTH.
 *
 * TODO: Include License.
 */
package de.monticore.lang.monticar.sol.generators.common;

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
     * Triggers the execution of this generation phase.
     * @throws Exception An exception which might occur during this generation phase.
     */
    void execute() throws Exception;
}
