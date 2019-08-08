/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.validator;

public interface LDValidator {
    /**
     * Checks whether the models on the models path are syntactically and semantically correct.
     * @throws Exception An exception which might get thrown during validation.
     */
    void validate() throws Exception;
}
