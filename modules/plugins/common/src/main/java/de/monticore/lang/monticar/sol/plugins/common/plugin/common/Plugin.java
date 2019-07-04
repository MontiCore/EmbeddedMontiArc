/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common;

import com.google.inject.Module;

public interface Plugin {
    /**
     * @return The module in which the bindings are configured.
     */
    Module getModule();
}
