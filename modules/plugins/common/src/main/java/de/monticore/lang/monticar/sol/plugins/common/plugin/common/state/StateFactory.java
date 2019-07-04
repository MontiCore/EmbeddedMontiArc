/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.state;

import java.io.IOException;

public interface StateFactory {
    State create(String identifier) throws IOException;
}
