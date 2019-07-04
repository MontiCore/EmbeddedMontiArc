/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.hc;

import java.nio.file.Path;

public interface HandCodeService {
    /**
     * @param generated The path where the generated part will be generated to.
     * @return True if the generated path has a handwritten counterpart, false otherwise.
     */
    boolean existsHandCodedPeer(Path generated);
}
