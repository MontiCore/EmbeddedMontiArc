/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.runtime.ls.workspace;

import org.junit.jupiter.api.Test;

public class DefaultWorkspaceServiceTests {
    DefaultWorkspaceService workspaceService = new DefaultWorkspaceService();

    @Test
    void testDidChangeConfiguration() {
        workspaceService.didChangeConfiguration(null);
    }

    @Test
    void testDidChangeWatchedFiles() {
        workspaceService.didChangeWatchedFiles(null);
    }
}
