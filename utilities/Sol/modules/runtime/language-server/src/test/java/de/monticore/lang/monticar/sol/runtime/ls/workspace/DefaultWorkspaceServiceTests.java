/*
 * (c) https://github.com/MontiCore/monticore
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
