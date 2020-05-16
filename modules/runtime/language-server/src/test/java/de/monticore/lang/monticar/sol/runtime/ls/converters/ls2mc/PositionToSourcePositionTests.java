/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.Position;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class PositionToSourcePositionTests {
    PositionToSourcePosition p2sp = new PositionToSourcePosition();

    @Test
    void testApply() {
        Position position = new Position(19, 30); // Input
        SourcePosition sourcePosition = new SourcePosition(20, 30); // Output

        // Assertions
        assertEquals(p2sp.apply(position), sourcePosition, "Position has not correctly been converted.");
    }
}
