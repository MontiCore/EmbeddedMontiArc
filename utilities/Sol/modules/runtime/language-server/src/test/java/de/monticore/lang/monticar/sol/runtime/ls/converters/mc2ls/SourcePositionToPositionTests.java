/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls;

import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.Position;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SourcePositionToPositionTests {
    SourcePositionToPosition sp2p = new SourcePositionToPosition();

    @Test
    void testApply() {
        SourcePosition sourcePosition = new SourcePosition(20, 30); // Input
        Position position = new Position(19, 30); // Output

        // Assertions
        assertEquals(sp2p.apply(sourcePosition), position, "SourcePosition has not correctly been converted.");
    }
}
