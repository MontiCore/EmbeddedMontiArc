package de.monticore.lang.monticar.borealis.runtime.language.converters.ls2mc;

import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.Position;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class PositionToSourcePositionTests {
    PositionToSourcePosition p2sp = new PositionToSourcePosition();

    @Test
    void testApply() {
        // Input
        Position position = new Position(20, 30);
        SourcePosition sourcePosition = new SourcePosition(20, 30);

        // Assertions
        assertEquals(p2sp.apply(position), sourcePosition, "Position has not correctly been converted.");
    }
}
