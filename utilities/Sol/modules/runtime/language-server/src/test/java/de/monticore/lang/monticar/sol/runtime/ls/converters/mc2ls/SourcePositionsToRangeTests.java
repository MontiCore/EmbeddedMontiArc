/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls;

import de.monticore.lang.monticar.sol.runtime.ls.converters.common.SourcePositions;
import de.se_rwth.commons.SourcePosition;
import org.eclipse.lsp4j.Position;
import org.eclipse.lsp4j.Range;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class SourcePositionsToRangeTests {
    @Mock SourcePositionToPosition sp2p;

    @InjectMocks SourcePositionsToRange sp2r;

    @Test
    public void testApply() {
        // Input
        SourcePosition start = new SourcePosition(10, 20);
        SourcePosition end = new SourcePosition(20, 10);
        SourcePositions sourcePositions = new SourcePositions(start, end);

        // Output
        Position startPosition = new Position(10, 20);
        Position endPosition = new Position(20, 10);
        Range range = new Range(startPosition, endPosition);

        // Mocks
        when(sp2p.apply(start)).thenReturn(startPosition);
        when(sp2p.apply(end)).thenReturn(endPosition);

        // Assertions
        assertEquals(sp2r.apply(sourcePositions), range, "SourcePositions has not been correctly converted.");
    }
}
