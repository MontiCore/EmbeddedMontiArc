/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

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
public class RangeToSourcePositionsTest {
    @Mock PositionToSourcePosition p2sp;

    @InjectMocks RangeToSourcePositions r2sp;

    @Test
    void testApply() {
        // Input
        Position startPosition = new Position(10, 20);
        Position endPosition = new Position(20, 10);
        Range range = new Range(startPosition, endPosition);

        // Output
        SourcePosition start = new SourcePosition(10, 20);
        SourcePosition end = new SourcePosition(20, 10);
        SourcePositions sourcePositions = new SourcePositions(start, end);

        // Mocks
        when(p2sp.apply(startPosition)).thenReturn(start);
        when(p2sp.apply(endPosition)).thenReturn(end);

        // Assertions
        assertEquals(r2sp.apply(range), sourcePositions, "Range has not been correctly converted.");
    }
}
