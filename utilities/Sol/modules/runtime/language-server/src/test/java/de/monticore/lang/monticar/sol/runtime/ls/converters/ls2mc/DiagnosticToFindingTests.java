/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

import de.monticore.lang.monticar.sol.runtime.ls.converters.common.SourcePositions;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DiagnosticSeverity;
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
public class DiagnosticToFindingTests {
    @Mock DiagnosticSeverityToType d2ft;
    @Mock RangeToSourcePositions r2sps;

    @InjectMocks DiagnosticToFinding d2f;

    @Test
    void testApply() {
        // Input
        Position startPosition = new Position(20, 45);
        Position endPosition = new Position(40, 20);
        Range range = new Range(startPosition, endPosition);
        Diagnostic diagnostic = new Diagnostic(range, "Found Error", DiagnosticSeverity.Error, "Syntax Errors");

        // Output
        SourcePosition startSourcePosition = new SourcePosition(20, 45);
        SourcePosition endSourcePosition = new SourcePosition(40, 20);
        SourcePositions sourcePositions = new SourcePositions(startSourcePosition, endSourcePosition);
        Finding finding = new Finding(Finding.Type.ERROR, "Found Error", startSourcePosition, endSourcePosition);

        // Mocks
        when(d2ft.apply(DiagnosticSeverity.Error)).thenReturn(Finding.Type.ERROR);
        when(r2sps.apply(range)).thenReturn(sourcePositions);

        // Assertion
        assertEquals(d2f.apply(diagnostic), finding, "Diagnostic has not correctly been converted.");
    }
}
