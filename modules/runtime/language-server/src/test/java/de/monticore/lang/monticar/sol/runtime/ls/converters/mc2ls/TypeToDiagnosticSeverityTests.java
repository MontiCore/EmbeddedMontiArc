/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls;

import de.se_rwth.commons.logging.Finding;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertSame;

public class TypeToDiagnosticSeverityTests {
    TypeToDiagnosticSeverity t2d = new TypeToDiagnosticSeverity();

    @Test
    void testApply() {
        // Output
        DiagnosticSeverity error = DiagnosticSeverity.forValue(1);
        DiagnosticSeverity warning = DiagnosticSeverity.forValue(2);

        // Assertions
        assertSame(t2d.apply(Finding.Type.ERROR), error, "Finding.Type was not correctly converted.");
        assertSame(t2d.apply(Finding.Type.WARNING), warning, "Finding.Type was not correctly converted.");
    }
}
