/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

import de.se_rwth.commons.logging.Finding;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertSame;

public class DiagnosticSeverityToTypeTests {
    final DiagnosticSeverityToType ds2t = new DiagnosticSeverityToType();

    @Test
    void testApply() {
        // Input
        DiagnosticSeverity error = DiagnosticSeverity.forValue(1);
        DiagnosticSeverity warning = DiagnosticSeverity.forValue(2);
        DiagnosticSeverity information = DiagnosticSeverity.forValue(3);

        // Assertions
        assertSame(ds2t.apply(error), Finding.Type.ERROR, "DiagnosticSeverity was not correctly converted.");
        assertSame(ds2t.apply(warning), Finding.Type.WARNING, "DiagnosticSeverity was not correctly converted.");
        assertSame(ds2t.apply(information), Finding.Type.WARNING, "DiagnosticSeverity was not correctly converted.");
    }
}
