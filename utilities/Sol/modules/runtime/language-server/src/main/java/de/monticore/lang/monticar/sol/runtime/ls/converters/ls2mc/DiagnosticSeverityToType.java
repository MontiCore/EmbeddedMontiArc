/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

import com.google.inject.Singleton;
import de.se_rwth.commons.logging.Finding;
import org.eclipse.lsp4j.DiagnosticSeverity;

import java.util.function.Function;

@Singleton
public class DiagnosticSeverityToType implements Function<DiagnosticSeverity, Finding.Type> {
    protected DiagnosticSeverityToType() {}

    @Override
    public Finding.Type apply(DiagnosticSeverity diagnosticSeverity) {
        switch (diagnosticSeverity) {
            case Error: return Finding.Type.ERROR;
            default: return Finding.Type.WARNING;
        }
    }
}
