/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls;

import com.google.inject.Singleton;
import de.se_rwth.commons.logging.Finding;
import org.eclipse.lsp4j.DiagnosticSeverity;

import java.util.function.Function;

@Singleton
public class TypeToDiagnosticSeverity implements Function<Finding.Type, DiagnosticSeverity> {
    protected TypeToDiagnosticSeverity() {}

    @Override
    public DiagnosticSeverity apply(Finding.Type type) {
        switch (type) {
            case ERROR: return DiagnosticSeverity.Error;
            case WARNING: return DiagnosticSeverity.Warning;
            default: return DiagnosticSeverity.Information;
        }
    }
}
