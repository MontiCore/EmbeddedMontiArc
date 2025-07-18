/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.ls2mc;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.converters.common.SourcePositions;
import de.se_rwth.commons.logging.Finding;
import org.eclipse.lsp4j.Diagnostic;

import java.util.function.Function;

@Singleton
public class DiagnosticToFinding implements Function<Diagnostic, Finding> {
    protected final DiagnosticSeverityToType ds2ft;
    protected final RangeToSourcePositions p2sps;

    @Inject
    protected DiagnosticToFinding(DiagnosticSeverityToType d2ft, RangeToSourcePositions p2sps) {
        this.ds2ft = d2ft;
        this.p2sps = p2sps;
    }

    @Override
    public Finding apply(Diagnostic diagnostic) {
        SourcePositions sourcePositions = this.p2sps.apply(diagnostic.getRange());
        Finding.Type type = this.ds2ft.apply(diagnostic.getSeverity());
        String message = diagnostic.getMessage();

        return new Finding(type, message, sourcePositions.start, sourcePositions.end);
    }
}
