/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.converters.mc2ls;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.converters.common.SourcePositions;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.lsp4j.Range;

import java.util.function.Function;

@Singleton
public class FindingToDiagnostic implements Function<Finding, Diagnostic> {
    protected final TypeToDiagnosticSeverity ft2ds;
    protected final SourcePositionsToRange sps2r;

    @Inject
    protected FindingToDiagnostic(TypeToDiagnosticSeverity ft2ds, SourcePositionsToRange sps2r) {
        this.ft2ds = ft2ds;
        this.sps2r = sps2r;
    }

    @Override
    public Diagnostic apply(Finding finding) {
        SourcePosition defaultSourcePosition = SourcePosition.getDefaultSourcePosition();
        SourcePosition start = finding.getSourcePosition().orElse(defaultSourcePosition);
        SourcePosition end = finding.getSourcePositionEnd().orElse(defaultSourcePosition);

        DiagnosticSeverity severity = this.ft2ds.apply(finding.getType());
        String message = finding.getMsg();
        Range range = this.sps2r.apply(new SourcePositions(start, end));
        Diagnostic diagnostic = new Diagnostic(range, message);

        diagnostic.setSeverity(severity);

        return diagnostic;
    }
}
