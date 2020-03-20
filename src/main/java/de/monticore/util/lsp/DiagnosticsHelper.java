/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.DiagnosticsLog;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.services.LanguageClient;

import java.util.ArrayList;
import java.util.List;

public class DiagnosticsHelper{
    private String languageServerIdentifier;
    private LanguageClient client;

    public DiagnosticsHelper(LanguageClient client, String languageServerIdentifier) {
        this.client = client;
        this.languageServerIdentifier = languageServerIdentifier;
    }

    protected void publishFindingsFromLog(VSCodeUri originalUri) {
        List<Finding> findings = DiagnosticsLog.getFindings();
        List<Diagnostic> diagnostics = new ArrayList<>(findingsToDiagnostics(findings));
        publishDiagnostics(originalUri, diagnostics);
    }

    protected void publishDiagnostics(VSCodeUri originalUri, List<Diagnostic> diagnostics) {
        PublishDiagnosticsParams publishDiagnosticsParams = new PublishDiagnosticsParams();
        publishDiagnosticsParams.setUri(originalUri.getEncodedString());
        publishDiagnosticsParams.setDiagnostics(diagnostics);
        Log.debug("Publishing diagnostics!:" + publishDiagnosticsParams, "default");
        client.publishDiagnostics(publishDiagnosticsParams);
    }

    protected List<Diagnostic> findingsToDiagnostics(List<Finding> findings) {
        List<Diagnostic> diagnosticList = new ArrayList<Diagnostic>();
        for (Finding finding : findings) {
            Log.debug("Process finding: " + finding.toString(), "findings");
            Position start = new Position(0, 0);
            Log.debug("Source possition: " + finding.getSourcePosition().toString(), "findings");
            if (finding.getSourcePosition().isPresent()) {
                SourcePosition sourcePosition = finding.getSourcePosition().get();
                start.setLine(sourcePosition.getLine() - 1);
                start.setCharacter(sourcePosition.getColumn());
            }
            Position end = new Position(start.getLine(), 100);
            Diagnostic e = new Diagnostic(new Range(start, end), finding.getMsg());
            e.setSeverity(DiagnosticSeverity.Error);
            e.setSource(languageServerIdentifier);
            diagnosticList.add(e);
        }
        return diagnosticList;
    }
}
