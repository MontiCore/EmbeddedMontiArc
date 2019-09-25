/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.document;

import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DidChangeTextDocumentParams;
import org.eclipse.lsp4j.DidOpenTextDocumentParams;
import org.eclipse.lsp4j.jsonrpc.validation.NonNull;
import org.eclipse.lsp4j.services.TextDocumentService;

import java.util.List;
import java.util.concurrent.CompletableFuture;

public interface ValidatorTextDocumentService extends TextDocumentService {
    /**
     * @param params The event parameters sent when a text document has been opened.
     * @return Diagnostics about the newly opened text document.
     */
    CompletableFuture<List<Diagnostic>> validate(@NonNull DidOpenTextDocumentParams params);

    /**
     * @param params The event parameters sent when a text document has been changed.
     * @return Diagnostics about the changed text document.
     */
    CompletableFuture<List<Diagnostic>> validate(@NonNull DidChangeTextDocumentParams params);

    /**
     * Sends diagnostics computed with {@link #validate(DidOpenTextDocumentParams) validate} to the client.
     * @param params The event parameters sent when a text document has been opened.
     */
    void pushDiagnostics(@NonNull DidOpenTextDocumentParams params);

    /**
     * Sends diagnostics computed with {@link #validate(DidOpenTextDocumentParams) validate} to the client.
     * @param params The event parameters sent when a text document has been changed.
     */
    void pushDiagnostics(@NonNull DidChangeTextDocumentParams params);
}
