/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.document;

import com.google.common.flogger.FluentLogger;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.ls.DuplexLanguageServer;
import de.monticore.lang.monticar.sol.runtime.ls.services.DiagnosticsService;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.jsonrpc.CompletableFutures;
import org.eclipse.lsp4j.services.LanguageClient;

import java.util.List;
import java.util.concurrent.CompletableFuture;

@Singleton
public class DefaultTextDocumentService implements ValidatorTextDocumentService {
    protected final FluentLogger logger;
    protected final TextDocuments documents;
    protected final DuplexLanguageServer server;
    protected final DiagnosticsService diagnostics;

    @Inject
    protected DefaultTextDocumentService(TextDocuments documents, DuplexLanguageServer server,
                                         DiagnosticsService diagnostics) {
        this.logger = FluentLogger.forEnclosingClass();
        this.documents = documents;
        this.server = server;
        this.diagnostics = diagnostics;
    }

    @Override
    public void didOpen(DidOpenTextDocumentParams params) {
        this.documents.onDidOpenTextDocumentParams(params);
        this.pushDiagnostics(params);
    }

    @Override
    public void didChange(DidChangeTextDocumentParams params) {
        this.documents.onDidChangeTextDocumentParams(params);
        this.pushDiagnostics(params);
    }

    @Override
    public void didClose(DidCloseTextDocumentParams params) {
        this.documents.onDidCloseTextDocumentParams(params);
    }

    @Override
    public void didSave(DidSaveTextDocumentParams params) {}

    @Override
    public CompletableFuture<List<Diagnostic>> validate(DidOpenTextDocumentParams params) {
        TextDocument document = this.documents.get(params.getTextDocument().getUri());

        return this.validate(document);
    }

    @Override
    public CompletableFuture<List<Diagnostic>> validate(DidChangeTextDocumentParams params) {
        TextDocument document = this.documents.get(params.getTextDocument().getUri());

        return this.validate(document);
    }

    protected CompletableFuture<List<Diagnostic>> validate(TextDocument document) {
        return CompletableFutures.computeAsync(monitor -> {
            monitor.checkCanceled();
            return this.diagnostics.validate(document);
        });
    }

    @Override
    public void pushDiagnostics(DidOpenTextDocumentParams params) {
        this.pushDiagnostics(params.getTextDocument().getUri(), this.validate(params));
    }

    @Override
    public void pushDiagnostics(DidChangeTextDocumentParams params) {
        this.pushDiagnostics(params.getTextDocument().getUri(), this.validate(params));
    }

    protected void pushDiagnostics(String uri, CompletableFuture<List<Diagnostic>> validation) {
        try {
            List<Diagnostic> diagnostics = validation.get();
            LanguageClient client = this.server.getRemoteProxy();

            this.logger.atInfo().log(diagnostics.toString());
            client.publishDiagnostics(new PublishDiagnosticsParams(uri, diagnostics));
        } catch(Exception ignored) {}
    }
}
