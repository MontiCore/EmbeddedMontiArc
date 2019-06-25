package de.monticore.lang.monticar.borealis.runtime.language.document;

import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DidChangeTextDocumentParams;
import org.eclipse.lsp4j.DidOpenTextDocumentParams;
import org.eclipse.lsp4j.services.TextDocumentService;

import java.util.List;
import java.util.concurrent.CompletableFuture;

public interface ValidatorTextDocumentService extends TextDocumentService {
    CompletableFuture<List<Diagnostic>> validate(DidOpenTextDocumentParams params);
    CompletableFuture<List<Diagnostic>> validate(DidChangeTextDocumentParams params);

    void pushDiagnostics(DidOpenTextDocumentParams params);
    void pushDiagnostics(DidChangeTextDocumentParams params);
}
