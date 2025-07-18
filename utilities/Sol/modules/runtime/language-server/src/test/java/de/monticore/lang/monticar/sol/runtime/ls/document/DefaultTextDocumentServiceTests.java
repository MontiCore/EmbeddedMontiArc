/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.document;

import de.monticore.lang.monticar.sol.runtime.ls.ls.DuplexLanguageServer;
import de.monticore.lang.monticar.sol.runtime.ls.services.DiagnosticsService;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.services.LanguageClient;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
public class DefaultTextDocumentServiceTests {
    final String uri = "file:///some/uri.java";

    @Mock TextDocuments documents;
    @Mock DuplexLanguageServer server;
    @Mock DiagnosticsService diagnostics;
    @Mock LanguageClient client;
    @Mock TextDocument document;
    @Mock VersionedTextDocumentIdentifier versionedDocument;

    @InjectMocks DefaultTextDocumentService service;

    @Test
    void testDidOpen() {
        DidOpenTextDocumentParams params = mock(DidOpenTextDocumentParams.class);

        when(params.getTextDocument()).thenReturn(document);
        when(documents.get(uri)).thenReturn(document);
        when(document.getUri()).thenReturn(uri);
        when(server.getRemoteProxy()).thenReturn(client);

        service.didOpen(params);

        verify(documents).onDidOpenTextDocumentParams(params);
    }

    @Test
    void testDidChange() {
        DidChangeTextDocumentParams params = mock(DidChangeTextDocumentParams.class);

        when(params.getTextDocument()).thenReturn(versionedDocument);
        when(versionedDocument.getUri()).thenReturn(uri);
        when(server.getRemoteProxy()).thenReturn(client);

        service.didChange(params);

        verify(documents).onDidChangeTextDocumentParams(params);
    }

    @Test
    void testDidClose() {
        DidCloseTextDocumentParams params = mock(DidCloseTextDocumentParams.class);

        service.didClose(params);

        verify(documents).onDidCloseTextDocumentParams(params);
    }

    @Test
    void testDidSave() {
        service.didSave(null);
    }

    @Test
    void testValidate() throws Exception {
        List<Diagnostic> diagnostics = new ArrayList<>();
        DidOpenTextDocumentParams openParams = mock(DidOpenTextDocumentParams.class);
        DidChangeTextDocumentParams changeParams = mock(DidChangeTextDocumentParams.class);

        when(openParams.getTextDocument()).thenReturn(document);
        when(changeParams.getTextDocument()).thenReturn(versionedDocument);
        when(documents.get(uri)).thenReturn(document);
        when(document.getUri()).thenReturn(uri);
        when(versionedDocument.getUri()).thenReturn(uri);

        assertEquals(diagnostics, service.validate(openParams).get(), "Diagnostics do not match.");
        assertEquals(diagnostics, service.validate(changeParams).get(), "Diagnostics do not match.");
        assertEquals(diagnostics, service.validate(document).get(), "Diagnostics do not match.");
    }

    @Test
    void testPushDiagnostics() {
        List<Diagnostic> diagnostics = new ArrayList<>();
        CompletableFuture<List<Diagnostic>> validation = CompletableFuture.completedFuture(diagnostics);
        DidOpenTextDocumentParams openParams = mock(DidOpenTextDocumentParams.class);
        DidChangeTextDocumentParams changeParams = mock(DidChangeTextDocumentParams.class);
        PublishDiagnosticsParams params = new PublishDiagnosticsParams(uri, diagnostics);

        when(openParams.getTextDocument()).thenReturn(document);
        when(changeParams.getTextDocument()).thenReturn(versionedDocument);
        when(documents.get(uri)).thenReturn(document);
        when(document.getUri()).thenReturn(uri);
        when(versionedDocument.getUri()).thenReturn(uri);
        when(server.getRemoteProxy()).thenReturn(client);

        service.pushDiagnostics(openParams);

        verify(client).publishDiagnostics(params);

        service.pushDiagnostics(changeParams);

        verify(client, times(2)).publishDiagnostics(params);

        service.pushDiagnostics(uri, validation);

        verify(client, times(3)).publishDiagnostics(params);
    }
}
