/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import org.eclipse.lsp4j.InitializeParams;
import org.eclipse.lsp4j.InitializeResult;
import org.eclipse.lsp4j.TextDocumentSyncKind;
import org.eclipse.lsp4j.TextDocumentSyncOptions;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.services.TextDocumentService;
import org.eclipse.lsp4j.services.WorkspaceService;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.util.concurrent.CompletableFuture;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

@ExtendWith(MockitoExtension.class)
public class DefaultDuplexLanguageServerTests {
    @Mock TextDocumentService documentService;
    @Mock WorkspaceService workspaceService;
    @Mock LanguageClient client;

    @InjectMocks DefaultDuplexLanguageServer server;

    @Test
    void testInitialize() throws Exception {
        CompletableFuture<InitializeResult> future = server.initialize(new InitializeParams());
        InitializeResult result = future.get();
        Either<TextDocumentSyncKind, TextDocumentSyncOptions> expected = Either.forLeft(TextDocumentSyncKind.Full);

        assertEquals(expected, result.getCapabilities().getTextDocumentSync(), "Text Document Synchronization is not Full.");
    }

    @Test
    void testShutdown() throws Exception {
        CompletableFuture<Object> future = server.shutdown();
        Object value = future.get();

        assertSame(Boolean.TRUE, value, "Object Value should be TRUE.");
    }

    @Test
    void testGetTextDocumentService() {
        assertEquals(documentService, server.getTextDocumentService(), "TextDocumentServices do not match.");
    }

    @Test
    void testGetWorkspaceService() {
        assertEquals(workspaceService, server.getWorkspaceService(), "WorkspaceServices do not match.");
    }

    @Test
    void testSetRemoteProxy() {
        server.setRemoteProxy(client);

        assertEquals(client, server.client, "LanguageClients do not match.");
    }

    @Test
    void testGetRemoteProxy() {
        server.setRemoteProxy(client);

        assertEquals(client, server.getRemoteProxy(), "LanguageClients do not match.");
    }
}
