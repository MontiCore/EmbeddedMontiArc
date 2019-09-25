/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import org.eclipse.lsp4j.InitializeParams;
import org.eclipse.lsp4j.InitializeResult;
import org.eclipse.lsp4j.ServerCapabilities;
import org.eclipse.lsp4j.TextDocumentSyncKind;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.services.TextDocumentService;
import org.eclipse.lsp4j.services.WorkspaceService;

import java.util.concurrent.CompletableFuture;

@Singleton
public class DefaultDuplexLanguageServer implements DuplexLanguageServer {
    protected final TextDocumentService documentService;
    protected final WorkspaceService workspaceService;

    protected LanguageClient client;

    @Inject
    protected DefaultDuplexLanguageServer(TextDocumentService documentService, WorkspaceService workspaceService) {
        this.documentService = documentService;
        this.workspaceService = workspaceService;
    }

    @Override
    public CompletableFuture<InitializeResult> initialize(InitializeParams params) {
        InitializeResult result = new InitializeResult(new ServerCapabilities());

        result.getCapabilities().setTextDocumentSync(TextDocumentSyncKind.Full);

        return CompletableFuture.completedFuture(result);
    }

    @Override
    public CompletableFuture<Object> shutdown() {
        return CompletableFuture.supplyAsync(() -> Boolean.TRUE);
    }

    @Override
    public void exit() {
        System.exit(0);
    }

    @Override
    public TextDocumentService getTextDocumentService() {
        return this.documentService;
    }

    @Override
    public WorkspaceService getWorkspaceService() {
        return this.workspaceService;
    }

    @Override
    public void setRemoteProxy(LanguageClient client) {
        this.client = client;
    }

    @Override
    public LanguageClient getRemoteProxy() {
        return this.client;
    }
}
