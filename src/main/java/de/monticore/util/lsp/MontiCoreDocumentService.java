/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.monticore.ast.ASTNode;
import de.monticore.util.lsp.features.completion.CompletionHandler;
import de.monticore.util.lsp.features.completion.MultiCompletionHandler;
import de.monticore.util.lsp.features.definition.DefinitionHandler;
import de.monticore.util.lsp.features.definition.MultiDefinitionHandler;
import de.se_rwth.commons.logging.DiagnosticsLog;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.IOUtils;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.services.LanguageClient;

import java.io.IOException;
import java.io.StringReader;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.CompletableFuture;

public abstract class MontiCoreDocumentService<ASTType extends ASTNode> implements ClientAwareTextDocumentService {
    protected Optional<DiagnosticsHelper> diagnosticsHelper = Optional.empty();
    protected Optional<LanguageClient> client = Optional.empty();
    protected MultiCompletionHandler completionHandler = new MultiCompletionHandler();
    protected MultiDefinitionHandler definitionHandler = new MultiDefinitionHandler();

    public MultiCompletionHandler getCompletionHandler() {
        return completionHandler;
    }

    public boolean addCompletionHandler(CompletionHandler completionHandler) {
        return this.completionHandler.add(completionHandler);
    }

    public MultiDefinitionHandler getDefinitionHandler() {
        return definitionHandler;
    }

    public boolean addDefinitionHandler(DefinitionHandler definitionHandler) {
        return this.definitionHandler.add(definitionHandler);
    }

    public void setClient(LanguageClient client){
        connect(client);
    }

    @Override
    public void connect(LanguageClient client) {
        Log.debug("setting client","DocumentService");
        this.client = Optional.ofNullable(client);
        setDiagnosticsHelper(new DiagnosticsHelper(client, getLanguageServerIdentifier()));
    }

    public Optional<LanguageClient> getClient() {
        return client;
    }

    public Optional<DiagnosticsHelper> getDiagnosticsHelper() {
        return diagnosticsHelper;
    }

    public void setDiagnosticsHelper(DiagnosticsHelper diagnosticsHelper) {
        Log.debug("setting diagnosticsHelper","DocumentService");
        this.diagnosticsHelper = Optional.of(diagnosticsHelper);
    }

    public MontiCoreDocumentService() {
        Log.info("Init TextDocumentService", "default");
    }

    protected abstract String getLanguageServerIdentifier();

    protected abstract Optional<ASTType> doParse(StringReader fullText);

    protected abstract Set<String> getModelFileExtensions();

    protected abstract void doCheckASTCocos(ASTType node);

    @Override
    public void didOpen(DidOpenTextDocumentParams didOpenTextDocumentParams) {
        Log.debug("call didOpen: " + didOpenTextDocumentParams.toString(), "default");
        if (getClient().isPresent()) {
            Path sourcePath;
            String fullText = didOpenTextDocumentParams.getTextDocument().getText();
            String uri = didOpenTextDocumentParams.getTextDocument().getUri();
            processDocument(fullText, uri);
        }
    }

    @Override
    public void didChange(DidChangeTextDocumentParams didChangeTextDocumentParams) {
        Log.debug("call didChange: " + didChangeTextDocumentParams.toString(), "default");
        if (getClient().isPresent()) {
            String fullText = didChangeTextDocumentParams.getContentChanges().get(0).getText();
            String uri = didChangeTextDocumentParams.getTextDocument().getUri();
            processDocument(fullText, uri);
        }
    }

    protected void processDocument(String fullText, String uri) {
        Path sourcePath = null;
        try {
            sourcePath = ModelPathHelper.pathFromUriString(uri);
        } catch (URISyntaxException e) {
            Log.error("Error parsing uri to path", e);
            return;
        }
        checkForErrors(fullText, sourcePath);
    }

    public void checkForErrors(String fullText, Path sourcePath) {
        DiagnosticsLog.clearAndUse();
        Optional<ASTType> tmp = parse(fullText, sourcePath.toString());
        if (tmp.isPresent()) {
            checkCoCos(sourcePath, tmp.get());
        }
        if (diagnosticsHelper.isPresent()) {
            diagnosticsHelper.get().publishFindingsFromLog(sourcePath);
        } else {
            Log.error("diagnosticsHelper not initialized!");
        }
    }

    protected void checkCoCos(Path sourcePath, ASTType node) {
        doCheckASTCocos(node);
    }

    public Optional<ASTType> parse(String fullText, String sourcePath) {
        Optional<ASTType> parseRes = Optional.empty();
        try {
            parseRes = doParse(new StringReader(fullText));
        } catch (Exception e) {
            DiagnosticsLog.error("Exception while parsing: ", e);
        }

        if (Log.getFindings().isEmpty()) {
            Log.debug("Parsed successfully", "default");
            return parseRes;
        } else {
            return Optional.empty();
        }
    }

    public Optional<ASTType> doParse(String uri) throws IOException {
        return doParse(new StringReader(IOUtils.toString(URI.create(uri), StandardCharsets.UTF_8)));
    }

    @Override
    public void didClose(DidCloseTextDocumentParams didCloseTextDocumentParams) {
        //IGNORE
    }

    @Override
    public void didSave(DidSaveTextDocumentParams didSaveTextDocumentParams) {
        //IGNORE
    }

    @Override
    public CompletableFuture<Either<List<CompletionItem>, CompletionList>> completion(CompletionParams position) {
        return completionHandler != null ? completionHandler.completion(position) : CompletableFuture.completedFuture(Either.forLeft(new ArrayList<>()));
    }
}
