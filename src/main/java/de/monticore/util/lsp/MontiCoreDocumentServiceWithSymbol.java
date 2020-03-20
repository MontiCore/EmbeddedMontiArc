/* (c) https://github.com/MontiCore/monticore */
package de.monticore.util.lsp;

import de.monticore.ast.ASTNode;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.monticore.util.lsp.features.completion.DefaultCompletionHandler;
import de.monticore.util.lsp.features.completion.LookaheadProvider;
import de.monticore.util.lsp.features.definition.ReflectionDefinitionHandler;
import de.monticore.util.lsp.util.ModelFileCacheProvider;
import de.se_rwth.commons.logging.DiagnosticsLog;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.eclipse.lsp4j.*;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.io.StringReader;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

public abstract class MontiCoreDocumentServiceWithSymbol<ASTType extends ASTNode, SymType extends Symbol>
        extends MontiCoreDocumentService<ASTType>
        implements ModelFileCacheProvider
{
    protected ModelFileCache modelFileCache;

    public MontiCoreDocumentServiceWithSymbol(@NotNull ModelFileCache modelFileCache) {
        this.modelFileCache = modelFileCache;
    }

    public void addDefaultHandlers() {
        addDefinitionHandler(new ReflectionDefinitionHandler<ASTType, SymType>(this));
        addCompletionHandler(new DefaultCompletionHandler(this, getLookaheadProvider()));
    }

    @NotNull
    public ModelFileCache getModelFileCache() {
        return modelFileCache;
    }

    public void setModelFileCache(@NotNull ModelFileCache modelFileCache) {
        this.modelFileCache = modelFileCache;
    }

    public Optional<ASTType> parseCached(VSCodeUri originalUri) {
        return getModelFileCache()
                .getCachedContentFor(originalUri)
                .flatMap(c -> doParse(new StringReader(c)));
    }

    public Optional<ASTType> parseCached(TextDocumentItem textDocumentItem) {
        try {
            return parseCached(new VSCodeUri(textDocumentItem));
        } catch (URISyntaxException e) {
            Log.error("Error in parseCached for textDocumentItem " + textDocumentItem, e);
        }
        return Optional.empty();
    }

    public Optional<ASTType> parseCached(TextDocumentIdentifier textDocumentIdentifier) {
        try {
            return parseCached(new VSCodeUri(textDocumentIdentifier));
        } catch (URISyntaxException e) {
            Log.error("Error in parseCached for textDocumentIdentifier " + textDocumentIdentifier, e);
        }
        return Optional.empty();
    }

    public Optional<SymType> getSymbolFor(VSCodeUri originalUri) {
        try {
            Optional<ASTType> astOpt = parse(FileUtils.readFileToString(originalUri.getFsPath().toFile(), StandardCharsets.UTF_8), originalUri);
            if (astOpt.isPresent()) {
                return getSymbol(astOpt.get(), originalUri);
            }
        } catch (IOException e) {
            Log.error("IOError", e);
        }
        return Optional.empty();
    }

    public Optional<SymType> getSymbolForCached(VSCodeUri originalUri) {
        return parseCached(originalUri).flatMap(astType -> getSymbol(astType, originalUri));
    }

    public Optional<SymType> getSymbolForCached(TextDocumentIdentifier textDocumentIdentifier) {
        try {
            return getSymbolForCached(new VSCodeUri(textDocumentIdentifier));
        } catch (URISyntaxException e) {
            Log.error("Error in getSymbolForCached for textDocumentIdentifier " + textDocumentIdentifier, e);
        }

        return Optional.empty();
    }

    public Optional<SymType> getSymbolForCached(TextDocumentItem textDocumentItem) {
        try {
            return getSymbolForCached(new VSCodeUri(textDocumentItem));
        } catch (URISyntaxException e) {
            Log.error("Error in getSymbolForCached for textDocumentItem " + textDocumentItem, e);
        }
        return Optional.empty();
    }

    private Optional<SymType> getSymbol(ASTType ast, VSCodeUri originalUri) {
        Scope symtab = this.createSymTab((this.getModelFileCache()).getTmpModelPath(originalUri));
        Log.debug("Created symtab", "default");
        DiagnosticsLog.clearAndUse();
        String fullSymbolName = this.getFullSymbolName(ast);
        return symtab.resolve(fullSymbolName, this.getSymbolKind());
    }

    @Override
    protected void checkCoCos(VSCodeUri originalUri, ASTType node) {
        super.checkCoCos(originalUri, node);
        getModelFileCache().addModelBasePath(originalUri, getPackageList(node));
        Optional<SymType> symOpt = getSymbol(node, originalUri);
        if (symOpt.isPresent()) {
            Log.debug("Resolved symbol", "default");
            SymType sym = symOpt.get();
            doCheckSymbolCoCos(originalUri, sym);
        } else {
            Log.error("Can not resolve symbol " + getFullSymbolName(node));
        }
    }

    @Override
    protected void processDocument(String fullText, VSCodeUri originalUri) {
        try {
            getModelFileCache().updateTmpContentFor(originalUri, fullText);
        } catch (IOException e) {
            Log.error("Error updating file", e);
        }
        checkForErrors(fullText, originalUri);
    }


    @Override
    protected void doCheckASTCocos(ASTType node) {
        // All cocos should be checked at doCheckSymbolCoCos
    }

    protected abstract void doCheckSymbolCoCos(VSCodeUri originalUri, SymType sym);

    protected abstract String getSymbolName(ASTType node);

    protected abstract String getFullSymbolName(ASTType node);

    protected abstract Scope createSymTab(Path... modelPath);

    protected abstract SymbolKind getSymbolKind();

    protected abstract List<String> getPackageList(ASTType node);

    protected abstract LookaheadProvider getLookaheadProvider();

    @Override
    public CompletableFuture<Either<List<? extends Location>, List<? extends LocationLink>>> definition(TextDocumentPositionParams position) {
        if (definitionHandler != null) {
            return definitionHandler.definition(position);
        } else {
            return CompletableFuture.completedFuture(null);
        }
    }
}
