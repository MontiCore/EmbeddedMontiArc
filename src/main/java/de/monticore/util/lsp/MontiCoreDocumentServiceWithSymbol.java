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

    public Optional<ASTType> parseCached(String uriString) {
        return getModelFileCache()
                .getCachedContentFor(uriString)
                .flatMap(c -> doParse(new StringReader(c)));
    }

    public Optional<ASTType> parseCached(TextDocumentItem textDocumentItem) {
        return parseCached(textDocumentItem.getUri());
    }

    public Optional<ASTType> parseCached(TextDocumentIdentifier textDocumentIdentifier) {
        return parseCached(textDocumentIdentifier.getUri());
    }

    public Optional<SymType> getSymbolFor(Path sourcePath) {
        try {
            Optional<ASTType> astOpt = parse(FileUtils.readFileToString(sourcePath.toFile(), StandardCharsets.UTF_8), sourcePath.toString());
            if (astOpt.isPresent()) {
                return getSymbol(astOpt.get(), sourcePath);
            }
        } catch (IOException e) {
            Log.error("IOError", e);
        }
        return Optional.empty();
    }

    public Optional<SymType> getSymbolForCached(String uriString) {
        Optional<ASTType> astOpt = parseCached(uriString);
        if (astOpt.isPresent()) {
            try {
                return getSymbol(astOpt.get(), ModelPathHelper.pathFromUriString(uriString));
            } catch (URISyntaxException e) {
                Log.error("Error parsing Uri", e);
            }
        }

        return Optional.empty();
    }

    public Optional<SymType> getSymbolForCached(TextDocumentIdentifier textDocumentIdentifier) {
        return getSymbolForCached(textDocumentIdentifier.getUri());
    }

    public Optional<SymType> getSymbolForCached(TextDocumentItem textDocumentItem) {
        return getSymbolForCached(textDocumentItem.getUri());
    }

    private Optional<SymType> getSymbol(ASTType ast, Path basePath) {
        Scope symtab = this.createSymTab((this.getModelFileCache()).getTmpModelPath(basePath));
        Log.debug("Created symtab", "default");
        DiagnosticsLog.clearAndUse();
        String fullSymbolName = this.getFullSymbolName(ast);
        return symtab.resolve(fullSymbolName, this.getSymbolKind());
    }

    @Override
    protected void checkCoCos(Path sourcePath, ASTType node) {
        super.checkCoCos(sourcePath, node);
        addModelBasePath(sourcePath, getPackageList(node));
        Optional<SymType> symOpt = getSymbol(node, sourcePath);
        if (symOpt.isPresent()) {
            Log.debug("Resolved symbol", "default");
            SymType sym = symOpt.get();
            doCheckSymbolCoCos(sourcePath, sym);
        } else {
            Log.error("Can not resolve symbol " + getFullSymbolName(node));
        }
    }

    protected void addModelBasePath(Path sourcePath, List<String> packageList) {
        Optional<Path> basePathOpt = ModelPathHelper.getBasePath(sourcePath, packageList);
        if (basePathOpt.isPresent()) {
            getModelFileCache().addModelBasePath(basePathOpt.get());
            Log.info("modelBasePath: " + basePathOpt.get().toString(), "debug");
        }
    }

    @Override
    protected void processDocument(String fullText, String uri) {
        Path sourcePath = null;
        try {
            sourcePath = ModelPathHelper.pathFromUriString(uri);
        } catch (URISyntaxException e) {
            Log.error("Error parsing uri to path", e);
            return;
        }

        try {
            getModelFileCache().updateTmpContentFor(sourcePath, fullText);
        } catch (IOException e) {
            Log.error("Error updating file", e);
        }
        checkForErrors(fullText, sourcePath);
    }


    @Override
    protected void doCheckASTCocos(ASTType node) {
        // All cocos should be checked at doCheckSymbolCoCos
    }

    protected abstract void doCheckSymbolCoCos(Path sourcePath, SymType sym);

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
