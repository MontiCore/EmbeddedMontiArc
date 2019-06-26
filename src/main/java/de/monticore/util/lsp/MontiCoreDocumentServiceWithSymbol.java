package de.monticore.util.lsp;

import de.monticore.ast.ASTNode;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.logging.DiagnosticsLog;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

public abstract class MontiCoreDocumentServiceWithSymbol<ASTType extends ASTNode, SymType extends Symbol> extends MontiCoreDocumentService<ASTType> {

    protected Optional<Path> modelBasePath = Optional.empty();
    protected Optional<ModelFileCache> modelFileCache = Optional.empty();

    public Optional<Path> getModelBasePath() {
        return modelBasePath;
    }

    public void setModelBasePath(Path modelBasePath) {
        this.modelBasePath = Optional.ofNullable(modelBasePath);
    }

    public Optional<ModelFileCache> getModelFileCache() {
        return modelFileCache;
    }

    public void setModelFileCache(ModelFileCache modelFileCache) {
        this.modelFileCache = Optional.of(modelFileCache);
    }

    @Override
    protected void checkCoCos(Path sourcePath, ASTType node) {
        super.checkCoCos(sourcePath, node);
        updateModelBasePath(sourcePath, getPackageList(node));
        if (getModelFileCache().isPresent()) {
            Scope symtab = createSymTab(getModelFileCache().get().getTmpModelPath());
            Log.debug("Created symtab", "default");
            DiagnosticsLog.clearAndUse();
            String fullSymbolName = getFullSymbolName(node);
            Optional<SymType> symOpt = symtab.resolve(fullSymbolName, getSymbolKind());
            if (symOpt.isPresent()) {
                Log.debug("Resolved symbol", "default");
                SymType sym = symOpt.get();
                doCheckSymbolCoCos(sourcePath, sym);
            } else {
                Log.error("Can not resolve symbol " + fullSymbolName);
            }
        }
    }

    protected void updateModelBasePath(Path sourcePath, List<String> packageList) {
        Optional<Path> basePathOpt = ModelPathHelper.getBasePath(sourcePath, packageList);
        if (basePathOpt.isPresent()) {
            setModelBasePath(basePathOpt.get());
            Log.info("modelBasePath: " + basePathOpt.get().toString(), "debug");
            try {
                copyModelOnce();
            } catch (IOException e) {
                setModelBasePath(null);
                Log.error("Error copying the model to tmp dir", e);
            }

        } else {
            setModelBasePath(null);
        }
    }

    protected void copyModelOnce() throws IOException {
        if (!getModelFileCache().isPresent()) {
            if (getModelBasePath().isPresent()) {
                setModelFileCache(new ModelFileCache(getModelBasePath().get(), getModelFileExtensions()));
            } else {
                Log.error("ModelBasePath is not set => can not initialze model file cache!");
            }
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

        if (getModelFileCache().isPresent()) {
            try {
                getModelFileCache().get().updateTmpContentFor(sourcePath, fullText);
            } catch (IOException e) {
                Log.error("Error updating file", e);
            }
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


}
