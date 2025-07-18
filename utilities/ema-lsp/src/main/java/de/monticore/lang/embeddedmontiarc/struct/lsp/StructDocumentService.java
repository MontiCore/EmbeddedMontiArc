package de.monticore.lang.embeddedmontiarc.struct.lsp;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.Utils;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._ast.ASTStructNode;
import de.monticore.lang.monticar.struct._cocos.StructCoCoChecker;
import de.monticore.lang.monticar.struct._parser.StructParser;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructSymbol;
import de.monticore.lang.monticar.struct.coco.DefaultStructCoCoChecker;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;
import de.monticore.util.lsp.ModelFileCache;
import de.monticore.util.lsp.MontiCoreDocumentServiceWithSymbol;
import de.monticore.util.lsp.features.completion.LookaheadProvider;
import de.se_rwth.commons.logging.Log;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.io.StringReader;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class StructDocumentService extends MontiCoreDocumentServiceWithSymbol<ASTStructCompilationUnit, StructSymbol> {
    private StructParser parser = new StructParser();
    private ModelingLanguageFamily modelFamily;

    public StructDocumentService(@NotNull ModelFileCache modelFileCache) {
        super(modelFileCache);
    }

    @Override
    protected void doCheckSymbolCoCos(Path path, StructSymbol structSymbol) {
        StructCoCoChecker checker = DefaultStructCoCoChecker.create();
        checker.checkAll((ASTStructNode) structSymbol.getAstNode().get());
        if (de.monticore.lang.math.LogConfig.getFindings().isEmpty()) {
            Log.info("No CoCos invalid", "default");
        } else {
            Log.info("Findings: " + de.monticore.lang.math.LogConfig.getFindings(), "default");
        }
    }

    @Override
    protected String getSymbolName(ASTStructCompilationUnit astStructCompilationUnit) {
        return astStructCompilationUnit.getStruct().getName();
    }

    @Override
    protected String getFullSymbolName(ASTStructCompilationUnit astStructCompilationUnit) {
        return String.join(".", astStructCompilationUnit.getPackageList()) + "." + getSymbolName(astStructCompilationUnit);
    }

    @Override
    protected Scope createSymTab(Path... paths) {
        ModelPath mp = new ModelPath();
        if (paths != null && paths.length > 0) {
            for (Path m : paths) {
                mp.addEntry(m);
            }
        }
        GlobalScope scope = new GlobalScope(mp, getModelingLanguageFamily());
        Utils.addBuiltInTypes(scope);
        return scope;
    }

    @Override
    protected SymbolKind getSymbolKind() {
        return StructSymbol.KIND;
    }

    @Override
    protected List<String> getPackageList(ASTStructCompilationUnit astStructCompilationUnit) {
        return astStructCompilationUnit.getPackageList();
    }

    @Override
    protected LookaheadProvider getLookaheadProvider() {
        return null;
    }

    @Override
    protected String getLanguageServerIdentifier() {
        return "Struct Language Server";
    }

    @Override
    protected Optional<ASTStructCompilationUnit> doParse(StringReader fullText) {
        Log.info("Parsing!", "default");
        try {
            return parser.parse(fullText);
        } catch (IOException e) {
            Log.error("Error parsing model: ", e);
        }
        return Optional.empty();
    }

    @Override
    protected Set<String> getModelFileExtensions() {
        return getModelingLanguageFamily().getModelingLanguages().stream().map(ModelingLanguage::getFileExtension).collect(Collectors.toSet());
    }

    protected ModelingLanguageFamily getModelingLanguageFamily() {
        if(modelFamily == null) {
            modelFamily = new ModelingLanguageFamily();
            modelFamily.addModelingLanguage(new StructLanguage());
            modelFamily.addModelingLanguage(new EnumLangLanguage());
        }

        return modelFamily;
    }
}
