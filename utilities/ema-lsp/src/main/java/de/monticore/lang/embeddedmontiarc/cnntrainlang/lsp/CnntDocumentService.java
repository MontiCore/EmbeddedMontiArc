package de.monticore.lang.embeddedmontiarc.cnntrainlang.lsp;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainCompilationUnit;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.cnntrain._parser.CNNTrainParser;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainLanguage;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
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
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class CnntDocumentService extends MontiCoreDocumentServiceWithSymbol<ASTCNNTrainCompilationUnit, CNNTrainCompilationUnitSymbol> {
    private CNNTrainParser parser = new CNNTrainParser();
    private ModelingLanguageFamily modelFamily;

    public CnntDocumentService(@NotNull ModelFileCache modelFileCache) {
        super(modelFileCache);
    }

    @Override
    public String getLanguageServerIdentifier() {
        return "CNNTrainLang Parser";
    }

    @Override
    public Optional<ASTCNNTrainCompilationUnit> doParse(StringReader fullText) {
        Log.info("Parsing!", "default");
        try {
            return parser.parse(fullText);
        } catch (IOException e) {
            Log.error("Error parsing model: ", e);
        }
        return Optional.empty();
    }

    @Override
    protected List<String> getPackageList(ASTCNNTrainCompilationUnit node) {
        return new ArrayList<>();
    }

    @Override
    protected LookaheadProvider getLookaheadProvider() {
        return null;
    }

    @Override
    protected String getSymbolName(ASTCNNTrainCompilationUnit node) {
        return node.getName();
    }

    @Override
    protected String getFullSymbolName(ASTCNNTrainCompilationUnit node) {
        return getSymbolName(node);
    }

    @Override
    protected void doCheckSymbolCoCos(Path sourcePath, CNNTrainCompilationUnitSymbol sym) {
        // CNNTrainCocos checker = new CNNTrainCocos();
        // checker.checkAll((CNNTrainCompilationUnitSymbol) sym.getAstNode().get());
        // CNNTrainCompilationUnitSymbol comp = (CNNTrainCompilationUnitSymbol) sym.getAstNode().get();
        ASTCNNTrainCompilationUnit astComp = (ASTCNNTrainCompilationUnit) sym.getAstNode().get();
        CNNTrainCocos.createChecker().checkAll(astComp);
        if (de.monticore.lang.math.LogConfig.getFindings().isEmpty()) {
            Log.info("No CoCos invalid", "default");
        } else {
            Log.info("Findings: " + de.monticore.lang.math.LogConfig.getFindings(), "default");
        }
    }

    @Override
    protected Set<String> getModelFileExtensions() {
        return getModelingLanguageFamily().getModelingLanguages().stream().map(ModelingLanguage::getFileExtension).collect(Collectors.toSet());
    }

    @Override
    protected SymbolKind getSymbolKind() {
        return CNNTrainCompilationUnitSymbol.KIND;
    }

    @Override
    public Scope createSymTab(Path... modelPath) {
        ConstantPortHelper.resetLastID();

        ModelingLanguageFamily fam = getModelingLanguageFamily();
        final ModelPath mp = new ModelPath();
        for (Path m : modelPath) {
            mp.addEntry(m);
        }

        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        return scope;
    }

    protected ModelingLanguageFamily getModelingLanguageFamily() {
        if(modelFamily == null) {
            modelFamily = new ModelingLanguageFamily();
            CNNTrainLanguage montiArcCNNTrainLanguage = new CNNTrainLanguage();
            modelFamily.addModelingLanguage(montiArcCNNTrainLanguage);
            modelFamily.addModelingLanguage(new StreamUnitsLanguage());
            modelFamily.addModelingLanguage(new StructLanguage());
            modelFamily.addModelingLanguage(new EnumLangLanguage());
            modelFamily.addModelingLanguage(new EventLanguage());
        }

        return modelFamily;
    }
}
