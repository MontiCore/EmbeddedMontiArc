package de.monticore.lang.embeddedmontiarc.cnnarchlang.lsp;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.lang.monticar.cnnarch._ast.ASTCNNArchCompilationUnit;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCoCoChecker;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchCocos;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchSymbolCoCoChecker;
import de.monticore.lang.monticar.cnnarch._parser.CNNArchParser;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchLanguage;
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

public class CnnaDocumentService extends MontiCoreDocumentServiceWithSymbol<ASTCNNArchCompilationUnit, CNNArchCompilationUnitSymbol> {
    private CNNArchParser parser = new CNNArchParser();
    private ModelingLanguageFamily modelFamily;

    public CnnaDocumentService(@NotNull ModelFileCache modelFileCache) {
        super(modelFileCache);
    }

    @Override
    public String getLanguageServerIdentifier() {
        return "CNNArchLang Parser";
    }

    @Override
    public Optional<ASTCNNArchCompilationUnit> doParse(StringReader fullText) {
        Log.info("Parsing!", "default");
        try {
            // of type ASTCNNArchCompilationUnit
            return parser.parse(fullText);
        } catch (IOException e) {
            Log.error("Error parsing model: ", e);
        }
        return Optional.empty();
    }

    @Override
    protected List<String> getPackageList(ASTCNNArchCompilationUnit node) {
        return new ArrayList<>();
    }

    @Override
    protected LookaheadProvider getLookaheadProvider() {
        return null;
    }

    @Override
    protected String getSymbolName(ASTCNNArchCompilationUnit node) {
        return node.getName();
    }

    @Override
    protected String getFullSymbolName(ASTCNNArchCompilationUnit node) {
        return getSymbolName(node);
    }

    @Override
    protected void doCheckSymbolCoCos(Path sourcePath, CNNArchCompilationUnitSymbol sym) {
        // write doCheckSymbolCoCos
        // /home/treiber/ema/languages/CNNArchLang/src/test/java/de/monticore/lang/monticar/cnnarch/cocos/AbstractCoCoTest.java

        CNNArchCoCoChecker astChecker = CNNArchCocos.createASTChecker();
        CNNArchSymbolCoCoChecker preResolveCocos = CNNArchCocos.createCNNArchPreResolveSymbolChecker();
        CNNArchSymbolCoCoChecker postResolveCocos = CNNArchCocos.createCNNArchPostResolveSymbolChecker();

        int findings = Log.getFindings().size();

        astChecker.checkAll((ASTCNNArchCompilationUnit) sym.getAstNode().get());
        System.out.print((ASTCNNArchCompilationUnit) sym.getAstNode().get());

        if (findings == Log.getFindings().size()) {
            preResolveCocos.checkAll(sym);
            if (findings == Log.getFindings().size()) {
                sym.getArchitecture().resolve();
                if (findings == Log.getFindings().size()) {
                    postResolveCocos.checkAll(sym);
                }
            }
        }

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
        return CNNArchCompilationUnitSymbol.KIND;
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
            CNNArchLanguage montiArcCNNArchLanguage = new CNNArchLanguage();
            modelFamily.addModelingLanguage(montiArcCNNArchLanguage);
            modelFamily.addModelingLanguage(new StreamUnitsLanguage());
            modelFamily.addModelingLanguage(new StructLanguage());
            modelFamily.addModelingLanguage(new EnumLangLanguage());
            modelFamily.addModelingLanguage(new EventLanguage());
        }

        return modelFamily;
    }
}
