package de.monticore.lang.embeddedmontiarc.lsp;

import de.monticore.ModelingLanguage;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.cocos.EmbeddedMontiArcCoCos;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcCoCoChecker;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.SymbolKind;
import de.monticore.util.lsp.MontiCoreDocumentServiceWithSymbol;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.io.StringReader;
import java.nio.file.Path;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class EmaDocumentService extends MontiCoreDocumentServiceWithSymbol<ASTEMACompilationUnit, EMAComponentSymbol> {
    private EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
    private ModelingLanguageFamily modelFamily;

    @Override
    public String getLanguageServerIdentifier() {
        return "EMA Parser";
    }

    @Override
    public Optional<ASTEMACompilationUnit> doParse(StringReader fullText) {
        Log.info("Parsing!", "default");
        try {
            return parser.parse(fullText);
        } catch (IOException e) {
            Log.error("Error parsing model: ", e);
        }
        return Optional.empty();
    }

    @Override
    protected List<String> getPackageList(ASTEMACompilationUnit node) {
        return node.getPackageList();
    }

    @Override
    protected String getSymbolName(ASTEMACompilationUnit node) {
        String name = node.getComponent().getName();
        return name.substring(0, 1).toLowerCase() + name.substring(1);
    }

    @Override
    protected String getFullSymbolName(ASTEMACompilationUnit node) {
        return String.join(".",getPackageList(node)) + "." + getSymbolName(node);
    }

    @Override
    protected void doCheckSymbolCoCos(Path sourcePath, EMAComponentSymbol sym) {
        EmbeddedMontiArcCoCoChecker checker = EmbeddedMontiArcCoCos.createChecker();
        checker.checkAll((ASTComponent) sym.getAstNode().get());
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
        return EMAComponentSymbol.KIND;
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
            ModelingLanguageFamily fam = new ModelingLanguageFamily();
            fam.addModelingLanguage(new EmbeddedMontiArcLanguage());
            fam.addModelingLanguage(new StreamLanguage());
            fam.addModelingLanguage(new StructLanguage());
        }

        return modelFamily;
    }
}
